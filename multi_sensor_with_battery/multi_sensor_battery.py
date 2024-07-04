import asyncio
import csv
import struct
from datetime import datetime
from bleak import BleakClient, BleakScanner
import logging
import os

# Sensor configurations
SENSOR_CONFIGS = {
    1: {
        "name": "Sense Right Hand",
        "service_uuid": "8E400004-B5A3-F393-E0A9-E50E24DCCA9E",
        "char_uuid": "8E400005-B5A3-F393-E0A9-E50E24DCCA9E",
        "reference_uuid": "8E400006-B5A3-F393-E0A9-E50E24DCCA9E",
        "prefix": "right_hand"
    },
    2: {
        "name": "Sense Left Hand",
        "service_uuid": "6E400001-B5A3-F393-E0A9-E50E24DCCA9E",
        "char_uuid": "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
        "reference_uuid": "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
        "prefix": "left_hand"
    },
    3: {
        "name": "Sense Ball",
        "service_uuid": "9E400001-C5C3-E393-B0A9-E50E24DCCA9E",
        "char_uuid": "9E400002-C5C3-E393-B0A9-E50E24DCCA9E",
        "reference_uuid": "9E400003-C5C3-E393-B0A9-E50E24DCCA9E",
        "prefix": "ball"
    },
    4: {
        "name": "Sense Right Leg",
        "service_uuid": "7E400001-A5B3-C393-D0E9-F50E24DCCA9E",
        "char_uuid": "7E400002-A5B3-C393-D0E9-F50E24DCCA9E",
        "reference_uuid": "7E400003-A5B3-C393-D0E9-F50E24DCCA9E",
        "prefix": "right_leg"
    },
    5: {
        "name": "Sense Left Leg",
        "service_uuid": "6E400001-B5C3-D393-A0F9-E50F24DCCA9E",
        "char_uuid": "6E400002-B5C3-D393-A0F9-E50F24DCCA9E",
        "reference_uuid": "6E400003-B5C3-D393-A0F9-E50F24DCCA9E",
        "prefix": "left_leg"
}
}

# Exercise configurations
EXERCISE_CONFIGS = {
    "single_hand": [1],  # Only right hand sensor
    "both_hands": [1, 2],  # Right and left hand sensors
    "hands_and_ball": [1, 2, 3],  # All sensors
    "all_sensors": [1, 2, 3, 4, 5]  # All sensors
}

csv_filename = "3_all_sensors_data.csv"

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def create_csv_file(sensor_ids):
    headers = []
    for sensor_id in sensor_ids:
        prefix = SENSOR_CONFIGS[sensor_id]["prefix"]
        headers.extend([
            f'{prefix}_timestamp',
            f'{prefix}_index',
            f'{prefix}_accel_x',
            f'{prefix}_accel_y',
            f'{prefix}_accel_z',
            f'{prefix}_gyro_x',
            f'{prefix}_gyro_y',
            f'{prefix}_gyro_z',
            f'{prefix}_battery_percentage'
        ])
    
    with open(csv_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
    logger.info(f"Created CSV file: {csv_filename} with headers: {headers}")

class SensorData:
    def __init__(self, sensor_ids):
        self.data = {i: {} for i in sensor_ids}
        self.sensor_ids = sensor_ids

    def add_data(self, sensor_index, index, values):
        self.data[sensor_index][index] = values
        logger.debug(f"Added data for sensor {sensor_index}, index {index}")

    def get_complete_data(self, index):
        return [self.data[i].get(index) for i in self.sensor_ids]

    def pop_data(self, index):
        return [self.data[i].pop(index, None) for i in self.sensor_ids]

sensor_data = None

async def notification_handler(sender, data, sensor_index):
    global sensor_data
    try:
        timestamp, index_value, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, battery_percentage = struct.unpack('<IIffffffB', data)
        timestamp_sec = timestamp / 1000
        values = [timestamp_sec, index_value] + [round(val, 4) for val in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]] + [battery_percentage]
        sensor_data.add_data(sensor_index, index_value, values)
        
        complete_data = sensor_data.get_complete_data(index_value)
        logger.debug(f"Complete data for index {index_value}: {complete_data}")
        if all(complete_data):
            combined_data = [item for sublist in complete_data for item in sublist]
            with open(csv_filename, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(combined_data)
            logger.info(f"Written combined data for index {index_value}")
            sensor_data.pop_data(index_value)
    except Exception as e:
        logger.error(f"Error in notification handler: {e}")
        
async def write_reference_timestamp(client, reference_uuid):
    reference_timestamp = int(datetime.now().timestamp())
    reference_data = struct.pack('<I', reference_timestamp)
    await client.write_gatt_char(reference_uuid, reference_data)
    logger.info(f"Written reference timestamp: {reference_timestamp}")

async def connect_and_run(device, sensor_config, sensor_index):
    async with BleakClient(device.address) as client:
        logger.info(f"Connected to {device.name}")
        await write_reference_timestamp(client, sensor_config['reference_uuid'])
        await client.start_notify(sensor_config['char_uuid'], lambda s, d: asyncio.create_task(notification_handler(s, d, sensor_index)))
        logger.info(f"Notifications started for {device.name}")
        
        while True:
            await asyncio.sleep(1)

async def main():
    global sensor_data

    # Choose exercise configuration
    print("Available exercises:")
    for exercise in EXERCISE_CONFIGS:
        print(f"- {exercise}")
    
    exercise_choice = input("Enter the exercise name: ").lower()
    if exercise_choice not in EXERCISE_CONFIGS:
        logger.error("Invalid exercise choice.")
        return

    sensor_ids = EXERCISE_CONFIGS[exercise_choice]
    logger.info(f"Selected exercise: {exercise_choice}, using sensors: {sensor_ids}")

    devices = await BleakScanner.discover()
    sense_balls = []
    
    for sensor_id in sensor_ids:
        config = SENSOR_CONFIGS[sensor_id]
        device = next((d for d in devices if d.name == config['name']), None)
        if device:
            sense_balls.append((device, config))
        else:
            logger.error(f"Sensor {config['name']} not found.")
    
    if not sense_balls:
        logger.error("No sensors found for the selected exercise.")
        return

    sensor_data = SensorData(sensor_ids)
    create_csv_file(sensor_ids)

    tasks = [connect_and_run(device, config, sensor_id) for sensor_id, (device, config) in zip(sensor_ids, sense_balls)]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Program stopped by user")
