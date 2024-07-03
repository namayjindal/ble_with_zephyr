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
    },
    2: {
        "name": "Sense Left Hand",
        "service_uuid": "6E400001-B5A3-F393-E0A9-E50E24DCCA9E",
        "char_uuid": "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
        "reference_uuid": "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    },
}

csv_filename = "2_fast_multi_sensor_data.csv"

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def create_csv_file(sensor_count):
    headers = ['Timestamp', 'Index']
    for i in range(1, sensor_count + 1):
        headers.extend([f'Accel_X{i}', f'Accel_Y{i}', f'Accel_Z{i}', f'Gyro_X{i}', f'Gyro_Y{i}', f'Gyro_Z{i}'])
    with open(csv_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
    logger.info(f"Created CSV file: {csv_filename}")

class SensorData:
    def __init__(self, sensor_count):
        self.data = {i: {} for i in range(sensor_count)}

    def add_data(self, sensor_index, index, values):
        self.data[sensor_index][index] = values
        logger.debug(f"Added data for sensor {sensor_index}, index {index}")

    def get_complete_data(self, index):
        return [self.data[i].get(index) for i in range(len(self.data))]

    def pop_data(self, index):
        return [self.data[i].pop(index, None) for i in range(len(self.data))]

sensor_data = None

async def notification_handler(sender, data, sensor_index):
    global sensor_data
    try:
        timestamp, index_value, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = struct.unpack('<IIffffff', data)
        timestamp_sec = timestamp / 1000
        values = [timestamp_sec, index_value] + [round(val, 4) for val in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]]
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
    devices = await BleakScanner.discover()
    sense_balls = []
    
    for sensor_id, config in SENSOR_CONFIGS.items():
        device = next((d for d in devices if d.name == config['name']), None)
        if device:
            sense_balls.append((device, config))
        else:
            logger.error(f"Sensor {config['name']} not found.")
    
    if not sense_balls:
        logger.error("No sensors found.")
        return

    sensor_count = len(sense_balls)
    sensor_data = SensorData(sensor_count)
    create_csv_file(sensor_count)

    tasks = [connect_and_run(device, config, i) for i, (device, config) in enumerate(sense_balls)]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Program stopped by user")
