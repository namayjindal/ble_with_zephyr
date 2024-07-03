import asyncio
import csv
from datetime import datetime
from bleak import BleakClient, BleakScanner

# UUIDs and other data
SENSOR_NAME = "Sense Ball"
SERVICE_UUID = "9E400001-C5C3-E393-B0A9-E50E24DCCA9E"
CHAR_UUID = "9E400003-C5C3-E393-B0A9-E50E24DCCA9E"

csv_filename = "4_sensor_data_with_timestamp_with_sensor_with_random_imu.csv"
STOP_FLAG = False

async def notification_handler(sender, data):
    timestamp = int.from_bytes(data[0:4], byteorder='little')
    index_value = int.from_bytes(data[4:5], byteorder='little')
    accel_x = int.from_bytes(data[5:7], byteorder='little', signed=True)
    accel_y = int.from_bytes(data[7:9], byteorder='little', signed=True)
    accel_z = int.from_bytes(data[9:11], byteorder='little', signed=True)
    gyro_x = int.from_bytes(data[11:13], byteorder='little', signed=True)
    gyro_y = int.from_bytes(data[13:15], byteorder='little', signed=True)
    gyro_z = int.from_bytes(data[15:17], byteorder='little', signed=True)

    with open(csv_filename, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, index_value, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z])
    
    print(f"Timestamp: {timestamp}, Index: {index_value}, Accel: [{accel_x}, {accel_y}, {accel_z}], Gyro: [{gyro_x}, {gyro_y}, {gyro_z}]")

async def connect_to_sensor(device, char_uuid):
    async with BleakClient(device) as client:
        if client.is_connected:
            print(f"Connected to {device.name}")
            await client.start_notify(char_uuid, notification_handler)
            while not STOP_FLAG:
                await asyncio.sleep(0.1)

async def scan_and_connect():
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name == SENSOR_NAME:
            await connect_to_sensor(device, CHAR_UUID)
            break

if __name__ == "__main__":
    with open(csv_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Index", "Accel_X", "Accel_Y", "Accel_Z", "Gyro_X", "Gyro_Y", "Gyro_Z"])
    asyncio.run(scan_and_connect())
