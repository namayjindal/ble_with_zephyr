import asyncio
import csv
import struct
from datetime import datetime
from bleak import BleakClient, BleakScanner
import logging

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# UUIDs and other data
SENSOR_NAME = "Sense Ball"
SERVICE_UUID = "9E400001-C5C3-E393-B0A9-E50E24DCCA9E"
CHAR_UUID = "9E400003-C5C3-E393-B0A9-E50E24DCCA9E"

csv_filename = "sensor_data_with_timestamp_with_sensor_with_real_imu.csv"
STOP_FLAG = False

async def notification_handler(sender, data):
    logger.debug(f"Received data: {data.hex()} from {sender}")
    try:
        timestamp, index_value, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = struct.unpack('<IIffffff', data)
        timestamp_sec = timestamp / 1000
        with open(csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp_sec, index_value] + [round(val, 4) for val in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]])
        logger.info(f"Timestamp: {timestamp_sec:.3f}, Index: {index_value}, Accel: [{accel_x:.4f}, {accel_y:.4f}, {accel_z:.4f}], Gyro: [{gyro_x:.4f}, {gyro_y:.4f}, {gyro_z:.4f}]")
    except Exception as e:
        logger.error(f"Error in notification handler: {e}")

async def connect_to_sensor(device, char_uuid):
    try:
        async with BleakClient(device) as client:
            logger.info(f"Connected: {client.is_connected}")
            if client.is_connected:
                logger.info(f"Connected to {device.name}")
                
                services = await client.get_services()
                for service in services:
                    logger.debug(f"Service: {service.uuid}")
                    for char in service.characteristics:
                        logger.debug(f"  Characteristic: {char.uuid}")
                        if char.uuid == char_uuid:
                            logger.info(f"Found matching characteristic: {char.uuid}")
                
                await client.start_notify(char_uuid, notification_handler)
                logger.info("Notifications enabled")
                
                while not STOP_FLAG:
                    await asyncio.sleep(1)
                    logger.debug("Waiting...")

    except Exception as e:
        logger.error(f"Error in connect_to_sensor: {e}")

async def scan_and_connect():
    try:
        devices = await BleakScanner.discover()
        for device in devices:
            logger.debug(f"Found device: {device.name} ({device.address})")
            if device.name == SENSOR_NAME:
                logger.info(f"Found {SENSOR_NAME} at {device.address}")
                await connect_to_sensor(device, CHAR_UUID)
                break
        else:
            logger.warning(f"{SENSOR_NAME} not found")
    except Exception as e:
        logger.error(f"Error in scan_and_connect: {e}")

if __name__ == "__main__":
    logger.info('Script started')
    with open(csv_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Index", "Accel_X", "Accel_Y", "Accel_Z", "Gyro_X", "Gyro_Y", "Gyro_Z"])
    logger.info('CSV file created')
    asyncio.run(scan_and_connect())
    logger.info('Script finished')