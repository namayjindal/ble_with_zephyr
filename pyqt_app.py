import sys
import asyncio
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox, QCalendarWidget, QMessageBox, QStackedWidget
from PyQt5.QtCore import QTimer, QDateTime, Qt, pyqtSignal, QObject
from bleak import BleakClient, BleakScanner
import csv
import struct
from datetime import datetime
import logging
import os
from collections import defaultdict
import qasync

# Add these imports at the top of your file
import objc
from Foundation import NSObject

# Add this class definition
class AppDelegate(NSObject):
    def applicationSupportsSecureRestorableState_(self, app):
        return True

# Existing sensor and exercise configurations
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
        "name": "Sense Right Leg",
        "service_uuid": "7E400001-A5B3-C393-D0E9-F50E24DCCA9E",
        "char_uuid": "7E400002-A5B3-C393-D0E9-F50E24DCCA9E",
        "reference_uuid": "7E400003-A5B3-C393-D0E9-F50E24DCCA9E",
        "prefix": "right_leg",
    },
    4: {
        "name": "Sense Left Leg",
        "service_uuid": "6E400001-B5C3-D393-A0F9-E50F24DCCA9E",
        "char_uuid": "6E400002-B5C3-D393-A0F9-E50F24DCCA9E",
        "reference_uuid": "6E400003-B5C3-D393-A0F9-E50F24DCCA9E",
        "prefix": "left_leg"
    },
    5: {
        "name": "Sense Ball",
        "service_uuid": "9E400001-C5C3-E393-B0A9-E50E24DCCA9E",
        "char_uuid": "9E400002-C5C3-E393-B0A9-E50E24DCCA9E",
        "reference_uuid": "9E400003-C5C3-E393-B0A9-E50E24DCCA9E",
        "prefix": "ball"
    }
}

EXERCISE_CONFIGS = {
    "single_hand": [3],  # Only right hand sensor
    "both_hands": [3, 4],  # Right and left hand sensors
    "hands_and_ball": [1, 2, 3],  # All sensors
    "all_sensors": [1, 2, 3, 4, 5]  # All sensors
}

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class SensorData:
    def __init__(self, sensor_ids):
        self.data = defaultdict(list)
        self.sensor_ids = sensor_ids

    def add_data(self, sensor_index, timestamp, values):
        self.data[sensor_index].append((timestamp, values))
        logger.debug(f"Added data for sensor {sensor_index}, timestamp {timestamp}")

    def get_synced_data(self):
        synced_data = []
        min_timestamp = min(min(self.data[sensor_index])[0] for sensor_index in self.sensor_ids)
        for sensor_index in self.sensor_ids:
            synced_readings = [entry for entry in self.data[sensor_index] if entry[0] >= min_timestamp]
            if synced_readings:
                synced_data.append(synced_readings[0])
        if len(synced_data) == len(self.sensor_ids):
            return synced_data
        return None

    def pop_synced_data(self):
        synced_data = self.get_synced_data()
        if synced_data:
            for sensor_index in self.sensor_ids:
                self.data[sensor_index] = self.data[sensor_index][1:]
            return synced_data
        return None
    
class BLEWorker(QObject):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal(str)
    data_received = pyqtSignal(int, float, list)

    def __init__(self, sensor_configs):
        super().__init__()
        self.sensor_configs = sensor_configs
        self.clients = {}
        self.is_running = False

    async def connect_sensors(self, sensor_ids):
        devices = await BleakScanner.discover()
        for sensor_id in sensor_ids:
            config = self.sensor_configs[sensor_id]
            device = next((d for d in devices if d.name == config['name']), None)
            if device:
                client = BleakClient(device.address)
                await client.connect()
                self.clients[sensor_id] = client
                await self.write_reference_timestamp(client, config['reference_uuid'])
                self.connected.emit(config['name'])
            else:
                self.disconnected.emit(config['name'])

    async def write_reference_timestamp(self, client, reference_uuid):
        reference_timestamp = int(datetime.now().timestamp())  # milliseconds
        reference_data = struct.pack('<I', reference_timestamp)
        await client.write_gatt_char(reference_uuid, reference_data)
        logging.info(f"Written reference timestamp: {reference_timestamp}")

    async def start_notifications(self):
        self.is_running = True
        for sensor_id, client in self.clients.items():
            config = self.sensor_configs[sensor_id]
            await client.start_notify(config['char_uuid'], 
                lambda s, d: self.notification_handler(sensor_id, s, d))

    def notification_handler(self, sensor_id, sender, data):
        timestamp, index_value, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, battery_percentage = struct.unpack('<IIffffffB', data)
        timestamp_sec = timestamp / 1000
        values = [index_value] + [round(value, 4) for value in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]] + [battery_percentage]
        self.data_received.emit(sensor_id, timestamp_sec, values)

    async def stop_notifications(self):
        self.is_running = False
        for client in self.clients.values():
            await client.disconnect()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.stacked_widget = QStackedWidget()
        
        # Create pages
        self.school_info_page = SchoolInfoPage(self.stacked_widget)
        self.student_info_page = StudentInfoPage(self.stacked_widget)
        self.exercise_page = ExercisePage(self.stacked_widget)
        
        # Add pages to stacked widget
        self.stacked_widget.addWidget(self.school_info_page)
        self.stacked_widget.addWidget(self.student_info_page)
        self.stacked_widget.addWidget(self.exercise_page)
        
        layout = QVBoxLayout()
        layout.addWidget(self.stacked_widget)
        self.setLayout(layout)
        
        self.setWindowTitle('BLE Sensor Data Collection')
        self.setGeometry(300, 300, 400, 300)

class SchoolInfoPage(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        
        self.school_name_input = QLineEdit()
        self.date_input = QCalendarWidget()
        
        layout.addWidget(QLabel("School Name:"))
        layout.addWidget(self.school_name_input)
        layout.addWidget(QLabel("Date:"))
        layout.addWidget(self.date_input)
        
        next_button = QPushButton("Next")
        next_button.clicked.connect(self.next_page)
        layout.addWidget(next_button)
        
        self.setLayout(layout)

    def next_page(self):
        self.stacked_widget.setCurrentIndex(1)

class StudentInfoPage(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        
        self.student_name_input = QLineEdit()
        self.grade_input = QComboBox()
        self.grade_input.addItems(["1st", "2nd", "3rd", "4th", "5th", "6th", "7th", "8th", "9th", "10th", "11th", "12th"])
        self.height_input = QLineEdit()
        self.weight_input = QLineEdit()
        self.gender_input = QComboBox()
        self.gender_input.addItems(["Boy", "Girl"])
        self.exercise_input = QComboBox()
        self.exercise_input.addItems(list(EXERCISE_CONFIGS.keys()))
        
        layout.addWidget(QLabel("Student Name:"))
        layout.addWidget(self.student_name_input)
        layout.addWidget(QLabel("Grade:"))
        layout.addWidget(self.grade_input)
        layout.addWidget(QLabel("Height (optional):"))
        layout.addWidget(self.height_input)
        layout.addWidget(QLabel("Weight (optional):"))
        layout.addWidget(self.weight_input)
        layout.addWidget(QLabel("Gender:"))
        layout.addWidget(self.gender_input)
        layout.addWidget(QLabel("Exercise:"))
        layout.addWidget(self.exercise_input)
        
        next_button = QPushButton("Next")
        next_button.clicked.connect(self.next_page)
        layout.addWidget(next_button)
        
        self.setLayout(layout)

    def next_page(self):
        self.stacked_widget.setCurrentIndex(2)

class ExercisePage(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.initUI()
        
        self.sensor_data = defaultdict(list)
        self.csv_filename = None
        self.connected_sensors = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_timer)
        self.exercise_time = 0
        self.is_paused = False

        self.ble_worker = BLEWorker(SENSOR_CONFIGS)
        self.ble_worker.connected.connect(self.on_sensor_connected)
        self.ble_worker.disconnected.connect(self.on_sensor_disconnected)
        self.ble_worker.data_received.connect(self.on_data_received)

    def initUI(self):
        layout = QVBoxLayout()
        
        self.connect_button = QPushButton("Connect to Sensors")
        self.connect_button.clicked.connect(self.connect_sensors)
        layout.addWidget(self.connect_button)
        
        self.status_label = QLabel("Not connected")
        layout.addWidget(self.status_label)
        
        self.start_button = QPushButton("Start Exercise")
        self.start_button.clicked.connect(self.start_exercise)
        self.start_button.setEnabled(False)
        layout.addWidget(self.start_button)
        
        self.timer_label = QLabel("00:00:00")
        layout.addWidget(self.timer_label)
        
        button_layout = QHBoxLayout()
        self.pause_button = QPushButton("Pause Exercise")
        self.pause_button.clicked.connect(self.pause_exercise)
        self.pause_button.setEnabled(False)
        button_layout.addWidget(self.pause_button)
        
        self.stop_button = QPushButton("Stop Exercise")
        self.stop_button.clicked.connect(self.stop_exercise)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)

    def connect_sensors(self):
        exercise = self.stacked_widget.widget(1).exercise_input.currentText()
        self.sensor_ids = EXERCISE_CONFIGS[exercise]
        asyncio.ensure_future(self.ble_worker.connect_sensors(self.sensor_ids))

    def on_sensor_connected(self, sensor_name):
        self.status_label.setText(f"Connected to {sensor_name}")
        self.connected_sensors.append(sensor_name)
        if len(self.connected_sensors) == len(self.sensor_ids):
            self.start_button.setEnabled(True)

    def on_sensor_disconnected(self, sensor_name):
        self.status_label.setText(f"Failed to connect to {sensor_name}")

    def start_exercise(self):
        self.csv_filename = self.generate_csv_filename()
        self.create_csv_file()
        asyncio.ensure_future(self.ble_worker.start_notifications())
        self.timer.start(1000)
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.stop_button.setEnabled(True)

    def pause_exercise(self):
        if not self.is_paused:
            self.timer.stop()
            self.pause_button.setText("Resume Exercise")
            self.is_paused = True
        else:
            self.timer.start(1000)
            self.pause_button.setText("Pause Exercise")
            self.is_paused = False

    def stop_exercise(self):
        self.timer.stop()
        asyncio.ensure_future(self.ble_worker.stop_notifications())
        self.save_data()
        self.ask_keep_data()

    def on_data_received(self, sensor_id, timestamp, values):
        if not self.is_paused:
            self.sensor_data[sensor_id].append([timestamp] + values)
            self.write_to_csv(sensor_id, timestamp, values)

    def update_timer(self):
        self.exercise_time += 1
        hours, remainder = divmod(self.exercise_time, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.timer_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

    def generate_csv_filename(self):
        school_info = self.stacked_widget.widget(0)
        student_info = self.stacked_widget.widget(1)
        school_name = school_info.school_name_input.text()
        date = school_info.date_input.selectedDate().toString(Qt.ISODate)
        student_name = student_info.student_name_input.text()
        exercise = student_info.exercise_input.currentText()
        return f"{school_name}_{date}_{student_name}_{exercise}.csv"

    def create_csv_file(self):
        headers = ['timestamp']
        for sensor_id in self.sensor_ids:
            config = SENSOR_CONFIGS[sensor_id]
            headers.extend([f"{config['prefix']}_{col}" for col in ['index', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'battery']])
        
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)

    def write_to_csv(self, sensor_id, timestamp, values):
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            row = [timestamp]
            for id in self.sensor_ids:
                if id == sensor_id:
                    row.extend(values)
                else:
                    row.extend([''] * 8)  # 8 empty values for unused sensors
            writer.writerow(row)

    def save_data(self):
        # This method is called when stopping the exercise
        # The data is already saved in real-time, so we don't need to do anything here
        pass

    def ask_keep_data(self):
        reply = QMessageBox.question(self, 'Exercise Completed', 
                                     'Do you want to keep the collected data?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        
        if reply == QMessageBox.Yes:
            self.label_data()
        else:
            os.remove(self.csv_filename)
            self.reset_exercise_page()

    def label_data(self):
        # Here you can implement a dialog to label the data if needed
        # For now, we'll just show a message
        QMessageBox.information(self, 'Data Saved', 
                                f'Data has been saved to {self.csv_filename}')
        self.reset_exercise_page()

    def reset_exercise_page(self):
        self.exercise_time = 0
        self.timer_label.setText("00:00:00")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.connect_button.setEnabled(True)
        self.status_label.setText("Not connected")
        self.sensor_data.clear()
        self.connected_sensors.clear()

async def main():
    def close_future(future, loop):
        loop.call_later(10, future.cancel)
        future.cancel()

    loop = asyncio.get_event_loop()
    future = asyncio.Future()

    app = QApplication.instance()
    if hasattr(app, 'aboutToQuit'):
        getattr(app, 'aboutToQuit').connect(
            lambda *args: close_future(future, loop)
        )
    
    main_window = MainWindow()
    main_window.show()

    await future
    return True

if __name__ == '__main__':
    try:
        qasync.run(main())
    except asyncio.exceptions.CancelledError:
        sys.exit(0)
