#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include "battery/battery.h"

#include <zephyr/settings/settings.h>



#include "kalman_filter.h"

#define MAX_RETRIES 5
#define RETRY_DELAY K_MSEC(10)
#define BATTERY_UPDATE_INTERVAL K_MSEC(1000)

#define CALIBRATION_SAMPLES 100
#define CALIBRATION_INTERVAL K_MSEC(10)

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

static KalmanFilter kf __attribute__((section(".data")));

struct sensor_data {
    uint32_t timestamp;
    uint32_t index;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint8_t battery_percentage;
} __packed;

struct calibration_data {
    bool valid;
    float accel_offset[3];
    float gyro_offset[3];
};

static struct calibration_data cal_data = {false, {0}, {0}};

static struct sensor_data s_data;
static uint32_t reference_timestamp = 0;
static uint8_t current_battery_percentage = 0;

static ssize_t read_index(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const struct sensor_data *data = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

// Add these functions for saving and loading calibration data
static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "cal", &next) && !next) {
        if (len != sizeof(cal_data)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &cal_data, sizeof(cal_data));
        if (rc < 0) {
            return rc;
        }
    }

    return 0;
}

static int save_calibration_data(void)
{
    return settings_save_one("sensor/cal", &cal_data, sizeof(cal_data));
}

static struct settings_handler conf = {
    .name = "sensor",
    .h_set = settings_set
};

static uint32_t index = 0;

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }
    printk("Connected\n");

    struct bt_le_conn_param param = {
        .interval_min = 24,
        .interval_max = 40,
        .latency = 0,
        .timeout = 400,
    };
    err = bt_conn_le_param_update(conn, &param);
    if (err) {
        printk("Failed to request longer connection interval (err %d)\n", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

// Modify the BLE write handler to accept the device pointer
static ssize_t write_reference_and_calibration(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                               const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    const struct device *sensor_dev = attr->user_data;
    
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len == 1) {
        const uint8_t *value = buf;

        if(value[0] == 0) {
            // This is a reference timestamp write
            reference_timestamp = k_uptime_get_32();
            index = 0;
            printk("Board written to. Timestamp reset to %u, index reset to 0\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", reference_timestamp);
        return len;
    } 
        else if (value[0] == 1) {
            calibrate_sensors(sensor_dev);
            printk("Calibration triggered via BLE\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
        return len;

    } else {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
}

/*Left Hand - (0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)*/

/*Right Hand - (0x8E400004, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)*/

/*Ball - (0x9E400001, 0xC5C3, 0xE393, 0xA0F9, 0xF50E24DCCA9E)*/

/*Sense Right Leg - (0x7E400001, 0xA5B3, 0xC393, 0xD0E9, 0xF50E24DCCA9E)*/

/*Sense Left Leg - (0x6E400001, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)*/

// Modify your BLE service definition to pass the device pointer
BT_GATT_SERVICE_DEFINE(index_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6E400001, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6E400002, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)),
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_index, NULL, &s_data),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6E400003, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)),
                           BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_reference_and_calibration, (void*)DEVICE_DT_GET_ONE(st_lsm6dsl))
);

void calibrate_sensors(const struct device *dev) {
    struct sensor_value accel[3], gyro[3];
    float accel_sum[3] = {0}, gyro_sum[3] = {0};

    printk("Starting sensor calibration...\n");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

        for (int j = 0; j < 3; j++) {
            accel_sum[j] += sensor_value_to_double(&accel[j]);
            gyro_sum[j] += sensor_value_to_double(&gyro[j]);
        }

        k_sleep(CALIBRATION_INTERVAL);
    }

    for (int i = 0; i < 3; i++) {
        cal_data.accel_offset[i] = accel_sum[i] / CALIBRATION_SAMPLES;
        cal_data.gyro_offset[i] = gyro_sum[i] / CALIBRATION_SAMPLES;
    }

    printk("Calibration complete.\n");
    printk("Accel offsets: [%.2f, %.2f, %.2f]\n", cal_data.accel_offset[0], cal_data.accel_offset[1], cal_data.accel_offset[2]);
    printk("Gyro offsets: [%.2f, %.2f, %.2f]\n", cal_data.gyro_offset[0], cal_data.gyro_offset[1], cal_data.gyro_offset[2]);

    cal_data.valid = true;
    
    // Save calibration data
    int err = save_calibration_data();
    if (err) {
        printk("Failed to save calibration data: %d\n", err);
    } else {
        printk("Calibration data saved successfully\n");
    }
}


// Apply calibration to sensor readings
void apply_calibration(float *accel, float *gyro) {
    for (int i = 0; i < 3; i++) {
        accel[i] -= cal_data.accel_offset[i];
        gyro[i] -= cal_data.gyro_offset[i];
    }
}

static void battery_update_handler(struct k_work *work)
{
    uint16_t battery_millivolt = 0;
    uint8_t battery_percentage = 0;

    battery_get_millivolt(&battery_millivolt);
    battery_get_percentage(&battery_percentage, battery_millivolt);

    if (battery_percentage != current_battery_percentage) {
        current_battery_percentage = battery_percentage;
        printk("Battery updated: %d%%\n", current_battery_percentage);
    }
}

K_WORK_DEFINE(battery_work, battery_update_handler);

void main(void)
{
    int err;
    index = 0;
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x6E400001, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)),
    };
    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    struct sensor_value odr_attr;

    printk("Detected device: %s\n", lsm6dsl_dev->name);

    if (!device_is_ready(lsm6dsl_dev)) {
        printk("sensor: device not ready.\n");
        return;
    }

    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for accelerometer.\n");
        return;
    }

    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for gyro.\n");
        return;
    }

    // Initialize battery monitoring
    err = battery_init();
    if (err) {
        printk("Battery init failed (err %d)\n", err);
        return;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    printk("Advertising successfully started\n");

    // Initial battery reading
    k_work_submit(&battery_work);

    err = settings_subsys_init();
    if (err) {
        printk("settings subsys initialization: fail (err %d)\n", err);
        return;
    }

    err = settings_register(&conf);
    if (err) {
        printk("settings register: fail (err %d)\n", err);
        return;
    }

    // Load saved settings
    err = settings_load();
    if (err) {
        printk("settings load: fail (err %d)\n", err);
    }

    if (cal_data.valid) {
        printk("Loaded calibration data:\n");
        printk("Accel offsets: [%.2f, %.2f, %.2f]\n", cal_data.accel_offset[0], cal_data.accel_offset[1], cal_data.accel_offset[2]);
        printk("Gyro offsets: [%.2f, %.2f, %.2f]\n", cal_data.gyro_offset[0], cal_data.gyro_offset[1], cal_data.gyro_offset[2]);
    } else {
        printk("No valid calibration data found. Please calibrate the sensors.\n");
    }

    printk("About to initialize Kalman filter\n");
    kalman_filter_init(&kf);
    printk("Kalman filter initialized\n");

    while (1) {
        if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
            printk("Sensor sample update error\n");
        } else {
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_X, &accel_x_out);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Y, &accel_y_out);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Z, &accel_z_out);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &gyro_x_out);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &gyro_y_out);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Z, &gyro_z_out);

            float accel[3] = {
                sensor_value_to_double(&accel_x_out),
                sensor_value_to_double(&accel_y_out),
                sensor_value_to_double(&accel_z_out)
            };
            float gyro[3] = {
                sensor_value_to_double(&gyro_x_out),
                sensor_value_to_double(&gyro_y_out),
                sensor_value_to_double(&gyro_z_out)
            };

            apply_calibration(accel, gyro);

            printk("Applying Kalman filter prediction...\n");
            kalman_filter_predict(&kf);
            printk("Prediction step completed.\n");

            // Print the state estimate after prediction
            printk("State estimate after prediction: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
                kf.x[0], kf.x[1], kf.x[2], kf.x[3], kf.x[4], kf.x[5]);

            // Prepare measurement for Kalman filter
            float measurement[MEASURE_DIM] = {
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2]
            };

            printk("Measurement: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
                measurement[0], measurement[1], measurement[2],
                measurement[3], measurement[4], measurement[5]);

            printk("Applying Kalman filter update...\n");

            // Lock the scheduler to protect the critical section
            k_sched_lock();

            kalman_filter_update(&kf, measurement);

            // Unlock the scheduler after the critical section
            k_sched_unlock();

            printk("Update step completed.\n");

            // Print the state estimate after update
            // printk("State estimate after update: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
            //     kf.x[0], kf.x[1], kf.x[2], kf.x[3], kf.x[4], kf.x[5]);

            s_data.timestamp = k_uptime_get_32() - reference_timestamp;
            s_data.index = index;
            s_data.accel_x = kf.x[0];
            s_data.accel_y = kf.x[1];
            s_data.accel_z = kf.x[2];
            s_data.gyro_x = kf.x[3];
            s_data.gyro_y = kf.x[4];
            s_data.gyro_z = kf.x[5];
            s_data.battery_percentage = current_battery_percentage;

            printk("Timestamp: %u, Index: %u, Accel: [%.3f, %.3f, %.3f], Gyro: [%.3f, %.3f, %.3f], Battery: %u%%\n", 
                   s_data.timestamp, s_data.index, 
                   s_data.accel_x, s_data.accel_y, s_data.accel_z, 
                   s_data.gyro_x, s_data.gyro_y, s_data.gyro_z,
                   s_data.battery_percentage);

            int retry_count = 0;
            while (bt_gatt_notify(NULL, &index_svc.attrs[1], &s_data, sizeof(s_data)) == -ENOMEM && retry_count < MAX_RETRIES) {
                printk("bt_gatt_notify failed: -ENOMEM. Retrying...\n");
                retry_count++;
                k_sleep(RETRY_DELAY);
            }

            index++;
            k_sleep(K_MSEC(50)); 
        }

        // Check if it's time to update battery percentage
        if (k_uptime_get() % BATTERY_UPDATE_INTERVAL.ticks == 0) {
            k_work_submit(&battery_work);
        }
    }
}
