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
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#define MAX_RETRIES 5
#define RETRY_DELAY K_MSEC(10)

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

struct sensor_data {
    uint32_t timestamp;
    uint32_t index;  // Change from uint8_t to uint32_t
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} __packed;

static struct sensor_data s_data;

static ssize_t read_index(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const struct sensor_data *data = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }
    printk("Connected\n");
    
    // Request a longer connection interval
    struct bt_le_conn_param param = {
        .interval_min = 24,  // 30 ms
        .interval_max = 40,  // 50 ms
        .latency = 0,
        .timeout = 400,      // 4 seconds
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

BT_GATT_SERVICE_DEFINE(index_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x9E400001, 0xC5C3, 0xE393, 0xB0A9, 0xE50E24DCCA9E))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x9E400003, 0xC5C3, 0xE393, 0xB0A9, 0xE50E24DCCA9E)),
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_index, NULL, &s_data),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

void main(void)
{
    int err;
    uint32_t index = 0;
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x9E400001, 0xC5C3, 0xE393, 0xB0A9, 0xE50E24DCCA9E)),
    };
    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    struct sensor_value odr_attr;

    printk("Detected device: %s\n", lsm6dsl_dev->name);

    if (!device_is_ready(lsm6dsl_dev)) {
        printk("sensor: device not ready.");
        return;
    }

    // Set accel/gyro sampling frequency to 104 Hz
    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for accelerometer.\n");
        return;
    }

    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for gyro.\n");
        return;
    }

    // Initialize the Bluetooth subsystem
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    // Start advertising
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    printk("Advertising successfully started\n");

    // In the main loop
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

            s_data.timestamp = k_uptime_get_32();
            s_data.index = index;
            s_data.accel_x = sensor_value_to_double(&accel_x_out);
            s_data.accel_y = sensor_value_to_double(&accel_y_out);
            s_data.accel_z = sensor_value_to_double(&accel_z_out);
            s_data.gyro_x = sensor_value_to_double(&gyro_x_out);
            s_data.gyro_y = sensor_value_to_double(&gyro_y_out);
            s_data.gyro_z = sensor_value_to_double(&gyro_z_out);

            printk("Timestamp: %u, Index: %u, Accel: [%.3f, %.3f, %.3f], Gyro: [%.3f, %.3f, %.3f]\n", 
                   s_data.timestamp, s_data.index, 
                   s_data.accel_x, s_data.accel_y, s_data.accel_z, 
                   s_data.gyro_x, s_data.gyro_y, s_data.gyro_z);

            int retry_count = 0;
            while (bt_gatt_notify(NULL, &index_svc.attrs[1], &s_data, sizeof(s_data)) == -ENOMEM && retry_count < MAX_RETRIES) {
                printk("bt_gatt_notify failed: -ENOMEM. Retrying...\n");
                retry_count++;
                k_sleep(RETRY_DELAY);
            }

            index++;
            k_sleep(K_MSEC(50));  // Adjust the sleep duration as needed
        }
    }
}
