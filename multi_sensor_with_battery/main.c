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

#define MAX_RETRIES 5
#define RETRY_DELAY K_MSEC(10)
#define BATTERY_UPDATE_INTERVAL K_MSEC(1000)

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

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

static struct sensor_data s_data;
static uint32_t reference_timestamp = 0;
static uint8_t current_battery_percentage = 0;

static ssize_t read_index(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const struct sensor_data *data = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static uint32_t index = 0;

static ssize_t write_reference(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len != sizeof(uint32_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    reference_timestamp = k_uptime_get_32();
    index = 0;
    
    printk("Board written to. Timestamp reset to %u, index reset to 0\n", reference_timestamp);
    return len;
}

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

/*Left Hand - 6E400001-B5A3-F393-E0A9-E50E24DCCA9E*/

/*Right Hand - 8E400004-B5A3-F393-E0A9-E50E24DCCA9E*/

/*Ball - 9E400001-C5C3-E393-B0A9-E50E24DCCA9E*/

/*Sense Right Leg - (0x7E400001, 0xA5B3, 0xC393, 0xD0E9, 0xF50E24DCCA9E)*/

/*Sense Left Leg - (0x6E400001, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)*/

BT_GATT_SERVICE_DEFINE(index_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6E400001, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6E400002, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)),
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_index, NULL, &s_data),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6E400003, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)),
                           BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_reference, NULL)
);

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

            s_data.timestamp = k_uptime_get_32() - reference_timestamp;
            s_data.index = index;
            s_data.accel_x = sensor_value_to_double(&accel_x_out);
            s_data.accel_y = sensor_value_to_double(&accel_y_out);
            s_data.accel_z = sensor_value_to_double(&accel_z_out);
            s_data.gyro_x = sensor_value_to_double(&gyro_x_out);
            s_data.gyro_y = sensor_value_to_double(&gyro_y_out);
            s_data.gyro_z = sensor_value_to_double(&gyro_z_out);
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
