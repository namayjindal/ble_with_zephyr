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

struct sensor_data {
    uint32_t timestamp;
    uint8_t index;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
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
    } else {
        printk("Connected\n");
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
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x9E400001, 0xC5C3, 0xE393, 0xB0A9, 0xE50E24DCCA9E)),
    };

    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, "XIAO-BLE", 8),
    };

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

    while (1) {
        s_data.timestamp = k_uptime_get();
        s_data.index++;
        s_data.accel_x = rand() % 2000 - 1000;  // Simulated accelerometer X value
        s_data.accel_y = rand() % 2000 - 1000;  // Simulated accelerometer Y value
        s_data.accel_z = rand() % 2000 - 1000;  // Simulated accelerometer Z value
        s_data.gyro_x = rand() % 2000 - 1000;   // Simulated gyroscope X value
        s_data.gyro_y = rand() % 2000 - 1000;   // Simulated gyroscope Y value
        s_data.gyro_z = rand() % 2000 - 1000;   // Simulated gyroscope Z value

        printk("Timestamp: %u, Index: %d, Accel: [%d, %d, %d], Gyro: [%d, %d, %d]\n", 
               s_data.timestamp, s_data.index, 
               s_data.accel_x, s_data.accel_y, s_data.accel_z, 
               s_data.gyro_x, s_data.gyro_y, s_data.gyro_z);
        
        bt_gatt_notify(NULL, &index_svc.attrs[1], &s_data, sizeof(s_data));
        k_sleep(K_MSEC(20));  // Adjust the sleep duration as needed
    }
}
