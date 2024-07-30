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
#include <stdio.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/settings/settings.h>

#include "kalman_filter.h"

#define MAX_RETRIES 5
#define RETRY_DELAY K_MSEC(10)
#define BATTERY_UPDATE_INTERVAL K_MSEC(50)

#define CALIBRATION_SAMPLES 100
#define CALIBRATION_INTERVAL K_MSEC(10)

#define BUFFER_SIZE 5

#define SLEEP_TIME_MS   1000

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

#define BATTERY_LOW_THRESHOLD 20
#define BATTERY_FULL_THRESHOLD 95
#define BATTERY_CHARGING_THRESHOLD 4500  // millivolts, adjust as needed

static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

// Add these definitions for LED control
#define LED_UPDATE_INTERVAL K_MSEC(50)
#define BATTERY_LOW_THRESHOLD 20
#define BATTERY_FULL_THRESHOLD 95

static uint8_t current_battery_percentage = 0;
static uint16_t current_battery_millivolt = 0;

static void update_leds(uint8_t battery_percentage, uint16_t battery_mv)
{
    static int64_t last_toggle = 0;
    static bool led_state = true;
    static enum {
        LED_OFF,
        LED_RED_BLINK,
        LED_GREEN_SOLID,
        LED_GREEN_BLINK,
        LED_BLUE_SOLID,
        LED_RED_SOLID
    } led_mode = LED_OFF;

    // Determine the LED mode based on battery status
    if (battery_mv >= BATTERY_CHARGING_THRESHOLD) {
        led_mode = LED_GREEN_BLINK;  // Charging
    } else if (battery_percentage >= BATTERY_FULL_THRESHOLD) {
        led_mode = LED_GREEN_SOLID;  // Full
    } else if (battery_percentage < BATTERY_LOW_THRESHOLD) {
        led_mode = LED_RED_BLINK;  // Low
    } else {
        led_mode = LED_OFF;  // Normal
    }

    // Update LEDs based on the mode
    switch (led_mode) {
        case LED_OFF:
            gpio_pin_set_dt(&led_red, 0);
            gpio_pin_set_dt(&led_green, 0);
            gpio_pin_set_dt(&led_blue, 0);
            printk("LED mode: LED_OFF\n");
            break;
        case LED_RED_BLINK:
            if (k_uptime_get() - last_toggle >= LED_UPDATE_INTERVAL.ticks) {
                led_state = !led_state;
                gpio_pin_set_dt(&led_red, led_state);
                gpio_pin_set_dt(&led_green, 0);
                gpio_pin_set_dt(&led_blue, 0);
                last_toggle = k_uptime_get();
            }
            printk("LED mode: LED_RED_BLINK\n");
            break;
        case LED_GREEN_SOLID:
            gpio_pin_set_dt(&led_red, 0);
            gpio_pin_set_dt(&led_green, 1);
            gpio_pin_set_dt(&led_blue, 0);
            printk("LED mode: LED_GREEN_SOLID\n");
            break;
        case LED_BLUE_SOLID:
            gpio_pin_set_dt(&led_red, 0);
            gpio_pin_set_dt(&led_green, 0);
            gpio_pin_set_dt(&led_blue, 1);
            printk("LED mode: LED_BLUE_SOLID\n");
            break;
        case LED_RED_SOLID:
            gpio_pin_set_dt(&led_red, 1);
            gpio_pin_set_dt(&led_green, 0);
            gpio_pin_set_dt(&led_blue, 0);
            printk("LED mode: LED_RED_SOLID\n");
            break;
        case LED_GREEN_BLINK:
            if (k_uptime_get() - last_toggle >= LED_UPDATE_INTERVAL.ticks) {
                led_state = !led_state;
                gpio_pin_set_dt(&led_red, 0);
                gpio_pin_set_dt(&led_green, led_state);
                gpio_pin_set_dt(&led_blue, 0);
                last_toggle = k_uptime_get();
            }
            printk("LED mode: LED_GREEN_BLINK\n");
            break;
    }
}

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

struct sensor_data_buffer {
    struct sensor_data data[BUFFER_SIZE];
    uint8_t count;
};

static struct sensor_data_buffer s_data_buffer = {0};

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

static KalmanFilter kf __attribute__((section(".data")));

struct calibration_data {
    bool valid;
    float accel_offset[3];
    float gyro_offset[3];
};

static struct calibration_data cal_data = {false, {0}, {0}};

static struct sensor_data s_data;
static uint32_t reference_timestamp = 0;

static ssize_t read_index(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const struct sensor_data_buffer *data = attr->user_data;
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

/*Right Hand - (0x8E400004, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)*/

/*Left Hand - (0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)*/

/*Sense Right Leg - (0x7E400001, 0xA5B3, 0xC393, 0xD0E9, 0xF50E24DCCA9E)*/

/*Sense Left Leg - (0x6E400001, 0xB5C3, 0xD393, 0xA0F9, 0xE50F24DCCA9E)*/

/*Ball - (0x9E400001, 0xC5C3, 0xE393, 0xA0F9, 0xF50E24DCCA9E)*/

// Modify your BLE service definition to pass the device pointer
BT_GATT_SERVICE_DEFINE(index_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8E400004, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8E400005, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)),
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_index, NULL, &s_data_buffer),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8E400006, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)),
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

    if (battery_percentage != current_battery_percentage || 
        abs((int)battery_millivolt - (int)current_battery_millivolt) > 50) {
        current_battery_percentage = battery_percentage;
        current_battery_millivolt = battery_millivolt;
        printk("Battery updated: %d%% (%d mV)\n", current_battery_percentage, current_battery_millivolt);
        update_leds(current_battery_percentage, current_battery_millivolt);
    }
}

K_WORK_DEFINE(battery_work, battery_update_handler);

void main(void)
{
    int err;
    index = 0;
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x8E400004, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)),
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

    int ret_red;
    int ret_green;
	bool red_led_state = true;
    bool green_led_state = true;

	if (!gpio_is_ready_dt(&led_red)) {
		return 0;
	}

    if (!gpio_is_ready_dt(&led_green)) {
		return 0;
	}

	ret_red = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
	if (ret_red < 0) {
		return 0;
	}

    ret_green = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
	if (ret_green < 0) {
		return 0;
	}

    // Initialize LEDs
    if (!device_is_ready(led_red.port) || !device_is_ready(led_green.port)) {
        printk("Error: LEDs device not ready\n");
        return;
    }

    int ret = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Error %d: failed to configure red LED\n", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Error %d: failed to configure green LED\n", ret);
        return;
    }

    while (1) {

        update_leds(current_battery_percentage, current_battery_millivolt);
		// printf("LED state: %s\n", red_led_state ? "ON" : "OFF");
		// k_msleep(SLEEP_TIME_MS);

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

            printk("Battery millivolt - %u\n", current_battery_millivolt);

            struct sensor_data current_data = {
                .timestamp = k_uptime_get_32() - reference_timestamp,
                .index = index,
                .accel_x = kf.x[0],
                .accel_y = kf.x[1],
                .accel_z = kf.x[2],
                .gyro_x = kf.x[3],
                .gyro_y = kf.x[4],
                .gyro_z = kf.x[5],
                .battery_percentage = current_battery_percentage
            };

            printk("Timestamp: %u, Index: %u, Accel: [%.3f, %.3f, %.3f], Gyro: [%.3f, %.3f, %.3f], Battery: %u%%\n", 
                   s_data.timestamp, s_data.index, 
                   s_data.accel_x, s_data.accel_y, s_data.accel_z, 
                   s_data.gyro_x, s_data.gyro_y, s_data.gyro_z,
                   s_data.battery_percentage);

            s_data_buffer.data[s_data_buffer.count] = current_data;
            s_data_buffer.count++;

            // If the buffer is full, send it
            if (s_data_buffer.count == BUFFER_SIZE) {
                int retry_count = 0;
                
                // Prepare the buffer to send
                uint8_t buffer_to_send[sizeof(uint8_t) + sizeof(s_data_buffer)];
                buffer_to_send[0] = s_data_buffer.count;
                memcpy(buffer_to_send + 1, &s_data_buffer, sizeof(s_data_buffer));

                while (bt_gatt_notify(NULL, &index_svc.attrs[1], buffer_to_send, sizeof(buffer_to_send)) == -ENOMEM 
                       && retry_count < MAX_RETRIES) {
                    printk("bt_gatt_notify failed: -ENOMEM. Retrying...\n");
                    retry_count++;
                    k_sleep(RETRY_DELAY);
                }

                // Reset the buffer after sending
                s_data_buffer.count = 0;
            }

            index++;
        }

        // Check if it's time to update battery percentage
        if (k_uptime_get() % BATTERY_UPDATE_INTERVAL.ticks == 0) {
            k_work_submit(&battery_work);
        }

        k_sleep(K_MSEC(50)); 
    }
}
