/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <esb.h>
#include <zephyr/sys_clock.h>
#include <string.h>
#include <nrfx.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);

static K_SEM_DEFINE(usb_configured_sem, 0, 1);
static volatile bool configured = false; 

/* --- Function Forward Declarations --- */
static void transmitter_thread(void *arg1, void *arg2, void *arg3);
void sample_and_transmit(struct k_timer *timer);
void enter_low_power(struct k_timer *dummy);
void update_sleep_timeout(void);
void led_timer_handler(struct k_timer *timer);
void check_battery_status(void);
void battery_warning_timer_handler(struct k_timer *timer);
void battery_pulse_timer_handler(struct k_timer *timer);
static void charging_pulse_timer_handler(struct k_timer *timer);
static void send_error_log(const char *msg);

/* --- Pin Definitions from Device Tree --- */
static const struct gpio_dt_spec data_plus = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(data_plus), gpios, {0});
static const struct gpio_dt_spec data_minus = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(data_minus), gpios, {0});
static const struct pwm_dt_spec red_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led0));
static const struct pwm_dt_spec green_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led0));
static const struct pwm_dt_spec blue_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led0));

#if DT_NODE_HAS_STATUS(ADC_NODE, okay)
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET();
#else
static const struct adc_dt_spec adc_channel = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc)),
    .channel_id = 0,  // P0.04 for battery ADC
};
#endif

/* --- LED and Timer Definitions --- */
enum led_color { LED_RED, LED_GREEN, LED_BLUE, LED_OFF, LED_YELLOW, LED_ORANGE };
static K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, NULL, enter_low_power);
static K_TIMER_DEFINE(sampling_timer, sample_and_transmit, NULL);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);

/* --- ESB and Power Management --- */
static struct esb_payload tx_payload;
static uint8_t last_payload_data[2] = {0xFF, 0xFF};
#define ERROR_PAYLOAD_FLAG 0x80  // High bit indicates error message

/* --- Hysteresis Sleep Configuration --- */
#define BASE_TIMEOUT K_SECONDS(60)  // 1 minute default
#define GRACE_PERIOD_MS 2000  // 2 seconds in milliseconds
static k_timeout_t current_timeout = BASE_TIMEOUT;
static int wakeup_count = 0;
static int64_t last_sleep_time = 0;

/* --- Battery Configuration --- */
#define BATTERY_CRITICAL_THRESHOLD 15  // 15% critical level
#define BATTERY_MAX_VOLTAGE 4200  // mV (fully charged, e.g., 4.2V)
#define BATTERY_MIN_VOLTAGE 3300  // mV (fully discharged, e.g., 3.3V)
#define CHARGING_THRESHOLD_DELTA 50  // 50mV increase indicates charging
#define FULL_CHARGE_THRESHOLD 4190  // 10mV below max to account for noise
static int battery_level = 100;  // Placeholder, updated by ADC
static bool battery_warning_active = false;
static int pulse_count = 0;
static int64_t last_voltage_time = 0;
static uint32_t last_voltage_mv = 0;
static uint32_t max_voltage_mv = 0;
static bool is_charging = false;

/* --- Thread Stack --- */
K_THREAD_STACK_DEFINE(transmitter_stack, 1024);
static struct k_thread transmitter_thread_data;

/* --- Function Definitions --- */

static int set_pwm_dt(const struct pwm_dt_spec *pwm, uint8_t channel, uint32_t period, uint32_t pulse)
{
    int ret = pwm_set_dt(pwm, period, pulse);
    if (ret) {
        char msg[32];
        snprintf(msg, sizeof(msg), "PWM set failed: %d", ret);
        send_error_log(msg);
    }
    return ret;
}

void set_led_status(enum led_color color)
{
    uint32_t period = PWM_SEC(1); // 1 second base period
    uint32_t red_pulse = 0, green_pulse = 0, blue_pulse = 0;

    switch (color) {
        case LED_RED:
            red_pulse = period / 2; // 50% duty cycle
            break;
        case LED_GREEN:
            green_pulse = period / 2; // 50% duty cycle
            break;
        case LED_BLUE:
            blue_pulse = period / 2; // 50% duty cycle
            break;
        case LED_YELLOW:
            red_pulse = period / 2; green_pulse = period / 2;
            break;
        case LED_ORANGE:
            red_pulse = period / 2; green_pulse = period / 4; // 25% green for orange
            break;
        case LED_OFF:
            red_pulse = 0; green_pulse = 0; blue_pulse = 0;
            break;
    }

    if (pwm_is_ready_dt(&red_pwm)) {
        set_pwm_dt(&red_pwm, 2, period, red_pulse);   // Channel 2 for red (P0.22)
        set_pwm_dt(&red_pwm, 3, period, green_pulse); // Channel 3 for green (P0.11)
        set_pwm_dt(&red_pwm, 0, period, blue_pulse);  // Channel 0 for blue (P0.24)
    }
}

void led_timer_handler(struct k_timer *timer)
{
    if (!is_charging) {  // Only turn off if not charging
        set_led_status(LED_OFF);
    }
}

void battery_warning_timer_handler(struct k_timer *timer)
{
    if (!is_charging && battery_warning_active) {
        k_timer_start(&battery_pulse_timer, K_NO_WAIT, K_NO_WAIT);  // Start pulsing sequence
    }
}

void battery_pulse_timer_handler(struct k_timer *timer)
{
    if (pulse_count < 8 && !is_charging) {  // 4 pulses, 2 states each (on/off)
        set_led_status(LED_RED);
        k_sleep(K_MSEC(500));
        set_led_status(LED_OFF);
        k_sleep(K_MSEC(500));
        pulse_count++;
    } else {
        pulse_count = 0;
        k_timer_stop(&battery_pulse_timer);
        if (battery_warning_active && !is_charging) {
            k_timer_start(&battery_warning_timer, K_MINUTES(5), K_NO_WAIT);  // Repeat after 5 minutes
        }
    }
}

static void charging_pulse_timer_handler(struct k_timer *timer)
{
    static bool led_on = true;
    if (is_charging) {
        set_led_status(led_on ? LED_GREEN : LED_OFF);
        led_on = !led_on;
        k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT); // Pulse every 500ms
    } else {
        k_timer_stop(&charging_pulse_timer);
        set_led_status(LED_OFF);
    }
}

void enter_low_power(struct k_timer *dummy)
{
    LOG_INF("Inactivity detected. Preparing to enter System OFF mode.");
    int retry_count = 0;
    const int max_retries = 3;
    while (retry_count < max_retries && !esb_is_idle()) {
        LOG_INF("ESB is busy, waiting %d/%d...", retry_count + 1, max_retries);
        k_sleep(K_MSEC(20));  // 20ms delay per retry
        retry_count++;
    }
    if (!esb_is_idle()) {
        char msg[32];
        snprintf(msg, sizeof(msg), "ESB busy after %d retries", max_retries);
        send_error_log(msg);
    }
    esb_disable();  // Disable ESB once idle or after retries
    set_led_status(LED_BLUE);  // Indicate power-saving mode
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);  // Blue LED for 4 seconds
    
    // Wakeup handled by GPIO interrupts
    esb_start_tx();  // Re-enable ESB on wakeup
    LOG_INF("Waking up, resetting inactivity timer.");
    k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
}

void update_sleep_timeout(void)
{
    int64_t now_ms = k_uptime_get();  // Current time in milliseconds
    k_ticks_t now_ticks = k_ms_to_ticks_ceil32(now_ms);  // Convert to ticks
    k_ticks_t last_ticks = k_ms_to_ticks_ceil32(last_sleep_time);  // Convert last time to ticks
    k_ticks_t grace_ticks = k_ms_to_ticks_ceil32(GRACE_PERIOD_MS);  // Convert grace period to ticks
    if (last_sleep_time != 0 && (now_ticks - last_ticks) < grace_ticks) {
        wakeup_count++;
        if (wakeup_count == 1) {
            current_timeout = K_SECONDS(120); // 2 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 2 minutes.");
        } else if (wakeup_count >= 2) {
            current_timeout = K_SECONDS(180); // 3 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 3 minutes.");
            wakeup_count = 2;
        } else if (wakeup_count >= 3) {
            current_timeout = K_SECONDS(240); // 4 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 4 minutes.");
            wakeup_count = 3;
        } else if (wakeup_count >= 4) {
            current_timeout = K_SECONDS(300); // 5 minutes
            LOG_INF("Quick wakeup again, capping sleep timeout at 5 minutes.");
            wakeup_count = 4;
        }
    } else {
        wakeup_count = 0;
        current_timeout = BASE_TIMEOUT;
        LOG_INF("Long inactivity or fresh start, resetting sleep timeout to 1 minute.");
    }
    last_sleep_time = now_ms;
}

void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    update_sleep_timeout();
    k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
    LOG_DBG("Pin activity detected, resetting inactivity timer to %d seconds.", K_TIMEOUT_TICKS(current_timeout) / 1000);
}

void check_battery_status(void)
{
    int16_t buf;
    struct adc_sequence sequence = {
        .channels = BIT(adc_channel.channel_id),
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .resolution = 12,
    };

    if (adc_channel.dev && device_is_ready(adc_channel.dev)) {
        if (adc_sequence_init_dt(&adc_channel, &sequence) == 0) {
            if (adc_read(adc_channel.dev, &sequence) == 0) {
                uint32_t adc_value = buf;
                uint32_t voltage_mv = (adc_value * BATTERY_MAX_VOLTAGE) / (1 << 12);  // 12-bit ADC
                battery_level = ((voltage_mv - BATTERY_MIN_VOLTAGE) * 100) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
                if (battery_level < 0) battery_level = 0;
                if (battery_level > 100) battery_level = 100;

                LOG_INF("Battery level: %d%%, Voltage: %d mV", battery_level, voltage_mv);

                // Power-on battery status
                if (k_uptime_get() < K_TIMEOUT_TICKS(K_SECONDS(5))) {
                    if (battery_level > 75) {
                        set_led_status(LED_GREEN);
                    } else if (battery_level > 50) {
                        set_led_status(LED_YELLOW);
                    } else if (battery_level > 25) {
                        set_led_status(LED_ORANGE);
                    } else {
                        set_led_status(LED_RED);
                    }
                    k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
                }

                // Charging detection
                int64_t now_ms = k_uptime_get();
                if (now_ms - last_voltage_time >= 1000) { // Check every 1 second
                    if (last_voltage_mv > 0) {
                        if ((voltage_mv - last_voltage_mv) > CHARGING_THRESHOLD_DELTA) {
                            is_charging = true;
                            k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT); // Start pulsating
                            battery_warning_active = false; // Disable warning during charging
                            k_timer_stop(&battery_warning_timer);
                            k_timer_stop(&battery_pulse_timer);
                            LOG_INF("Charging detected, starting green pulse.");
                        } else if (is_charging && voltage_mv >= FULL_CHARGE_THRESHOLD) {
                            k_timer_stop(&charging_pulse_timer);
                            set_led_status(LED_GREEN); // Static green at full charge
                            max_voltage_mv = voltage_mv;
                            LOG_INF("Full charge reached, static green.");
                        } else if (!is_charging && last_voltage_mv > 0 && (voltage_mv - last_voltage_mv) <= -CHARGING_THRESHOLD_DELTA) {
                            // Voltage drop without charging, do nothing (LED off)
                        }
                    }
                    last_voltage_time = now_ms;
                    last_voltage_mv = voltage_mv;
                    if (voltage_mv > max_voltage_mv) {
                        max_voltage_mv = voltage_mv; // Update max voltage
                    }
                }

                // Critical battery warning (only if not charging)
                if (battery_level < BATTERY_CRITICAL_THRESHOLD && !is_charging && !battery_warning_active) {
                    battery_warning_active = true;
                    k_timer_start(&battery_warning_timer, K_NO_WAIT, K_NO_WAIT);
                }
            } else {
                send_error_log("ADC read failed");
            }
        } else {
            send_error_log("ADC sequence init failed");
        }
    } else {
        send_error_log("ADC device not ready");
    }
}

void sample_and_transmit(struct k_timer *timer)
{
    uint8_t payload_data[2];
    payload_data[0] = gpio_pin_get_dt(&data_plus);
    payload_data[1] = gpio_pin_get_dt(&data_minus);

    if (payload_data[0] != last_payload_data[0] || payload_data[1] != last_payload_data[1]) {
        k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
        tx_payload.length = sizeof(payload_data);
        memcpy(tx_payload.data, payload_data, sizeof(payload_data));
        
        last_payload_data[0] = payload_data[0];
        last_payload_data[1] = battery_warning_active ? 1 : payload_data[1];  // Example data tweak for warning

        if (esb_write_payload(&tx_payload) == 0) {
            LOG_INF("TX -> D+: %d, D-: %d", payload_data[0], payload_data[1]);
        } else {
            send_error_log("ESB write payload failed");
        }
    }
}


static void transmitter_thread(void *arg1, void *arg2, void *arg3)
{
    int err;

    LOG_INF("Starting Wireless Keyboard Transmitter...");

    /* --- Initialize GPIOs --- */
    if (!device_is_ready(&data_plus) || !device_is_ready(&data_minus)) {
        send_error_log("Data GPIOs not ready");
        return;
    }
    gpio_pin_configure_dt(&data_plus, GPIO_INPUT);
    gpio_pin_configure_dt(&data_minus, GPIO_INPUT);

    /* --- Initialize LEDs --- */
    if (!pwm_is_ready_dt(&red_pwm) || !pwm_is_ready_dt(&green_pwm) || !pwm_is_ready_dt(&blue_pwm)) {
        send_error_log("PWM devices not ready");
        return;
    }

    /* --- Power-On LED Status --- */
    set_led_status(LED_GREEN); // Initial state, will be updated by check_battery_status
    k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);  // Green LED for 5 seconds on power-on
    check_battery_status();  // Check and indicate battery status

    /* --- Configure GPIO Interrupt for Wake-up --- */
    static struct gpio_callback pin_cb_data;
    gpio_init_callback(&pin_cb_data, pin_activity_handler, BIT(data_plus.pin) | BIT(data_minus.pin));
    err = gpio_add_callback(data_plus.port, &pin_cb_data);
    if (err) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Failed to add callback, err %d", err);
        send_error_log(msg);
        return;
    }
    gpio_pin_interrupt_configure_dt(&data_plus, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&data_minus, GPIO_INT_EDGE_BOTH);

    /* --- Initialize ESB --- */
    struct esb_config config = ESB_DEFAULT_CONFIG;
    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.mode = ESB_MODE_PTX;
    config.bitrate = ESB_BITRATE_2MBPS;
    config.retransmit_count = 3;
    config.payload_length = 32;  // Increased to accommodate error messages

    err = esb_init(&config);
    if (err) {
        char msg[32];
        snprintf(msg, sizeof(msg), "ESB initialization failed, err %d", err);
        send_error_log(msg);
        return;
    }

    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err) { 
        send_error_log("Failed to set base address 0"); 
        return; 
    }

    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err) { 
        send_error_log("Failed to set prefixes"); 
        return; 
    }
    
    LOG_INF("Transmitter initialized successfully");
    
    /* --- Start Timers --- */
    k_timer_start(&sampling_timer, K_NO_WAIT, K_MSEC(10));
    k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);

    /* --- Thread Exits, Idle Thread Takes Over --- */
}

int main(void)
{
    int err;
    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        err = usb_enable(NULL);
        if (err) {
            LOG_ERR("Failed to enable USB, err %d", err);
            return 0;
        }
        LOG_INF("USB enabled successfully.");
    }

    k_thread_create(&transmitter_thread_data, transmitter_stack,
                    K_THREAD_STACK_SIZEOF(transmitter_stack),
                    transmitter_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    return 0;
}

static void send_error_log(const char *msg)
{
    if (msg && esb_is_idle()) {
        struct esb_payload error_payload;
        error_payload.noack = false;
        error_payload.length = strlen(msg) + 1;  // Include null terminator
        if (error_payload.length > sizeof(error_payload.data)) {
            error_payload.length = sizeof(error_payload.data);  // Truncate if too long
        }
        memcpy(error_payload.data, msg, error_payload.length);
        error_payload.data[0] |= ERROR_PAYLOAD_FLAG;  // Set flag to indicate error
        if (esb_write_payload(&error_payload) != 0) {
            // Fallback if ESB write fails (optional logging to console if desired)
        }
    }
}
