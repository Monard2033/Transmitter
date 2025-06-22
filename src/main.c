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
 #include <string.h>
 #include <zephyr/sys/poweroff.h> // For sys_poweroff
 #include <nrfx.h> // For nrf_gpio_ functions
 #include <hal/nrf_gpio.h> // Use the correct path for nrf_gpio functions
 
 LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);
 /* --- Function Forward Declarations --- */
 void sample_and_transmit(struct k_timer *timer);
 void enter_low_power(struct k_timer *timer);
 
 /* --- Pin Definitions from Device Tree --- */
 #define GPIO020_NODE   DT_ALIAS(datapluspin)
 #define GPIO020        DT_GPIO_FLAGS(GPIO020_NODE, gpios)
 static const struct gpio_dt_spec *data_plus = &((const struct gpio_dt_spec)GPIO_DT_SPEC_GET(GPIO020_NODE, gpios));
 
 #define GPIO022_NODE  DT_ALIAS(dataminuspin)
 #define GPIO022       DT_GPIO_FLAGS(GPIO022_NODE, gpios)
 static const struct gpio_dt_spec *data_minus = &((const struct gpio_dt_spec)GPIO_DT_SPEC_GET(GPIO022_NODE, gpios));
 
 #define GPIO017_NODE DT_ALIAS(spdtswitchpin)
 #define GPIO017      DT_GPIO_FLAGS(GPIO017_NODE, gpios)
 static const struct gpio_dt_spec *spdt_switch = &((const struct gpio_dt_spec)GPIO_DT_SPEC_GET(GPIO017_NODE, gpios));
 
 #define GPIO024_NODE         DT_ALIAS(ledred)
 #define GPIO024          DT_GPIO_FLAGS(GPIO024_NODE, gpios)
 static const struct gpio_dt_spec *led_red = &((const struct gpio_dt_spec)GPIO_DT_SPEC_GET(GPIO024_NODE, gpios));
 
 #define GPIO10_NODE       DT_ALIAS(ledgreen)
 #define GPIO10        DT_GPIO_FLAGS(GPIO10_NODE, gpios)
 static const struct gpio_dt_spec *led_green = &((const struct gpio_dt_spec)GPIO_DT_SPEC_GET(GPIO10_NODE, gpios));
 
 #define GPIO011_NODE        DT_ALIAS(ledblue)
 #define GPIO011        DT_GPIO_FLAGS(GPIO011_NODE, gpios)
 static const struct gpio_dt_spec *led_blue = &((const struct gpio_dt_spec)GPIO_DT_SPEC_GET(GPIO011_NODE, gpios));
 
 enum led_color { LED_RED, LED_GREEN, LED_BLUE, LED_OFF};
 
 /* --- ESB and Power Management --- */
 static struct esb_payload tx_payload;
 static uint8_t last_payload_data[2] = {0xFF, 0xFF}; // Initialize to a value that won't match first read
 
 static K_TIMER_DEFINE(sampling_timer, sample_and_transmit, NULL);
 static K_TIMER_DEFINE(inactivity_timer, enter_low_power, NULL);
 
 #define INACTIVITY_TIMEOUT K_SECONDS(60)
 #define SAMPLE_INTERVAL K_MSEC(10) // Changed to milliseconds for more realistic sampling
 
 /* --- Function Definitions --- */
 
 void set_led_status(enum led_color color)
 {
	 // Ensure the GPIO port is ready before trying to set the pin
	 if (led_red && device_is_ready(led_red->port)) {
		 gpio_pin_set_dt(led_red, (color == LED_RED));
	 }
	 if (led_green && device_is_ready(led_green->port)) {
		 gpio_pin_set_dt(led_green, (color == LED_GREEN));
	 }
	 if (led_blue && device_is_ready(led_blue->port)) {
		 gpio_pin_set_dt(led_blue, (color == LED_BLUE));
	 }
 }
 
 void enter_low_power(struct k_timer *timer)
 {
	 LOG_INF("Inactivity detected. Entering System ON idle mode.");
	 set_led_status(LED_BLUE); // Indicate idle state
	 esb_disable(); // Disable ESB to save power
	 // System will enter idle mode via k_sleep(K_FOREVER) in main
 }
 
 void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
 {
	 // Restart the inactivity timer on any pin activity
	 k_timer_start(&inactivity_timer, INACTIVITY_TIMEOUT, K_NO_WAIT);
	 LOG_DBG("Pin activity detected, resetting inactivity timer.");
 }
 
 void sample_and_transmit(struct k_timer *timer)
 {
	 uint8_t payload_data[2];
	 payload_data[0] = gpio_pin_get_dt(data_plus);
	 payload_data[1] = gpio_pin_get_dt(data_minus);
 
	 // Only transmit if the data has changed
	 if (payload_data[0] != last_payload_data[0] || payload_data[1] != last_payload_data[1]) {
		 // Reset inactivity timer because we have active data change
		 k_timer_start(&inactivity_timer, INACTIVITY_TIMEOUT, K_NO_WAIT);
		 set_led_status(LED_GREEN); // Indicate active transmission
 
		 // Prepare payload for ESB
		 tx_payload.length = sizeof(payload_data);
		 memcpy(tx_payload.data, payload_data, sizeof(payload_data));
		 
		 // Update last sent data
		 last_payload_data[0] = payload_data[0];
		 last_payload_data[1] = payload_data[1];
 
		 if (esb_write_payload(&tx_payload) == 0) {
			 LOG_INF("TX -> D+: %d, D-: %d", payload_data[0], payload_data[1]);
		 } else {
			 LOG_ERR("Failed to write payload");
			 set_led_status(LED_RED);
		 }
	 }
 }
 
 int main(void)
 {
	 /* ---- Initialize GPIOs --- */
	 int err;
 
	 LOG_INF("Starting Wireless Keyboard Transmitter...");
 
	 /* --- Initialize GPIOs --- */
	 if (!device_is_ready(spdt_switch->port)) {
		 LOG_ERR("SPDT Switch GPIO device not ready");
		 set_led_status(LED_RED);
		 return 0;
	 }
	 // Configure switch pin as input and wakeup source
	 err = gpio_pin_configure_dt(spdt_switch, GPIO_INPUT);
	 if (err) {
		 LOG_ERR("Failed to configure SPDT switch pin, err %d", err);
		 return 0;
	 }
	 // Configure SPDT switch as wakeup source for System OFF
	 nrf_gpio_cfg_input(spdt_switch->pin, GPIO_PULL_UP);
	 nrf_gpio_cfg_sense_set(spdt_switch->pin, GPIO_PIN_CNF_SENSE_Low); // Wake on LOW to HIGH transition
 
	 if (gpio_pin_get_dt(spdt_switch) == 0) {
		 LOG_WRN("SPDT switch is off. Entering System OFF mode.");
		 set_led_status(LED_RED);
		 sys_poweroff();
		 return 0; // Unreachable after sys_poweroff
	 }
 
	 // Configure data pins as inputs
	 if (!device_is_ready(data_plus->port) || !device_is_ready(data_minus->port)) {
		 LOG_ERR("Data GPIOs not ready");
		 return 0;
	 }
	 gpio_pin_configure_dt(data_plus, GPIO_INPUT);
	 gpio_pin_configure_dt(data_minus, GPIO_INPUT);
 
	 /* --- Initialize LEDs --- */
	 if (led_red && device_is_ready(led_red->port)) gpio_pin_configure_dt(led_red, GPIO_OUTPUT_INACTIVE);
	 if (led_green && device_is_ready(led_green->port)) gpio_pin_configure_dt(led_green, GPIO_OUTPUT_INACTIVE);
	 if (led_blue && device_is_ready(led_blue->port)) gpio_pin_configure_dt(led_blue, GPIO_OUTPUT_INACTIVE);
 
	 set_led_status(LED_GREEN); // Indicate device is on and active
 
	 /* --- Configure GPIO Interrupt for Wake-up --- */
	 static struct gpio_callback pin_cb_data;
	 gpio_init_callback(&pin_cb_data, pin_activity_handler, BIT(data_plus->pin) | BIT(data_minus->pin));
	 err = gpio_add_callback(data_plus->port, &pin_cb_data); // Use port from gpio_dt_spec
	 if (err) {
		 LOG_ERR("Failed to add callback on port, err %d", err);
		 return 0;
	 }
	 gpio_pin_interrupt_configure_dt(data_plus, GPIO_INT_EDGE_BOTH);
	 gpio_pin_interrupt_configure_dt(data_minus, GPIO_INT_EDGE_BOTH);
 
	 /* --- Initialize ESB --- */
	 struct esb_config config = ESB_DEFAULT_CONFIG;
	 config.protocol = ESB_PROTOCOL_ESB_DPL;
	 config.mode = ESB_MODE_PTX;
	 config.bitrate = ESB_BITRATE_2MBPS;
	 config.retransmit_count = 3;
	 config.payload_length = 2; // Fixed payload length
 
	 err = esb_init(&config);
	 if (err) {
		 LOG_ERR("ESB initialization failed, err %d", err);
		 return 0;
	 }
	 // Set a base address and pipe for communication
	 uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
	 err = esb_set_base_address_0(base_addr_0);
	 if (err) { LOG_ERR("Failed to set base address 0"); return 0; }
	 uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
	 // Set prefixes for the ESB pipes
	 err = esb_set_prefixes(prefixes, 8);
	 if (err) { LOG_ERR("Failed to set prefixes"); return 0; }
	 
	 LOG_INF("Transmitter initialized successfully");
	 
	 /* --- Start Timers --- */
	 k_timer_start(&sampling_timer, K_NO_WAIT, SAMPLE_INTERVAL);
	 k_timer_start(&inactivity_timer, INACTIVITY_TIMEOUT, K_NO_WAIT);
	 
	 // System will enter idle mode automatically when idle
	 while (1) {
		 k_sleep(K_FOREVER);
	 }
 
	 return 0;
 }