#include "system_tasks.h"

static const struct gpio_dt_spec led_stat = GPIO_DT_SPEC_GET(LED_STAT_NODE, gpios);
static const struct gpio_dt_spec led_ble = GPIO_DT_SPEC_GET(LED_BLE_NODE, gpios);
static const struct gpio_dt_spec led_batt = GPIO_DT_SPEC_GET(LED_BATT_NODE, gpios);
static const struct gpio_dt_spec led_chg = GPIO_DT_SPEC_GET(LED_CHG_NODE, gpios);

static const struct gpio_dt_spec power_sw = GPIO_DT_SPEC_GET(POWER_SW_NODE, gpios);

static const struct gpio_dt_spec wake_button = GPIO_DT_SPEC_GET(WAKE_SW_NODE, gpios);
static struct gpio_callback wake_button_cb_data;

const struct gpio_dt_spec batt_chg_spec = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 5,
	.dt_flags = GPIO_ACTIVE_LOW
};
static struct gpio_callback batt_chg_cb_data;

K_THREAD_STACK_DEFINE(blink_stack_area, BLINK_STACK_SIZE);

/* Custom workque designated only to process blink works. */
struct k_work_q blink_work_q;

/* Work submitted by wake button ISRs and processed by handle_button function. */
struct k_work button_work;

K_WORK_DEFINE(wake_blink_work, handle_wake_blink);
K_WORK_DEFINE(blink_work, handle_blink);
K_WORK_DEFINE(normal_status_blink_work, handle_normal_status_blink);
K_WORK_DEFINE(recording_status_blink_work, handle_recording_status_blink);
K_WORK_DEFINE(error_blink_work, handle_error_blink);

K_TIMER_DEFINE(blink_timer, blink_timer_handler, NULL);

bool system_standby_status = true;

uint8_t system_error_count = 0;

/* Charging (true) or discharging (false) state of the battery.
   It is detemined based on the trend of the consecutive battery pack voltage measurements. */
bool battery_charging = false;
bool charger_connected = false;

/* State of the auto sleep counter. Also shows if recording or other continous process is going on. */
bool auto_sleep_counter_running = true;
uint32_t auto_sleep_counter = 0;
uint32_t auto_sleep_counter_limit = 600 / BLINKING_SLEEP;	// 600 [sec] -> 10 minutes

int64_t wake_button_pressed_time = 0;
int64_t wake_button_released_time = 0;

void handle_wake_blink(struct k_work *work) {
	k_msleep(SYSTEM_WAKE_LENGTH);
	gpio_pin_set_dt(&led_stat, 0);
	gpio_pin_set_dt(&led_ble, 0);
	gpio_pin_set_dt(&led_batt, 0);
	gpio_pin_set_dt(&led_chg, 0);
}

void blink_timer_handler(struct k_timer *timer) {
    k_work_submit(&blink_work);
}

void wake_button_init_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	wake_button_pressed_time = k_uptime_get();
	gpio_pin_interrupt_configure_dt(&wake_button, GPIO_INT_EDGE_BOTH);
	gpio_remove_callback(wake_button.port, &wake_button_cb_data);
	gpio_init_callback(&wake_button_cb_data, wake_button_interrupt, BIT(wake_button.pin));
	gpio_add_callback(wake_button.port, &wake_button_cb_data);
}

void wake_button_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	if (gpio_pin_get_dt(&wake_button)) {   // Rising edge
		wake_button_pressed_time = k_uptime_get();
		k_timer_stop(&blink_timer);   // Stops the blink_timer in order to turn on the LEDs
		gpio_pin_set_dt(&led_stat, 1);   // Resets the LED if the wake button released before specified time
		gpio_pin_set_dt(&led_ble, 1);
		k_work_submit_to_queue(&blink_work_q, &wake_blink_work);   // Shows if the wake button can be released after specified time
	} else {   // Falling edge
		wake_button_released_time = k_uptime_get();
		gpio_pin_set_dt(&led_stat, 0);   // Resets the LED if the wake button released before specified time
		gpio_pin_set_dt(&led_ble, 0);
		k_work_submit(&button_work);
		k_timer_start(&blink_timer, K_MSEC(500), K_SECONDS(2));   // Resets the blink_timer
	}
}

void battery_charge_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	if (gpio_pin_get_dt(&batt_chg_spec)) {   // Rising edge
		gpio_pin_set_dt(&led_chg, 1);
		gpio_pin_set_dt(&led_batt, 1);
		charger_connected = true;
		battery_charging = true;
	} else {   // Falling edge
		gpio_pin_set_dt(&led_batt, 0);
		battery_charging = false;
		read_battery();
	}
}

/* Function pointer of callback function called on simple button press. */
void (*button_press_cb)(void) = NULL;

void handle_button(struct k_work *item) {
	if (wake_button_released_time-wake_button_pressed_time >= SYSTEM_WAKE_LENGTH) {
		if (!system_standby_status) {
			system_sleep();
		}
	} else if (wake_button_released_time-wake_button_pressed_time >= BUTTON_MIN) {
		if (!system_standby_status) {
			if (button_press_cb != NULL) {
				button_press_cb();
			}
		}
	}
}

/* *--------------- */
void handle_normal_status_blink(struct k_work *work) {
	gpio_pin_set_dt(&led_stat, 1);
	k_msleep(50);
	gpio_pin_set_dt(&led_stat, 0);
}

/* *--******------- */
void handle_recording_status_blink(struct k_work *work) {
	gpio_pin_set_dt(&led_stat, 1);
	k_msleep(50);
	gpio_pin_set_dt(&led_stat, 0);
	k_msleep(100);
	gpio_pin_set_dt(&led_stat, 1);
	k_msleep(500);
	gpio_pin_set_dt(&led_stat, 0);
}

/* ************---- */
void handle_error_blink(struct k_work *work) {}

void handle_blink(struct k_work *work) {
    if (system_error_count > 0) {
		k_work_submit_to_queue(&blink_work_q, &error_blink_work);
	}

	if (!auto_sleep_counter_running) {   // Recording in progeress
		k_work_submit_to_queue(&blink_work_q, &recording_status_blink_work);
	} else {
		k_work_submit_to_queue(&blink_work_q, &normal_status_blink_work);
		auto_sleep_counter++;
		if (auto_sleep_counter >= auto_sleep_counter_limit) {
            //system_sleep();
		}
	}
}

void raise_sytem_error() {
	if (system_error_count < 255) {
		system_error_count++;
	}
}

void clear_sytem_error() {
	if (system_error_count > 0) {
		system_error_count--;
	}
}

void reset_system_auto_sleep_counter() {
	auto_sleep_counter = 0;
}

void stop_system_auto_sleep_counter() {
	k_timer_stop(&blink_timer);
	auto_sleep_counter_running = false;
	auto_sleep_counter = 0;
}

void start_system_auto_sleep_counter() {
	auto_sleep_counter = 0;
	auto_sleep_counter_running = true;
	k_timer_start(&blink_timer, K_MSEC(500), K_SECONDS(2));
}

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

struct adc_channel_cfg battery_adc_ch_cfg = {
	.gain = ADC_GAIN_1_6,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 0,
	.input_positive = SAADC_CH_PSELN_PSELN_AnalogInput0
};

static int16_t battery_adc_sample_buffer;
int32_t battery_pre_mv_value;

struct adc_sequence battery_adc_sequence = {
	.channels = BIT(0),
	.buffer = &battery_adc_sample_buffer,
	.buffer_size = sizeof(battery_adc_sample_buffer),
	.resolution = 10
};

bool check_battery() {
    if (adc_read(adc_dev, &battery_adc_sequence) < 0) {
		//printk("ADC error");
        return false;
    }

    int32_t battery_mv_value = battery_adc_sample_buffer;
	adc_raw_to_millivolts(adc_ref_internal(adc_dev), battery_adc_ch_cfg.gain, battery_adc_sequence.resolution, &battery_mv_value);

    battery_mv_value = battery_mv_value*3.609-49.514;
	battery_pre_mv_value = battery_mv_value;

	int perc = (float)((battery_mv_value-BATTERY_OFF_VALUE)/(float)(BATTERY_CHARGED_VALUE-BATTERY_OFF_VALUE))*100.0;

    if (battery_mv_value > BATTERY_OFF_VALUE && (perc > BATTERY_CRITICAL_VALUE)) {
        return true;
    }

	//printk("BATT: %d [mv], %d", battery_mv_value, perc);

    return false;
}

int32_t read_battery() {
    int32_t battery_mv_value = battery_adc_sample_buffer;
	adc_raw_to_millivolts(adc_ref_internal(adc_dev), battery_adc_ch_cfg.gain, battery_adc_sequence.resolution, &battery_mv_value);

	battery_mv_value = battery_mv_value*3.609-49.514;

    if (battery_pre_mv_value > battery_mv_value + BATTERY_THRESHOLD) {
        battery_pre_mv_value = battery_mv_value;
    } else if (battery_pre_mv_value < battery_mv_value - BATTERY_THRESHOLD) {
        battery_pre_mv_value = battery_mv_value;
    }

	if (battery_pre_mv_value > 4400) {
		charger_connected = true;
		gpio_pin_set_dt(&led_chg, 1);
	} else if (!battery_charging) {
		charger_connected = false;
		gpio_pin_set_dt(&led_chg, 0);
	}

	if (battery_pre_mv_value < 3000) {
		gpio_pin_set_dt(&led_batt, 1);
	} else {
		gpio_pin_set_dt(&led_batt, 0);
	}

    adc_read_async(adc_dev, &battery_adc_sequence, NULL);

	return battery_mv_value;
}

K_THREAD_STACK_DEFINE(system_tasks_stack_area, 1024);
struct k_thread system_tasks_thread_data;

/* Function pointer of callback function called before system enters in sleep mode. */
void (*system_sleep_cb)(void) = NULL;

void system_sleep() {
    gpio_pin_set_dt(&power_sw, 0);
    gpio_pin_set_dt(&led_stat, 0);
    gpio_pin_set_dt(&led_ble, 0);

    //printk("STANDBY\n");

    if (system_sleep_cb != NULL) {
        system_sleep_cb();
    }

    system_standby_status = true;
	gpio_pin_configure_dt(&led_stat, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_ble, GPIO_OUTPUT_INACTIVE);
    sys_poweroff();
}

void system_init(void (*on_system_sleep_cb)(void), void (*on_button_press_cb)(void)) {
    system_sleep_cb = on_system_sleep_cb;
	button_press_cb = on_button_press_cb;

    gpio_pin_configure_dt(&wake_button, GPIO_INPUT);
    gpio_pin_configure_dt(&batt_chg_spec, GPIO_INPUT);

	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(WAKE_SW_NODE, gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(WAKE_SW_NODE, gpios), NRF_GPIO_PIN_SENSE_LOW);

	adc_channel_setup(adc_dev, &battery_adc_ch_cfg);
	check_battery();

	k_msleep(500);

	if (!check_battery()) {
		k_msleep(1000);
		sys_poweroff();
	}

	while (system_standby_status && gpio_pin_get_dt(&wake_button)) {
		if (k_uptime_get() < SYSTEM_WAKE_LENGTH) {
			k_msleep(10);
		} else {
			system_standby_status = false;
			gpio_pin_configure_dt(&power_sw, GPIO_OUTPUT_ACTIVE);
		}
	}

	if (system_standby_status) {
		gpio_pin_configure_dt(&power_sw, GPIO_OUTPUT_INACTIVE);
		gpio_pin_configure_dt(&led_stat, GPIO_OUTPUT_INACTIVE);
		gpio_pin_configure_dt(&led_ble, GPIO_OUTPUT_INACTIVE);
		sys_poweroff();
	}

	k_msleep(500);

    k_thread_create(&system_tasks_thread_data, system_tasks_stack_area, K_THREAD_STACK_SIZEOF(system_tasks_stack_area), system_tasks, NULL, NULL, NULL, SYSTEM_TASKS_THREAD_PRIORITY, 0, K_NO_WAIT);
}

void system_tasks(void *, void *, void *) {
	gpio_pin_interrupt_configure_dt(&wake_button, GPIO_INT_EDGE_TO_ACTIVE);

	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(WAKE_SW_NODE, gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(WAKE_SW_NODE, gpios), NRF_GPIO_PIN_SENSE_LOW);

	gpio_init_callback(&wake_button_cb_data, wake_button_init_interrupt, BIT(wake_button.pin));
	gpio_add_callback(wake_button.port, &wake_button_cb_data);

	gpio_pin_interrupt_configure_dt(&batt_chg_spec, GPIO_INT_EDGE_BOTH);

	nrf_gpio_cfg_input(batt_chg_spec.pin, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(batt_chg_spec.pin, NRF_GPIO_PIN_SENSE_LOW);

	gpio_init_callback(&batt_chg_cb_data, battery_charge_interrupt, BIT(batt_chg_spec.pin));
	gpio_add_callback(batt_chg_spec.port, &batt_chg_cb_data);

	k_work_queue_init(&blink_work_q);
	k_work_queue_start(&blink_work_q, blink_stack_area, K_THREAD_STACK_SIZEOF(blink_stack_area), BLINK_PRIORITY, NULL);

	gpio_pin_configure_dt(&led_stat, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led_ble, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led_batt, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_chg, GPIO_OUTPUT_INACTIVE);

	k_msleep(600);
	gpio_pin_set_dt(&led_stat, 0);
	gpio_pin_set_dt(&led_ble, 0);

	k_work_init(&button_work, handle_button);

	k_timer_start(&blink_timer, K_SECONDS(2), K_SECONDS(2));

	if (gpio_pin_get_dt(&batt_chg_spec)) {   // Rising edge
		gpio_pin_set_dt(&led_chg, 1);
		gpio_pin_set_dt(&led_batt, 1);
		charger_connected = true;
		battery_charging = true;
	} else {   // Falling edge
		gpio_pin_set_dt(&led_batt, 0);
		battery_charging = false;
		read_battery();
	}

    while (1) {
		int32_t batt = read_battery();
		//printk("BATT PRE: %d, ACT: %d [mv]\n", battery_pre_mv_value, batt);
        k_msleep(BATTERY_CHECK_SLEEP);
    }
}