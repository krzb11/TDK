#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/poweroff.h>
#include <hal/nrf_gpio.h>
#include <zephyr/pm/device.h>

#define LED_STAT_NODE 	DT_ALIAS(led0)
#define LED_BLE_NODE 	DT_ALIAS(led1)
#define LED_BATT_NODE 	DT_ALIAS(led2)
#define LED_CHG_NODE 	DT_ALIAS(led3)

#define POWER_SW_NODE	DT_ALIAS(powersw)

#define WAKE_SW_NODE	DT_ALIAS(sw0)

/* Thread sleep in seconds between blinking work submits. */
#define BLINKING_SLEEP 2

/* Milliseconds required keeping the wake button pressed to perform system wake-up. */
#define SYSTEM_WAKE_LENGTH 2000

/* Milliseconds required keeping the wake button pressed to perform button function. */
#define BUTTON_MIN 10

#define BLINK_STACK_SIZE 512
#define BLINK_PRIORITY 5

/* Processes new work submitted during device wake-up. Typically turns on the LEDs and turns them off 
   after specified time required pressing the button to wake up the device. */
void handle_wake_blink(struct k_work *work);

/* Processes new work submitted periodically by blink_timer. 
   Tipically submits further works to blink the LEDs according to the system state. */
void handle_blink(struct k_work *work);

/* Submits new work periodically to blink the LEDs. */
void blink_timer_handler(struct k_timer *timer);

/* Button interrupt routine performed on first falling edge right after successfully waking up the device. */
void wake_button_init_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* Button interrupt routine performed on both rising and falling edge. */
void wake_button_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* Processes new work submitted by wake button interrupt routines. */
void handle_button(struct k_work *item);

/* Processes new work submitted to blink the STAT LED in normal operation. */
void handle_normal_status_blink(struct k_work *work);

/* Processes new work submitted to blink the STAT LED during signal recording and other continous operations. */
void handle_recording_status_blink(struct k_work *work);

/* Processes new work submitted to blink the ERR LED in case of system error. */
void handle_error_blink(struct k_work *work);

/* Increases the system error counter. */
void raise_sytem_error();

/* Decreases the system error counter. */
void clear_sytem_error();

/* Reset the auto sleep counter to zero, but it does not start, nor stops it. */
void reset_system_auto_sleep_counter();

/* Stops and resets the auto sleep counter. */
void stop_system_auto_sleep_counter();

/* Resets and start the auto sleep counter. */
void start_system_auto_sleep_counter();

/* Thread sleep in milliseconds between battery voltage measurements. */
#define BATTERY_CHECK_SLEEP 5000

/* The maximal voltage of the battery pack in millivolts.
   This is the value of the battery, when it is fully charged. */
#define BATTERY_CHARGED_VALUE 4200

/* The turn-off voltage of the battery pack in millivolts. */
#define BATTERY_OFF_VALUE 2800

/* The critical power percentage of the battery pack. */
#define BATTERY_CRITICAL_VALUE 2

/* The warning power percentage of the battery pack. */
#define BATTERY_WARNING_VALUE 20

/* Threshold in millivolts during battery readings checking battery_charging state. */
#define BATTERY_THRESHOLD 50

#define ADC_NODE DT_NODELABEL(adc)

/* Returns true if the battery voltage reaches the critical value. */
bool check_battery();

/** Reads and processes the battery voltage level.
    @return the actual reading value in millivolts. */
int32_t read_battery();

#define SYSTEM_TASKS_THREAD_PRIORITY 0

/** Checks if wake-up conditions are met. Once the device is successfully waked up, it initiates the system_tasks thread.
   @param on_system_sleep_cb callback funtion pointer. Called before system enters in sleep mode. */
void system_init(void (*on_system_sleep_cb)(void), void (*on_button_press_cb)(void));

void system_sleep();

/* Function executed by the system_tasks thread. 
   Initiates the system peripherials and starts the timers to perform system supervision tasks.*/
void system_tasks(void *, void *, void *);