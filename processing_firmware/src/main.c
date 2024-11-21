#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>

#include <system_tasks.h>
#include <processing.h>

#include <zephyr/drivers/flash.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

void button_press_handler() {
	if (get_recording_state()) {
		stop_recording();
		printk("Stopped.\n");
	} else {
		printk("Starting...\n");
		start_recording();
	}
}

static const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

/* UART payload buffer element size. */
#define UART_BUF_SIZE 256

#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50000 /* Wait for RX complete event time in microseconds. */

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_work_delayable uart_work;

K_SEM_DEFINE(nus_write_sem, 0, 1);

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		//LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf,
					   struct uart_data_t,
					   data[0]);
		}

		tx_done();

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			//LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		//LOG_DBG("UART_RX_RDY");
		/*buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if (!true) {   // Discard unrelevant messages
			buf->len = 0;
		}

		disable_req = true;
		uart_rx_disable(uart);
*/
		break;

	case UART_RX_DISABLED:
		//LOG_DBG("UART_RX_DISABLED");
		/*disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			//LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_RX_TIMEOUT);
*/
		break;

	case UART_RX_BUF_REQUEST:
		//LOG_DBG("UART_RX_BUF_REQUEST");
		/*buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			//LOG_WRN("Not able to allocate UART receive buffer");
		}
*/
		break;

	case UART_RX_BUF_RELEASED:
		//LOG_DBG("UART_RX_BUF_RELEASED");
		/*buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}
*/
		break;

	case UART_TX_ABORTED:
		//LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data[0]);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item) {
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		//LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
}

static int uart_init(void) {
	int err;
	struct uart_data_t *rx;

	if (!device_is_ready(uart)) {
		//LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_RX_TIMEOUT);
}

void process_incoming_UART_message(uint8_t *buf_data, uint16_t buf_len) {
	return;
}

void send_UART_data(char *buf_data, uint8_t len) {
	struct uart_data_t *tx = k_malloc(sizeof(*tx));

	tx->len = len;
	memcpy(&tx->data[0], buf_data, len);

	if (uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS)) {
		k_fifo_put(&fifo_uart_tx_data, tx);
	}
}

void before_sleep() {
	k_msleep(200);
	printk("SLEEPING\n");
}

#define UART_RX_THREAD_STACK_SIZE 500
#define UART_RX_THREAD_PRIORITY 5

extern void handle_uart_rx(void *, void *, void *) {
	while (1) {
		/* Wait indefinitely for data to be sent over Bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
		process_incoming_UART_message(buf->data, buf->len);
		k_free(buf);
	}
};

K_THREAD_DEFINE(uart_rx_thread_tid, UART_RX_THREAD_STACK_SIZE,
                handle_uart_rx, NULL, NULL, NULL,
                UART_RX_THREAD_PRIORITY, 0, 0);

static const struct gpio_dt_spec power = GPIO_DT_SPEC_GET(POWER_SW_NODE, gpios);

int main(void) {
	system_init(&before_sleep, &button_press_handler);
	k_msleep(500);

	NRF_CLOCK_S->HFCLKCTRL = (CLOCK_HFCLKCTRL_HCLK_Div1 << CLOCK_HFCLKCTRL_HCLK_Pos);

	printk("Starting\n");

	uart_init();

	processing_init();

	clear_sytem_error();

	while (1) {
		k_msleep(5000);
	}

	return(0);	
}
