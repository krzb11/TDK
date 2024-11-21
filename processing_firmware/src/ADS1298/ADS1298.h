/**
 * @file
 * @brief Library header file for TI ADS1298 ADC.
 * @author Balázs Kráz @ PPKE ITK - Allonic
 * @date 2024
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "ADS129x_codes.h"

#define UNERESP 2001    // Unexpected response error

#define TCLK    0.514   // ADC CLK period length in us

/**
 * @brief Enumerator for data rate settings.
 * Values are valid in HR mode.
 * 
 */
enum datarate {
    DR_32kSPS,
    DR_16kSPS,
    DR_8kSPS,
    DR_4kSPS,
    DR_2kSPS,
    DR_1kSPS,
    DR_500SPS
};

/**
 * @brief Enumerator for channel gain settings.
 * 
 */
enum gain {
    G_1,
    G_2,
    G_3,
    G_4,
    G_6,
    G_8,
    G_12
};

/**
 * @brief Struct definition of the ADC device.
 * Contais device pointers, configuration and status values.
 * 
 */
struct ADS1298 {
    struct device *spi_dev;           // Device pointer of the SPI device
    struct gpio_dt_spec *drdy_spec;   // GPIO device pointer of DRDY pin (active low)
    struct spi_config spi_cfg;              // SPI  configuration structure
    struct gpio_callback drdy_cb_data;      // GPIO callback structure of DRDY pin

    void (*callback_fn)();                  // Callback function pointer for continous ADC data reading
    struct k_work work;                     // Work structure for workqueue
    struct k_poll_signal signal;            // Signal raised when async read is finished

    uint8_t data_buffer[27];                // Data buffer for ADC reading
    struct spi_buf data_rx_bufs[1];         // SPI buffers for ADC data reading
	struct spi_buf_set data_rx_buf_set;     // SPI bufferset for for ADC reading

    bool standby_mode;                      // ADC standby status, false in normal operation
    bool high_resolution_mode;              // ADC HR (true) or LP (false) mode
    bool rdatac_mode;                       // ADC Read Data Continous mode status
    bool conversion_status;                 // ADC conversion status
    uint8_t enabled_channels;               // Enabled channels byte
    enum datarate datarate;                 // ADC data rate applicabble to HR mode
    enum gain gain[8];                      // Gain settings of each channel

    uint8_t drive_channel;                  // Channel ID used as RLD out (%2=0: CH_n/2_P, %2=1: CH_n/2_N)

    bool continous_reading;                 // Continous reading status in RDATAC mode
};

/**
 * @brief Initialize the device stuct and creates SPI controller configuration. 
 * 
 * @param ads_device the device to be configured
 * @param spi_dev SPI device defined in devicetree
 * @param spi_cfg spi_config for SPI configuration
 */
void init_ads(struct ADS1298 *ads_device, const struct device *spi_dev, const struct gpio_dt_spec *drdy_spec, struct spi_config spi_cfg);

/**
 * @brief Test the SPI communication and configure the ADC device.
 * 
 * @param ads_device the device to be configured
 * @param high_resolution high-resolution or low-power mode status
 * @param dr datarate enumerator for sample rate
 * @param rdatac_mode RDATAC mode status
 * @param enabled_channels byte of enabled channels
 * @param gain gain enumerator array for each channel gain settings
 * @return int 0 if successful, negative errno code in failure
 */
int config_ads(struct ADS1298 *ads_device, bool high_resolution, enum datarate dr, bool rdatac_mode, uint8_t enabled_channels, enum gain gain[8]);

/**
 * @brief Put the ADC device to low-power standby or normal mode.
 * 
 * @param ads_device the target device
 * @param standby_mode true to set standby, false to set normal mode
 * @return int 0 if successful, negative errno code in failure 
 */
int set_ads_standby(struct ADS1298 *ads_device, bool standby_mode);

/**
 * @brief Reset the ADC, clear the registers.
 * 
 * @param ads_device the target device
 * @return int 0 if successful, negative errno code in failure 
 */
int reset_ads(struct ADS1298 *ads_device);

/**
 * @brief Set the ADC device to high-resolution or low-power mode. 
 * 
 * @param ads_device the target device
 * @param high_resolution true to set HR, false to set LP mode
 * @return int 0 if successful, negative errno code in failure
 */
int set_ads_mode(struct ADS1298 *ads_device, bool high_resolution);

/**
 * @brief Configures the ADC channels.
 * 
 * @param ads_device the target device
 * @param enabled_channels byte of enabled channels
 * @param gain gain enumerator array of gain values per channel
 * @return int 0 if successful, negative errno code in failure 
 */
int set_ads_channels(struct ADS1298 *ads_device, uint8_t enabled_channels, enum gain gain[8]);

/**
 * @brief Set the ADC sample rate. 
 * In LP mode, the data rate drops to half the value.
 * 
 * @param ads_device the target device
 * @param dr datarate enumerator value valid in HR mode
 * @return int 0 if successful, negative errno code in failure 
 */
int set_ads_data_rate(struct ADS1298 *ads_device, enum datarate dr);

/**
 * @brief Set the ADC contionous read mode. 
 * RDATAC or SDATAC mode.
 * 
 * @param ads_device the target device
 * @param rdatac_mode RDATAC if true, SDATAC if false
 * @return int 0 if successful, negative errno code in failure 
 */
int set_ads_read_mode(struct ADS1298 *ads_device, bool rdatac_mode);

/**
 * @brief Set the ADC RLD out channel. 
 * 
 * @param ads_device the target device
 * @param channel_number output channel number
 * @param channel_P outout channel polarity (P if true, N if false)
 * @return int 0 if successful, negative errno code in failure 
 */
int set_ads_drive_channel(struct ADS1298 *ads_device, uint8_t channel_number, bool channel_P);

/**
 * @brief Send single command to the ADC.
 * 
 * @param ads_device the target device
 * @param cmd command to be sent
 * @return int 0 if successful, negative errno code in failure 
 */
int send_ads_command(const struct ADS1298 *ads_device, uint8_t cmd);

/**
 * @brief Write single register of the ADC device.
 * 
 * @param ads_device the target device
 * @param add register address
 * @param val new register value
 * @return int 0 if successful, negative errno code in failure 
 */
int write_register(const struct ADS1298 *ads_device, uint8_t add, uint8_t val);

/**
 * @brief Write multiple registers of the ADC device.
 * 
 * @param ads_device the target device
 * @param add starting register address
 * @param vals array of new register values
 * @param length length of vals array
 * @return int 0 if successful, negative errno code in failure 
 */
int write_registers(const struct ADS1298 *ads_device, uint8_t add, uint8_t vals[], uint8_t length);

/**
 * @brief Read single register of the ADC device.
 * 
 * @param ads_device the target device
 * @param add register address
 * @return int register value if successful, negative errno code in failure 
 */
int read_register(const struct ADS1298 *ads_device, uint8_t add);

/**
 * @brief Single ADC data read into the data_buffer of the ads_device.
 * 
 * @param ads_device the target device
 * @return int 0 if successful, negative errno code in failure
 */
int read_ads_data(struct ADS1298 *ads_device);

/**
 * @brief Start continous ADC data reading into the data_buffer of the ads_device.
 * 
 * @param ads_device the target device
 * @param callback_fn callback funtion pointer evoked when new data is available
 * @return int 0 if successful, negative errno code in failure
 */
int start_ads_reading(struct ADS1298 *ads_device, gpio_callback_handler_t callback_fn);

/**
 * @brief Stop ADC data reading.
 * 
 * @param ads_device the target device
 * @return int 0 if successful, negative errno code in failure
 */
int stop_ads_reading(struct ADS1298 *ads_device);

/**
 * @brief Set the ADC conversion status.
 * Conversion starts and waits tsettle time according to data rate and power mode if true.
 * Waiting with k_sleep().
 * 
 * @param ads_device the target device
 * @param conversion_status starts if true, stops if false
 * @return int 0 if successful, negative errno code in failure
 */
int set_ads_conversion(struct ADS1298 *ads_device, bool conversion_status);

/**
 * @brief Helper function for ADC data reading. Do not use this function at itself. 
 * Read data into the data_buffer of the ads_device.
 * 
 * @param ads_device the target device
 * @return int 0 if successful, negative errno code in failure
 */
int _get_ads_data(struct ADS1298 *ads_device);

/**
 * @brief Helper function for asynchronous ADC data reading. 
 * Read data into the data_buffer of the ads_device.
 * 
 * @param ads_device the target device
 * @return int 0 if successful, negative errno code in failure
 */
int _get_ads_data_async(struct ADS1298 *ads_device);