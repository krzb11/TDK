/**
 * @file
 * @brief Library for TI ADS1298 ADC
 * @author Balázs Kráz @ PPKE ITK - Allonic
 * @date 2024
 */

#include "ADS1298.h"


/**
 * @brief Array of data rate setting register flags. Indexable by datarate enumerator.
 * 
 */
const uint8_t datarate_flags[] = {CONFIG1_DR_32kSPS, CONFIG1_DR_16kSPS, CONFIG1_DR_8kSPS, CONFIG1_DR_4kSPS, CONFIG1_DR_2kSPS, CONFIG1_DR_1kSPS, CONFIG1_DR_500SPS};

/**
 * @brief Array of gain setting register flags. Indexable by gain enumerator.
 * 
 */
const uint8_t gain_flags[] = {CHSET_GAIN_1, CHSET_GAIN_2, CHSET_GAIN_3, CHSET_GAIN_4, CHSET_GAIN_6, CHSET_GAIN_8, CHSET_GAIN_12};

void init_ads(struct ADS1298 *ads_device, const struct device *spi_dev, const struct gpio_dt_spec *drdy_spec, const struct spi_config spi_cfg) {
    //gpio_pin_configure_dt(cs_spec, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(drdy_spec, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(drdy_spec, GPIO_INT_DISABLE);

    ads_device->spi_dev = spi_dev;
    //ads_device->cs_spec = cs_spec;
    ads_device->drdy_spec = drdy_spec;
    ads_device->spi_cfg = spi_cfg;

    /** Default values: */
    ads_device->enabled_channels = 0xFF;
    ads_device->high_resolution_mode = false;
    ads_device->datarate = DR_500SPS;
    ads_device->rdatac_mode = true;
    ads_device->gain[0] = G_6;
    ads_device->gain[1] = G_6;
    ads_device->gain[2] = G_6;
    ads_device->gain[3] = G_6;
    ads_device->gain[4] = G_6;
    ads_device->gain[5] = G_6;
    ads_device->gain[6] = G_6;
    ads_device->gain[7] = G_6;
    ads_device->standby_mode = false;
    ads_device->conversion_status = false;

    ads_device->drive_channel = 0;

    ads_device->continous_reading = false;

    ads_device->data_rx_bufs[0].buf = ads_device->data_buffer;
    ads_device->data_rx_bufs[0].len = 27;

    ads_device->data_rx_buf_set.buffers = ads_device->data_rx_bufs;
    ads_device->data_rx_buf_set.count = 1;

    k_poll_signal_init(&ads_device->signal);

    reset_ads(ads_device);
}

int config_ads(struct ADS1298 *ads_device, bool high_resolution, enum datarate dr, bool rdatac_mode, uint8_t enabled_channels, enum gain gain[8]) {
    int err = set_ads_mode(ads_device, high_resolution);
    if (err < 0) return err;

    err = set_ads_data_rate(ads_device, dr);
    if (err < 0) return err;

    err = set_ads_channels(ads_device, enabled_channels, gain);
    if (err < 0) return err;

    err = set_ads_read_mode(ads_device, rdatac_mode);
    if (err < 0) return err;

    err = write_register(ads_device, CONFIG3_REGISTER, CONFIG3_PD_REFBUF | BIT(6) | CONFIG3_RLDREF_INT | CONFIG3_PD_RLD);   // Enable RLD
    if (err < 0) return err;

    err = read_register(ads_device, ID_REGISTER);
    if (err < 0) {
        return err;
    } else if (err != 0x92) {
        return err;
    }

    return 0;
}

int set_ads_standby(struct ADS1298 *ads_device, bool standby_mode) {
    uint8_t cmd;

    if (standby_mode) {
        cmd = STANDBY_COMMAND;
    } else {
        cmd = WAKEUP_COMMAND;
    }

    int err = send_ads_command(ads_device, cmd);

    if (err == 0) {
        ads_device->standby_mode = standby_mode;
    }

    return err;
};

int reset_ads(struct ADS1298 *ads_device) {
    int err = send_ads_command(ads_device, RESET_COMMAND);

    if (err == 0) {
        ads_device->enabled_channels = 0xFF;
        ads_device->high_resolution_mode = false;
        ads_device->datarate = DR_500SPS;
        ads_device->rdatac_mode = true;
        ads_device->gain[0] = G_6;
        ads_device->gain[1] = G_6;
        ads_device->gain[2] = G_6;
        ads_device->gain[3] = G_6;
        ads_device->gain[4] = G_6;
        ads_device->gain[5] = G_6;
        ads_device->gain[6] = G_6;
        ads_device->gain[7] = G_6;
        ads_device->standby_mode = false;
        ads_device->conversion_status = false;
        ads_device->drive_channel = 0;
    }

    return err;
};

int set_ads_mode(struct ADS1298 *ads_device, bool high_resolution) {
    uint8_t val = datarate_flags[ads_device->datarate];

    if (high_resolution) val = val | CONFIG1_HR;

    int err = write_register(ads_device, CONFIG1_REGISTER, val);

    if (err == 0) {
        ads_device->high_resolution_mode = high_resolution;
    }

    return err;
};

int set_ads_channels(struct ADS1298 *ads_device, uint8_t enabled_channels, enum gain gain[8]) {
    uint8_t vals[8];

    for (uint8_t i = 0; i < 8; i++) {
        if (enabled_channels >> i & 0x01) {
            vals[i] = gain_flags[gain[i]] | /*CHSET_MUX_SHORT*/ CHSET_MUX_NORMAL /*CHSET_MUX_TEST*/ /*CHSET_MUX_MVDD*/;
        } else {
            vals[i] = CHSET_PD | gain_flags[gain[i]] | CHSET_MUX_SHORT;
        }
    }

    /* Set RLD out channel if needed */
    if (ads_device->drive_channel > 0) {
        uint8_t drive_ch_num = (ads_device->drive_channel-1)/2;
        enabled_channels = enabled_channels & ~(0x01 << drive_ch_num);
        
        if (ads_device->drive_channel % 2) {
            vals[drive_ch_num] = CHSET_PD | CHSET_MUX_RLD_DRN;  // set the n-th channel N as RLD out
        } else {
            vals[drive_ch_num] = CHSET_PD | CHSET_MUX_RLD_DRP; // set the n-th channel P as RLD out
        }
    }
    
    int err = write_registers(ads_device, CH1SET_REGISTER, vals, 8);

    if (err == 0) {
        ads_device->gain[0] = gain[0];
        ads_device->gain[1] = gain[1];
        ads_device->gain[2] = gain[2];
        ads_device->gain[3] = gain[3];
        ads_device->gain[4] = gain[4];
        ads_device->gain[5] = gain[5];
        ads_device->gain[6] = gain[6];
        ads_device->gain[7] = gain[7];
        ads_device->enabled_channels = enabled_channels;
    }

    err = write_register(ads_device, RLD_SENSP_REGISTER, enabled_channels);     // set RLD derivation channels
    if (err < 0) {
        return err;
    }

    err = write_register(ads_device, RLD_SENSN_REGISTER, enabled_channels);     // set RLD derivation channels
    if (err < 0) {
        return err;
    }

    return err;
};

int set_ads_data_rate(struct ADS1298 *ads_device, enum datarate dr) {
    uint8_t val = datarate_flags[dr];
    if (ads_device->high_resolution_mode) val = val | CONFIG1_HR;

    int err = write_register(ads_device, CONFIG1_REGISTER, val);

    if (err == 0) {
        ads_device->datarate = dr;
    }

    return err;
};

int set_ads_read_mode(struct ADS1298 *ads_device, bool rdatac_mode) {
    uint8_t cmd;

    if (rdatac_mode) {
        cmd = RDATAC_COMMAND;
    } else {
        cmd = SDATAC_COMMAND;
    }

    int err = send_ads_command(ads_device, cmd);

    if (err == 0) {
        ads_device->rdatac_mode = rdatac_mode;
    }

    return err;
};

int set_ads_drive_channel(struct ADS1298 *ads_device, uint8_t channel_number, bool channel_P) {
    ads_device->drive_channel = (channel_number-1)*2+1+(uint8_t)(channel_P);

    int err = set_ads_channels(ads_device, ads_device->enabled_channels, ads_device->gain);

    return err;
};

int send_ads_command(const struct ADS1298 *ads_device, uint8_t cmd) {
    uint8_t tx_buffer[1];
	struct spi_buf spi_tx_bufs[1];
	struct spi_buf_set spi_tx_buf_set;

    tx_buffer[0] = cmd;

    spi_tx_bufs[0].buf = tx_buffer;
    spi_tx_bufs[0].len = 1;
    
    spi_tx_buf_set.buffers = spi_tx_bufs;
    spi_tx_buf_set.count = 1;

    //gpio_pin_set_dt(ads_device->cs_spec, 1);
    int err = spi_write(ads_device->spi_dev, &ads_device->spi_cfg, &spi_tx_buf_set);
    //gpio_pin_set_dt(ads_device->cs_spec, 0);
    
    return err;
};

int write_register(const struct ADS1298 *ads_device, uint8_t add, uint8_t val) {
    uint8_t vals[1] = {val};

    return write_registers(ads_device, add, vals, 1);
}

int write_registers(const struct ADS1298 *ads_device, uint8_t add, uint8_t vals[], uint8_t length) {
    uint8_t tx_buffer[length+4];            // SDATAC, CMD+ADD, NUM, RDATAC 
	struct spi_buf spi_tx_bufs[1];
	struct spi_buf_set spi_tx_buf_set;

    if (ads_device->rdatac_mode) {
        tx_buffer[0] = SDATAC_COMMAND;      // stop RDATAC
		tx_buffer[1] = WREG_COMMAND | add;  // write at address
		tx_buffer[2] = length-1;            // write n-1 value

        uint8_t i;
        for (i = 3; i < length+3; i++) {
		    tx_buffer[i] = vals[i-3];		// write values
        }

		tx_buffer[i] = RDATAC_COMMAND;      // start RDATAC

		spi_tx_bufs[0].buf = tx_buffer;
		spi_tx_bufs[0].len = i+1;           // sizeof(vals)+4
    } else {
		tx_buffer[0] = WREG_COMMAND | add;  // write address
		tx_buffer[1] = length-1;            // write n-1 value

        uint8_t i;
        for (i = 2; i < length+2; i++) {
		    tx_buffer[i] = vals[i-2];		// write values
        }

		spi_tx_bufs[0].buf = tx_buffer;
		spi_tx_bufs[0].len = i;             // sizeof(vals)+2
    }

    spi_tx_buf_set.buffers = spi_tx_bufs;
    spi_tx_buf_set.count = 1;

    //gpio_pin_set_dt(ads_device->cs_spec, 1);
    int err = spi_write(ads_device->spi_dev, &ads_device->spi_cfg, &spi_tx_buf_set);
    //gpio_pin_set_dt(ads_device->cs_spec, 0);
    
    return err;
};

int read_register(const struct ADS1298 *ads_device, uint8_t add) {
    uint8_t tx_buffer[3];
	struct spi_buf spi_tx_bufs[1];
	struct spi_buf_set spi_tx_buf_set;
	
	uint8_t rx_buffer[1];
	struct spi_buf spi_rx_bufs[1];
	struct spi_buf_set spi_rx_buf_set;

    if (ads_device->rdatac_mode) {
        tx_buffer[0] = SDATAC_COMMAND;      // stop RDATAC
		tx_buffer[1] = RREG_COMMAND | add;  // read address
        tx_buffer[2] = 0x00;	            // read n-1

		spi_tx_bufs[0].buf = tx_buffer;
		spi_tx_bufs[0].len = 3;

        spi_tx_buf_set.buffers = spi_tx_bufs;
        spi_tx_buf_set.count = 1;

        //gpio_pin_set_dt(ads_device->cs_spec, 1);
        int err = spi_write(ads_device->spi_dev, &ads_device->spi_cfg, &spi_tx_buf_set);

        if (err == 0) {
            spi_rx_bufs[0].buf = rx_buffer;
            spi_rx_bufs[0].len = 1;

            spi_rx_buf_set.buffers = spi_rx_bufs;
            spi_rx_buf_set.count = 1;

            err = spi_read(ads_device->spi_dev, &ads_device->spi_cfg, &spi_rx_buf_set);
        }

        if (err == 0) {
            tx_buffer[0] = RDATAC_COMMAND;      // start RDATAC

            spi_tx_bufs[0].buf = tx_buffer;
            spi_tx_bufs[0].len = 1;

            spi_tx_buf_set.buffers = spi_tx_bufs;
            spi_tx_buf_set.count = 1;

            err = spi_write(ads_device->spi_dev, &ads_device->spi_cfg, &spi_tx_buf_set);
        }

        if (err == 0) {
            return rx_buffer[0];
        } else {
            return err;
        }
    } else {
		tx_buffer[0] = RREG_COMMAND | add;  // read address
        tx_buffer[1] = 0x00;	            // read n-1

		spi_tx_bufs[0].buf = tx_buffer;
		spi_tx_bufs[0].len = 2;

        spi_tx_buf_set.buffers = spi_tx_bufs;
        spi_tx_buf_set.count = 1;

        int err = spi_write(ads_device->spi_dev, &ads_device->spi_cfg, &spi_tx_buf_set);

        if (err == 0) {
            spi_rx_bufs[0].buf = rx_buffer;
            spi_rx_bufs[0].len = 1;

            spi_rx_buf_set.buffers = spi_rx_bufs;
            spi_rx_buf_set.count = 1;

            err = spi_read(ads_device->spi_dev, &ads_device->spi_cfg, &spi_rx_buf_set);
        }

        if (err == 0) {
            return rx_buffer[0];
        } else {
            return err;
        }
    }
}

int read_ads_data(struct ADS1298 *ads_device) {
	int err;

    ads_device->data_rx_bufs[0].buf = ads_device->data_buffer;
    ads_device->data_rx_bufs[0].len = 27;
    ads_device->data_rx_buf_set.buffers = ads_device->data_rx_bufs;
    ads_device->data_rx_buf_set.count = 1;

    if (!ads_device->conversion_status) {
        err = set_ads_conversion(ads_device, true);         // start ADC conversion if needed
        if (err < 0) return err;
    }

    if (ads_device->continous_reading) {
        err = stop_ads_reading(ads_device);                 // stop continous data reading if active
        if (err < 0) return err;
    }

    if (!ads_device->rdatac_mode) {
        send_ads_command(ads_device, RDATA_COMMAND);        // send RDATA command in SDATAC mode
    }

    while (gpio_pin_get_dt(ads_device->drdy_spec) == 0) {}  // wait until DRDY goes low (active)
    
    err = _get_ads_data(ads_device);                        // read 27 bytes of data into the buffer

    return err;
}

int start_ads_reading(struct ADS1298 *ads_device, gpio_callback_handler_t callback_fn) {
    int err;

    if (ads_device->continous_reading) {
        err = stop_ads_reading(ads_device);                 // stop ongoing continous reading
        if (err < 0) return err;
    }

    if (!ads_device->rdatac_mode) {
        err = set_ads_read_mode(ads_device, true);          // set RDATAC mode
        if (err < 0) return err;
    }

    if (!ads_device->conversion_status) {
        err = set_ads_conversion(ads_device, true);         // start ADC conversion if needed
        if (err < 0) return err;
    }

    err = gpio_pin_interrupt_configure(ads_device->drdy_spec->port, ads_device->drdy_spec->pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) return err;

    gpio_init_callback(&ads_device->drdy_cb_data, callback_fn, BIT(ads_device->drdy_spec->pin));
	err = gpio_add_callback(ads_device->drdy_spec->port, &ads_device->drdy_cb_data);
    if (err < 0) return err;

    if (err == 0) {
        ads_device->continous_reading = true;
    }

    return err;
}

int stop_ads_reading(struct ADS1298 *ads_device) {
    int err = 0;

    if (ads_device->continous_reading) {
        err = gpio_remove_callback(ads_device->drdy_spec->port, &ads_device->drdy_cb_data);
        if (err < 0) return err;

        err = gpio_pin_interrupt_configure(ads_device->drdy_spec->port, ads_device->drdy_spec->pin, GPIO_INT_DISABLE);
        if (err < 0) return err;

        err = set_ads_conversion(ads_device, false);

        ads_device->continous_reading = false;
    }

    return err;
}

int set_ads_conversion(struct ADS1298 *ads_device, bool conversion_status) {
    uint8_t cmd;

    if (conversion_status) {
        cmd = START_COMMAND;
    } else {
        cmd = STOP_COMMAND;
    }

    int err = send_ads_command(ads_device, cmd);

    if (err == 0) {
        ads_device->conversion_status = conversion_status;

        if (conversion_status) {
            int32_t tsettle;

            if (ads_device->high_resolution_mode) {                         // t_settle time is based on the datasheet
                tsettle = (float)(288*(ads_device->datarate+1)+10)*TCLK; 
            } else {
                tsettle = (float)(576*(ads_device->datarate+1)+10)*TCLK;
            }
            
            k_usleep(tsettle);
        }
    }

    return err;
};

int _get_ads_data(struct ADS1298 *ads_device) {
    ads_device->data_rx_bufs[0].buf = ads_device->data_buffer;
    ads_device->data_rx_bufs[0].len = 27;
    ads_device->data_rx_buf_set.buffers = ads_device->data_rx_bufs;
    ads_device->data_rx_buf_set.count = 1;

    int err = spi_read(ads_device->spi_dev, &ads_device->spi_cfg, &ads_device->data_rx_buf_set);

    return err;
};

int _get_ads_data_async(struct ADS1298 *ads_device) {
    ads_device->data_rx_bufs[0].buf = ads_device->data_buffer;
    ads_device->data_rx_bufs[0].len = 27;
    ads_device->data_rx_buf_set.buffers = ads_device->data_rx_bufs;
    ads_device->data_rx_buf_set.count = 1;

    int err = spi_transceive_signal(ads_device->spi_dev, &ads_device->spi_cfg, NULL, &ads_device->data_rx_buf_set, &ads_device->signal);

    return err;
};