#include <ADS1298.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/flash.h>
#include <arm_math.h>

/* Number of samples in a window. */
#define WINDOW_SIZE 128

/* Number of samples from the previous window in a window. */
#define WINDOW_OVERLAP 25

/* Number of first windows left unprocessed. */
#define DROP_WINDOWS 10

/* Number of windows used for threshold calculation. */
#define CALIBRARION_WINDOWS 10

/* Number of IIR filter stages. */
#define FILTER_STAGES 8

/** Number of the last segments of which activation status is stored.
 *  Must be a divisor of 256 for circular indexing! (1, 2, 4, 8, 16, 32, 64, 128 or 256)
 */
#define SEGMENT_HISTORY_LENGTH 8
 
/** Length of vector containing a segment's feature values.
 *  Features: RMS (1/ch), SD (1/ch), AbsMax (1/ch)
 */
#define FEATURE_VECTOR_SIZE (1+1+1)*6

/** Number of classes used for classification.
 *  1st: wrist flexion, 2nd: wrist extension, 3rd: finger tap, 4th: finger flexion, 5th: finger extension
 */
#define NO_CLASSES 5

/** Number of preceeding segments of which prediction probabilities is stored. 
 *  PREDICTION_HISTORY_LENGTH*NO_CLASSES must be a divisor of 256 for circular indexing! (1, 2, 4, 8, 16, 32, 64, 128 or 256)
 */
#define PREDICTION_HISTORY_LENGTH 32 

/* Stacksize used by receive_samples_work_q. */
#define RECEIVESAMPLES_WORK_Q_STACKSIZE 2048

/* Work que priotity of receive_samples_work_q. */
#define RECEIVESAMPLES_WORK_Q_PRIORITY -1

/* Stacksize used by handle_new_samples_work_q. */
#define NEWSAMPLES_WORK_Q_STACKSIZE 2048

/* Work que priotity of handle_new_samples_work_q. */
#define NEWSAMPLES_WORK_Q_PRIORITY 1

/* Buffer of new samples containing the 6 channels' converted values. */
static float sample_readings_buffer[6];

/** Buffer of float32 type samples of channel 1.
 *  Used to read the new converted samples into.
 */
static float sample_ch1_buffer[256];

/** Buffer of float32 type samples of channel 2.
 *  Used to read the new converted samples into.
 */
static float sample_ch2_buffer[256];

/** Buffer of float32 type samples of channel 3.
 *  Used to read the new converted samples into.
 */
static float sample_ch3_buffer[256];

/** Buffer of float32 type samples of channel 4.
 *  Used to read the new converted samples into.
 */
static float sample_ch4_buffer[256];

/** Buffer of float32 type samples of channel 5.
 *  Used to read the new converted samples into.
 */
static float sample_ch5_buffer[256];

/** Buffer of float32 type samples of channel 6.
 *  Used to read the new converted samples into.
 */
static float sample_ch6_buffer[256];

/** Buffer of float32 type samples of channel 1 of the current segment.
 *  Used to store the samples during processing.
 */
static float32_t segment_ch1_buffer[WINDOW_SIZE];

/** Buffer of float32 type samples of channel 2 of the current segment.
 *  Used to store the samples during processing.
 */
static float32_t segment_ch2_buffer[WINDOW_SIZE];

/** Buffer of float32 type samples of channel 3 of the current segment.
 *  Used to store the samples during processing.
 */
static float32_t segment_ch3_buffer[WINDOW_SIZE];

/** Buffer of float32 type samples of channel 4 of the current segment.
 *  Used to store the samples during processing.
 */
static float32_t segment_ch4_buffer[WINDOW_SIZE];

/** Buffer of float32 type samples of channel 5 of the current segment.
 *  Used to store the samples during processing.
 */
static float32_t segment_ch5_buffer[WINDOW_SIZE];

/** Buffer of float32 type samples of channel 6 of the current segment.
 *  Used to store the samples during processing.
 */
static float32_t segment_ch6_buffer[WINDOW_SIZE];

/* Intermediate output buffer used by IIR filter on samples of channel 1. */
static float32_t filter_ch1_buffer[WINDOW_SIZE-WINDOW_OVERLAP];

/* Intermediate output buffer used by IIR filter on samples of channel 2. */
static float32_t filter_ch2_buffer[WINDOW_SIZE-WINDOW_OVERLAP];

/* Intermediate output buffer used by IIR filter on samples of channel 3. */
static float32_t filter_ch3_buffer[WINDOW_SIZE-WINDOW_OVERLAP];

/* Intermediate output buffer used by IIR filter on samples of channel 4. */
static float32_t filter_ch4_buffer[WINDOW_SIZE-WINDOW_OVERLAP];

/* Intermediate output buffer used by IIR filter on samples of channel 5. */
static float32_t filter_ch5_buffer[WINDOW_SIZE-WINDOW_OVERLAP];

/* Intermediate output buffer used by IIR filter on samples of channel 6. */
static float32_t filter_ch6_buffer[WINDOW_SIZE-WINDOW_OVERLAP];

/** Activation states of channels. bit[0] -> channel 1; bit[1] -> channel 2 ... bit[5] -> channel 6
 *  Last segment's activation state is accessed by index [channel_activation_states_offset%SEGMENT_HISTORY_LENGTH].
 */
static uint8_t channel_activation_sates[SEGMENT_HISTORY_LENGTH];

/* Array conatining the actual active segment feature values. */
static float32_t segment_feature_vector[FEATURE_VECTOR_SIZE];

/** RMS values of the 6 channel of the actual segment/ window.
 *  Used for segmentation (channel activation feature) and feature extraction.
 */
static float32_t segment_rms_values[6];

/** Binary outputs of the predictions of the last PREDICTION_HISTORY_LENGTH segments. All probabilities are 0 if the segment is inactive.
 *  Each class's output of the last active segment is accessed by index [(NO_CLASSES*(prediction_probabilities_offset%PREDICTION_HISTORY_LENGTH))+class_id].
 */
static bool prediction_outputs[NO_CLASSES*PREDICTION_HISTORY_LENGTH];

/** Binary outputs of the prediction of the last active segment. All outputs are 0 if the segment is inactive.
 *  Each class's output is accessed by index [class_id].
 */
static bool segment_predictions[NO_CLASSES];

/** Conversion unit used by sample conversion. Vref/(2^23-1)
 *  Results in [uV].
 */
#define CONVERSION_UNIT 0.286102329027930f

/* Device structure pointer of the on-board Flash device. */
static const struct device *flash_device;


/** Converts multichannel samples of 24 bit 2's complement to float32 format.
 *  @param input_buf: pointer of the sample buffer (ch_num*3 bytes)
 *  @param ch_num: number of channels of which samples are to converted
 *  @param output_buf: pointer of the ouput buffer (ch_num*1 float32)
 */
void convert_samples_to_float(const uint8_t *input_buf, uint8_t ch_num, float *output_buf);

/** Processes the new samples in the sample buffer.
 *  Work item is passed by receive_samples;
 */
void handle_new_samples(struct k_work *item);

/** Filters the last (WINDOW_SIZE-WINDOW_OVERLAP) number of samples on both channels.
 *  The filtered samples are written back to the corresponding sample_chx_buffers.
 */
void filter_samples();

/** Checks if the sample values in the last window statisfies the activation conditions.
 *  Calculates the absolute maximum values of the window and operates on the channel_activation_sates array.
 *  @return true, if the last window is active, false otherwise
 */
bool segment_samples();

/** Extracts the RMS and SD values of both the active segment. Also uses the segment_abs_max_values.
 *  The values are written into the segment_feature_vector.
 */
void feature_extract_segment();

/** Executes SVM prediction on the segment_feature_vector.
 *  The predticion class outputs are written into the prediction_outputs array.
 */
void predict_segment();

/** Initializizes and configures the Flash device. 
 *  @returns negative error code in case of failure
 */
int flash_init();

/* Enables the writing process of the segment data into the Flash. */
void enable_write_segment_to_flash(const struct device *flash_dev);

/** Writes the last complete segment data into flash. 
 *  2 bytes of uint16_t incremental segment ID
 *  WINDOW_SIZE lenght of channel 1 float32_t filtered samples
 *  WINDOW_SIZE lenght of channel 2 float32_t filtered samples
 *  FEATURE_VECTOR_SIZE length of float32_t feature vector
 *  NO_CLASSES length of float32_t class probabilities
 */
void write_segment_to_flash();

/** Converts samples from the ADS device's buffer and moves to the sample buffer.
 *  Polls the signal emitted by _get_ads_data_async of ADS.
 */
void receive_samples();

/** Initializizes and configures the ADS1298 device. 
 *  @returns negative error code in case of failure
 */
int adc_init();

/** Callback function called when new data is available on ADS device.
 *  Rreads the new samples by calling _get_ads_data_async and submits new work to receive and process the new samples.
 */
void adc_read_data_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/** Getter function of recording_in_progress.
 *  @return true, if recording and processing are in progress
 */
bool get_recording_state();

/* Starts signal recording and processing. */
void start_recording();

/* Stops signal recording and processing. */
void stop_recording();

/** Initializes the devices needed for signal processing, typically the ADS1298 ADC and the on-board Flash.
 *  Once the required devices are successfully configured, it initiates the processing_tasks thread.
 */
void processing_init();

/* Function executed by the system_tasks thread. 
   Initiates the system peripherials and starts the timers to perform system supervision tasks.*/
void processing_tasks();

void send_UART_data(char *buf , uint8_t len);

void tx_start();
void tx_done();
