#include <processing.h>
#include <system_tasks.h>
#include <arm_math.h>

static const struct device *spi4_dev = DEVICE_DT_GET(DT_NODELABEL(spi4));

const struct gpio_dt_spec adc_pwdn_spec = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
	.pin = 8,
	.dt_flags = GPIO_ACTIVE_LOW | GPIO_PULL_UP
};

const struct gpio_dt_spec adc_reset_spec = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
	.pin = 9,
	.dt_flags = GPIO_ACTIVE_LOW | GPIO_PULL_UP
};

const struct gpio_dt_spec drdy_spec = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
	.pin = 11,
	.dt_flags = GPIO_ACTIVE_LOW
};

struct spi_config spi_cfg = {
	.frequency = 6000000U,
	.operation = SPI_WORD_SET(8) | SPI_MODE_CPHA,
	.slave = 0,
	.cs = {.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(spi4_reg)), .delay = 0}
};


const struct gpio_dt_spec osc_out_spec = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 25,
	.dt_flags = GPIO_ACTIVE_HIGH
};

/* Number of last new samples read into the buffer waiting for processing. */
uint8_t unprocessed_samples = 0;

/** Offset value of sample_chx_buffer used for circular indexing.
 *  Index of the last samples written into the buffers. (Index of the place for the first new sample.)
 */
uint8_t sample_buffers_write_offset = 0;

/** Offset value of sample_chx_buffer used for circular indexing.
 *  Index of the last samples read from the buffers. (Index of the previous segment's first sample.)
 *  Starting with window overlap because only WINDOW_SIZE-WINDOW_OVERLAP number of new samples has to be filtered.
 */
uint8_t sample_buffers_read_offset = 0;

/* Activation threshold calibration is done. */
bool threshold_calibrated = false;

/* Filter noise is ok. */
bool noise_passed = false;

/* Sum of means of channel 1 during calibration process. */
float32_t ch1_calibration_sum = 0;

/* Sum of means of channel 2 during calibration process. */
float32_t ch2_calibration_sum = 0;

/* Sum of means of channel 3 during calibration process. */
float32_t ch3_calibration_sum = 0;

/* Sum of means of channel 4 during calibration process. */
float32_t ch4_calibration_sum = 0;

/* Sum of means of channel 5 during calibration process. */
float32_t ch5_calibration_sum = 0;

/* Sum of means of channel 6 during calibration process. */
float32_t ch6_calibration_sum = 0;

/* Number of means summed during calibration process. */
uint16_t calibration_windows = 0;


/* IIR lattice filter state buffer for channel 1. */
static float32_t ch1_iir_state[WINDOW_SIZE-WINDOW_OVERLAP+FILTER_STAGES];

/* IIR lattice filter state buffer for channel 2. */
static float32_t ch2_iir_state[WINDOW_SIZE-WINDOW_OVERLAP+FILTER_STAGES];

/* IIR lattice filter state buffer for channel 3. */
static float32_t ch3_iir_state[WINDOW_SIZE-WINDOW_OVERLAP+FILTER_STAGES];

/* IIR lattice filter state buffer for channel 4. */
static float32_t ch4_iir_state[WINDOW_SIZE-WINDOW_OVERLAP+FILTER_STAGES];

/* IIR lattice filter state buffer for channel 5. */
static float32_t ch5_iir_state[WINDOW_SIZE-WINDOW_OVERLAP+FILTER_STAGES];

/* IIR lattice filter state buffer for channel 6. */
static float32_t ch6_iir_state[WINDOW_SIZE-WINDOW_OVERLAP+FILTER_STAGES];

/** IIR lattice filter k (reflection) coefficients buffer. */
const float32_t iir_k_coeffs[FILTER_STAGES] = {0.19079f, 0.25747f, 0.032504f, -0.91891f, 0.98002f, -0.97315f, 0.99696f, -0.95731f};

/** IIR lattice filter v (ladder) coefficients buffer. */
const float32_t iir_v_coeffs[FILTER_STAGES+1] = {0.42276f, 0.38152f, -0.36963f, -0.29507f, -0.011324f, 0.0031678f, -0.0011917f, -0.00014486f, -2.1694e-05f};

/* IIR lattice filter instance for signal filtration on channel 1. */
arm_iir_lattice_instance_f32 ch1_iir_instance;

/* IIR lattice filter instance for signal filtration on channel 2. */
arm_iir_lattice_instance_f32 ch2_iir_instance;

/* IIR lattice filter instance for signal filtration on channel 3. */
arm_iir_lattice_instance_f32 ch3_iir_instance;

/* IIR lattice filter instance for signal filtration on channel 4. */
arm_iir_lattice_instance_f32 ch4_iir_instance;

/* IIR lattice filter instance for signal filtration on channel 5. */
arm_iir_lattice_instance_f32 ch5_iir_instance;

/* IIR lattice filter instance for signal filtration on channel 6. */
arm_iir_lattice_instance_f32 ch6_iir_instance;

/* Threshold values above the channel activation feature values result in segment considered active. */
float32_t channel_activation_threshold[6] = {72.0f, 34.0f, 42.0f, 19.0f, 33.0f, 28.0f};

/** Offset value of channel_activation_states array. Index of the actual segment's state.
 *  Use with modulo SEGMENT_HISTORY_LENGTH in order to index in a circular manner.
 */
uint8_t channel_activation_states_offset = SEGMENT_HISTORY_LENGTH-1;

/** Offset value of prediction_outputs array. Index of the actual active segment's class probabilities.
 *  Use with modulo PREDICTION_HISTORY_LENGTH in order to index in a circular manner. Increase it by 1.
 */
uint8_t prediction_outputs_offset = PREDICTION_HISTORY_LENGTH-1;

/* Last 5 predicted classes. */
static uint16_t last_predictions[5] = {0, 0, 0, 0, 0};

/* Current output class. */
static uint16_t output_class = 0;

/* ADS device stucture. Used to configure the device and read samples from it. */
struct ADS1298 ads_dev;

/* Work que for receiving new samples. */
struct k_work_q receive_samples_work_q;

/* Work element for receiving new samples. */
struct k_work receive_samples_work;

/* Work que for processing new samples. */
struct k_work_q handle_new_samples_work_q;

/* Work element for processing new samples. */
struct k_work handle_new_samples_work;

/* Poll events stucture for signaling new samples ready for reading. */
struct k_poll_event new_samples_to_read_events[1] = {K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &ads_dev.signal, 0)};

/* Stack definition used by receive_samples_work_q. */
K_THREAD_STACK_DEFINE(receive_samples_work_q_stack_area, RECEIVESAMPLES_WORK_Q_STACKSIZE);

/* Stack definition used by handle_new_samples_work_q. */
K_THREAD_STACK_DEFINE(handle_new_samples_work_q_stack_area, NEWSAMPLES_WORK_Q_STACKSIZE);

/** Incremental ID for segment identification.
 *  Used by write_segment_to_flash funtion.
 */
uint16_t segment_ID = 0;

/* Flash offset value used by write_segment_to_flash funtion. */
off_t flash_offset = 0;

/* Switch if the segment data is to be written into the Flash. */
bool segments_to_flash = false;

/* Status of the recording and processing. */
bool recording_in_progress = false;


/* Trained model parameters of wrist flexion vs all binary classifier. */
arm_svm_polynomial_instance_f32 wrist_flexion_svm;
uint32_t wrist_flexion_nbSupportVectors = 11;
uint32_t wrist_flexion_degree = 2;
float32_t wrist_flexion_coef0 = 0.100000f;
float32_t wrist_flexion_gamma = 0.055556f;
float32_t wrist_flexion_intercept = -1.054216f;
int32_t wrist_flexion_classes[] = {0, 1};
float32_t wrist_flexion_dualCoefs[] = {-9.207964955256005e-07f, -5.179342507049141e-08f, -3.210040507799258e-07f, -2.7577847422517496e-08f, -5.751853382985106e-07f, -5.418894932327786e-08f, 1.3871743553836105e-06f, 1.7298185671195015e-08f, 4.1933480814559174e-07f, 1.0279979752838455e-07f, 2.393895969154132e-08f};
float32_t wrist_flexion_supportVectors[] = {28.792036f, 20.976504f, 51.024899f, 66.91614f, 29.938461f, 29.547399f, 28.904411f, 21.051117f, 51.225337f, 67.179047f, 30.055618f, 29.660737f, 80.838434f, 80.342601f, 153.136103f, 205.021666f, 116.905769f, 69.27461f, 32.725555f, 30.657646f, 75.765236f, 83.048714f, 25.316579f, 26.631344f, 32.844159f, 30.776486f, 76.059934f, 83.371177f, 25.400305f, 26.712563f, 126.240081f, 107.968571f, 251.470479f, 333.693886f, 72.563289f, 64.432743f, 41.992887f, 29.21878f, 77.086382f, 105.029333f, 28.586664f, 30.04198f, 42.144344f, 29.313461f, 77.255318f, 105.441313f, 28.692648f, 30.1486f, 106.955259f, 83.468761f, 220.72319f, 321.203273f, 96.334715f, 74.672218f, 33.812587f, 23.037153f, 79.953438f, 62.593876f, 21.892009f, 25.054736f, 33.903414f, 23.11692f, 80.248298f, 62.792143f, 21.972903f, 25.107952f, 96.043695f, 86.307491f, 336.001041f, 222.402752f, 62.247858f, 64.431506f, 9.087964f, 5.313291f, 6.94278f, 24.379374f, 7.352226f, 4.670135f, 9.108583f, 5.322975f, 6.956122f, 24.469102f, 7.379905f, 4.684171f, 25.313371f, 16.895783f, 18.718258f, 123.669778f, 30.707096f, 15.497593f, 85.815015f, 53.618943f, 31.242502f, 26.90959f, 84.780005f, 16.183415f, 86.140545f, 53.82741f, 31.346914f, 27.00357f, 85.068798f, 16.218048f, 256.838512f, 165.491151f, 131.363607f, 98.276023f, 348.336027f, 42.053764f, 15.115102f, 9.720967f, 40.669075f, 44.036704f, 47.917977f, 13.165142f, 15.173484f, 9.759145f, 40.825121f, 44.208497f, 48.105528f, 13.216625f, 37.377165f, 26.922215f, 110.635145f, 145.627942f, 133.184769f, 42.202784f, 30.027523f, 24.726321f, 79.28102f, 42.74917f, 41.489126f, 8.114174f, 30.14335f, 24.819525f, 79.589049f, 42.913129f, 41.652076f, 8.144361f, 100.49865f, 99.823654f, 233.28795f, 120.161466f, 117.891934f, 27.209541f, 31.689163f, 45.470518f, 81.563237f, 89.904556f, 35.514948f, 15.72814f, 31.80844f, 45.526147f, 81.874871f, 90.232698f, 35.64761f, 15.789192f, 124.593207f, 112.923139f, 215.951838f, 265.670379f, 102.348655f, 40.001103f, 49.472415f, 57.887808f, 73.49758f, 137.513053f, 22.91266f, 13.476096f, 49.636471f, 58.100567f, 73.731353f, 138.039464f, 22.983339f, 13.527688f, 154.814139f, 180.656057f, 247.94963f, 523.83324f, 62.756344f, 38.527201f, 24.633304f, 35.952577f, 68.880111f, 100.900411f, 25.6246f, 13.594027f, 24.724567f, 36.091333f, 69.031395f, 101.28965f, 25.704923f, 13.64042f, 68.807263f, 152.209543f, 251.129861f, 292.72544f, 64.526834f, 36.772896f};

/* Trained model parameters of wrist extension vs all binary classifier. */
arm_svm_polynomial_instance_f32 wrist_extension_svm;
uint32_t wrist_extension_nbSupportVectors = 15;
uint32_t wrist_extension_degree = 2;
float32_t wrist_extension_coef0 = 0.100000f;
float32_t wrist_extension_gamma = 0.055556f;
float32_t wrist_extension_intercept = -2.712931f;
int32_t wrist_extension_classes[] = {0, 2};
float32_t wrist_extension_dualCoefs[] = {-1.1848910344355149e-06f, -5.94878016029533e-06f, -2.0597765293634255e-06f, -6.196971044985442e-06f, -1.0876727289986015e-09f, -3.8568506879245577e-07f, -3.0065983440380924e-06f, -7.065537671355493e-07f, -1.0497500314636233e-05f, -1.5276560849901836e-06f, 1.7870259836335918e-06f, 9.416458439585063e-07f, 1.242069967631477e-06f, 2.6787391875547924e-05f, 7.573663506296492e-07f};
float32_t wrist_extension_supportVectors[] = {59.76826f, 65.565327f, 10.559885f, 10.680576f, 11.376868f, 37.442835f, 59.997482f, 65.760332f, 10.599952f, 10.722543f, 11.409301f, 37.561672f, 220.985965f, 157.469772f, 29.846827f, 35.553161f, 29.011206f, 103.013025f, 72.847752f, 65.257725f, 12.844306f, 10.046274f, 13.747795f, 47.958176f, 73.097967f, 65.514059f, 12.89423f, 10.065049f, 13.801659f, 48.141394f, 194.886378f, 207.050389f, 36.008685f, 32.266982f, 35.312445f, 137.502745f, 98.444644f, 128.133178f, 15.105736f, 14.293339f, 6.66965f, 16.773896f, 98.72131f, 128.559421f, 15.162621f, 14.349238f, 6.688286f, 16.811955f, 335.070658f, 385.144653f, 50.007534f, 60.542063f, 24.809215f, 58.659709f, 61.74361f, 86.439463f, 11.987389f, 12.735057f, 16.699576f, 59.465686f, 61.984982f, 86.771153f, 12.025266f, 12.783916f, 16.764849f, 59.698176f, 242.043729f, 280.36621f, 37.571229f, 41.686866f, 46.123424f, 223.270768f, 21.056446f, 29.448239f, 155.817611f, 789.283604f, 179.821502f, 21.562037f, 21.128909f, 29.561999f, 156.429484f, 792.382977f, 180.506208f, 21.644613f, 59.832232f, 82.12676f, 417.375291f, 2860.706361f, 485.279301f, 58.815075f, 159.519449f, 65.026922f, 50.724709f, 27.953133f, 7.496383f, 23.225974f, 160.07538f, 65.282388f, 50.906806f, 28.036686f, 7.525821f, 23.309339f, 419.069108f, 157.695434f, 180.559712f, 81.49664f, 17.347682f, 69.798133f, 52.749092f, 82.44354f, 12.076853f, 11.99871f, 10.897298f, 38.903463f, 52.929058f, 82.761503f, 12.122313f, 12.03526f, 10.917638f, 38.983704f, 141.960771f, 383.429377f, 49.866546f, 36.865795f, 25.361568f, 97.975848f, 90.44886f, 77.666023f, 14.724718f, 10.097804f, 17.916335f, 45.70375f, 90.546274f, 77.960172f, 14.782177f, 10.135306f, 17.985524f, 45.87896f, 322.843905f, 276.973603f, 37.588738f, 31.404486f, 43.420509f, 132.632411f, 65.190973f, 70.68902f, 11.599418f, 9.272503f, 13.943146f, 31.067721f, 65.433454f, 70.966626f, 11.643791f, 9.308933f, 13.997868f, 31.169525f, 177.819036f, 184.887545f, 36.062065f, 23.995053f, 46.292961f, 128.285912f, 68.006304f, 68.69504f, 14.924957f, 10.98479f, 17.150553f, 46.283363f, 68.211112f, 68.889475f, 14.972545f, 11.018167f, 17.204392f, 46.464787f, 258.950104f, 212.741208f, 39.966709f, 31.191944f, 41.730132f, 172.829331f, 147.490807f, 164.357486f, 39.415111f, 43.162941f, 18.664597f, 90.450234f, 147.903315f, 164.994573f, 39.569168f, 43.298287f, 18.733628f, 90.736453f, 476.461956f, 411.041735f, 100.761438f, 160.379375f, 53.481394f, 204.839621f, 113.111023f, 201.285376f, 34.536429f, 13.148347f, 52.398394f, 61.246903f, 113.534853f, 202.076283f, 34.671609f, 13.199263f, 52.603367f, 61.483183f, 423.011932f, 754.762843f, 96.715594f, 42.188281f, 188.770341f, 252.080438f, 58.649737f, 75.148268f, 10.610576f, 4.095063f, 6.171106f, 33.957154f, 58.773205f, 75.434244f, 10.645133f, 4.110773f, 6.190973f, 34.060883f, 173.242293f, 251.045959f, 30.404101f, 11.407114f, 17.182334f, 78.508383f, 59.902668f, 60.846683f, 8.991529f, 4.466071f, 6.772376f, 34.627364f, 60.136908f, 61.085541f, 9.02667f, 4.483317f, 6.788603f, 34.707809f, 178.357828f, 170.502522f, 30.404101f, 14.360149f, 22.056674f, 133.206142f, 75.633793f, 115.028216f, 12.465162f, 18.598326f, 12.406601f, 56.824906f, 75.930892f, 115.476454f, 12.511679f, 18.671397f, 12.454434f, 57.028893f, 239.250898f, 650.717238f, 39.854785f, 81.91198f, 48.753335f, 248.427033f};

/* Trained model parameters of finger tap vs all binary classifier. */
arm_svm_polynomial_instance_f32 finger_tap_svm;
uint32_t finger_tap_nbSupportVectors = 26;
uint32_t finger_tap_degree = 2;
float32_t finger_tap_coef0 = 0.100000f;
float32_t finger_tap_gamma = 0.055556f;
float32_t finger_tap_intercept = -0.944816f;
int32_t finger_tap_classes[] = {3, 0};
float32_t finger_tap_dualCoefs[] = {-1.8216019247716276e-06f, -0.0001395302479826883f, -2.3708576897130846e-05f, -2.229125892745569e-06f, -2.3604320167491538e-05f, -3.5929274278735785e-06f, -1.1996792724810436e-05f, -1.1597860803219138e-06f, -3.109592120773486e-05f, -2.2806719772789855e-06f, -2.4053721251618724e-05f, -3.6191812690517516e-05f, 0.00022831246691969325f, 9.78093269693008e-06f, 6.216659998573109e-06f, 6.767719516457412e-06f, 5.408280393429208e-06f, 1.0733924460659545e-06f, 6.200934861183189e-07f, 5.9968840219033e-07f, 3.931294723030395e-06f, 9.938291810704136e-06f, 2.224386782934227e-05f, 2.950926501830668e-06f, 1.6156765522520663e-06f, 1.8062149483681686e-06f};
float32_t finger_tap_supportVectors[] = {65.618712f, 48.097154f, 8.120183f, 12.288758f, 8.07196f, 24.566337f, 65.815665f, 48.243459f, 8.149887f, 12.325121f, 8.093847f, 24.661729f, 172.505805f, 131.938441f, 20.406139f, 42.508668f, 18.097798f, 68.971149f, 52.364776f, 64.066489f, 12.508657f, 8.571482f, 10.741198f, 29.615573f, 52.521543f, 64.316175f, 12.557569f, 8.593664f, 10.781426f, 29.73178f, 141.555562f, 175.526905f, 41.442353f, 27.401334f, 29.356606f, 84.848337f, 64.743782f, 56.471401f, 10.480027f, 11.241736f, 9.814154f, 28.106119f, 64.992334f, 56.690377f, 10.52058f, 11.283458f, 9.847831f, 28.216534f, 241.359373f, 187.167023f, 39.86891f, 30.280428f, 24.032823f, 95.657557f, 57.480544f, 67.072543f, 11.954146f, 11.802723f, 13.29143f, 28.37221f, 57.697956f, 67.321243f, 12.000048f, 11.846698f, 13.340879f, 28.393655f, 241.359373f, 287.37935f, 42.073374f, 32.837958f, 45.658029f, 97.444753f, 32.760024f, 31.86125f, 7.580368f, 13.729942f, 30.217043f, 23.15392f, 32.882949f, 31.902974f, 7.604373f, 13.775004f, 30.331246f, 23.183003f, 111.071746f, 79.299953f, 20.614544f, 56.263107f, 125.72942f, 59.850191f, 69.141117f, 58.697024f, 13.62225f, 71.815167f, 15.831383f, 33.493857f, 69.376222f, 58.926812f, 13.671745f, 72.092913f, 15.885385f, 33.600985f, 201.002124f, 171.472401f, 44.872355f, 412.443849f, 36.048881f, 103.514103f, 60.929471f, 81.60422f, 15.542929f, 15.001009f, 14.659621f, 69.520545f, 61.094738f, 81.910865f, 15.600616f, 15.047368f, 14.695645f, 69.78353f, 190.332041f, 309.022206f, 49.982711f, 40.414897f, 38.707468f, 283.661634f, 52.749092f, 82.44354f, 12.076853f, 11.99871f, 10.897298f, 38.903463f, 52.929058f, 82.761503f, 12.122313f, 12.03526f, 10.917638f, 38.983704f, 141.960771f, 383.429377f, 49.866546f, 36.865795f, 25.361568f, 97.975848f, 43.363595f, 84.006376f, 12.069192f, 11.61109f, 11.427072f, 27.117961f, 43.511527f, 84.314762f, 12.116616f, 11.636172f, 11.471477f, 27.224453f, 113.654744f, 345.493758f, 31.255414f, 43.95798f, 39.420734f, 75.755885f, 90.44886f, 77.666023f, 14.724718f, 10.097804f, 17.916335f, 45.70375f, 90.546274f, 77.960172f, 14.782177f, 10.135306f, 17.985524f, 45.87896f, 322.843905f, 276.973603f, 37.588738f, 31.404486f, 43.420509f, 132.632411f, 48.429501f, 76.980502f, 10.191898f, 10.49212f, 10.788096f, 41.122804f, 48.61639f, 77.268533f, 10.219335f, 10.530996f, 10.825698f, 41.196324f, 152.083213f, 292.795771f, 34.912658f, 31.09532f, 37.804283f, 113.468109f, 39.420029f, 49.298382f, 14.976678f, 25.664793f, 12.179072f, 22.311679f, 39.574594f, 49.484406f, 15.031869f, 25.755131f, 12.224593f, 22.385935f, 142.589384f, 152.274055f, 61.656468f, 105.548474f, 39.581777f, 82.32127f, 6.783081f, 5.012163f, 4.886774f, 13.793773f, 8.325475f, 4.176881f, 6.809522f, 5.031857f, 4.899546f, 13.841108f, 8.348374f, 4.190914f, 19.848215f, 12.258481f, 15.511174f, 52.398117f, 30.707096f, 14.063045f, 73.788632f, 92.587937f, 15.350079f, 14.892324f, 7.635139f, 16.256837f, 74.078501f, 92.951331f, 15.386778f, 14.923077f, 7.664205f, 16.320009f, 194.753547f, 286.577522f, 63.692207f, 74.09439f, 28.511543f, 38.938028f, 157.496989f, 181.814399f, 40.827341f, 39.104151f, 22.447578f, 109.573962f, 158.115835f, 182.52874f, 40.943861f, 39.242439f, 22.527076f, 109.96923f, 406.168967f, 523.589256f, 168.374645f, 115.95108f, 67.542708f, 338.947924f, 147.490807f, 164.357486f, 39.415111f, 43.162941f, 18.664597f, 90.450234f, 147.903315f, 164.994573f, 39.569168f, 43.298287f, 18.733628f, 90.736453f, 476.461956f, 411.041735f, 100.761438f, 160.379375f, 53.481394f, 204.839621f, 113.111023f, 201.285376f, 34.536429f, 13.148347f, 52.398394f, 61.246903f, 113.534853f, 202.076283f, 34.671609f, 13.199263f, 52.603367f, 61.483183f, 423.011932f, 754.762843f, 96.715594f, 42.188281f, 188.770341f, 252.080438f, 133.773534f, 75.324269f, 32.498174f, 79.67452f, 17.726664f, 22.037511f, 134.290018f, 75.619205f, 32.625457f, 79.987113f, 17.794877f, 22.123024f, 549.837054f, 372.641752f, 110.358134f, 293.684172f, 53.211003f, 89.091744f, 229.418731f, 161.068621f, 29.67947f, 12.98129f, 33.167163f, 188.550788f, 230.305152f, 161.695538f, 29.79606f, 13.024497f, 33.294555f, 189.275903f, 746.03154f, 381.197004f, 89.232142f, 41.553795f, 119.272232f, 708.47348f, 133.190103f, 61.245904f, 26.214287f, 20.749889f, 77.489021f, 19.230683f, 133.710837f, 61.469264f, 26.291448f, 20.829719f, 77.793427f, 19.303224f, 566.83335f, 207.578825f, 74.589164f, 65.050393f, 209.477965f, 61.431973f, 104.188078f, 52.133103f, 23.756299f, 16.266308f, 63.363963f, 15.6735f, 104.57157f, 52.321186f, 23.849451f, 16.328196f, 63.604887f, 15.735081f, 272.808072f, 236.423171f, 85.617661f, 47.499587f, 173.56062f, 45.563233f, 59.902668f, 60.846683f, 8.991529f, 4.466071f, 6.772376f, 34.627364f, 60.136908f, 61.085541f, 9.02667f, 4.483317f, 6.788603f, 34.707809f, 178.357828f, 170.502522f, 30.404101f, 14.360149f, 22.056674f, 133.206142f, 87.527485f, 123.102326f, 16.48613f, 13.058964f, 15.32471f, 67.615299f, 87.866472f, 123.500494f, 16.542614f, 13.110065f, 15.384463f, 67.880826f, 247.920477f, 390.473873f, 46.244597f, 56.362295f, 48.131093f, 198.755203f, 75.633793f, 115.028216f, 12.465162f, 18.598326f, 12.406601f, 56.824906f, 75.930892f, 115.476454f, 12.511679f, 18.671397f, 12.454434f, 57.028893f, 239.250898f, 650.717238f, 39.854785f, 81.91198f, 48.753335f, 248.427033f, 60.818539f, 75.49555f, 35.542006f, 86.364652f, 26.021898f, 47.188286f, 61.041438f, 75.68967f, 35.65132f, 86.703996f, 26.123804f, 47.323836f, 185.219025f, 275.83755f, 93.841613f, 249.328254f, 78.541321f, 175.173739f, 104.850503f, 244.306057f, 38.263527f, 40.080462f, 19.941262f, 94.788989f, 105.22464f, 245.22593f, 38.412997f, 40.236963f, 20.010232f, 95.146409f, 274.614504f, 883.32409f, 172.860247f, 187.621015f, 65.00895f, 253.239005f};

/* Trained model parameters of finger flexion vs all binary classifier. */
arm_svm_polynomial_instance_f32 finger_flexion_svm;
uint32_t finger_flexion_nbSupportVectors = 30;
uint32_t finger_flexion_degree = 2;
float32_t finger_flexion_coef0 = 0.100000f;
float32_t finger_flexion_gamma = 0.055556f;
float32_t finger_flexion_intercept = -1.014884f;
int32_t finger_flexion_classes[] = {4, 0};
float32_t finger_flexion_dualCoefs[] = {-7.28346935781142e-05f, -7.474637216203663e-05f, -7.5633199653346945e-06f, -1.0066940952491705e-05f, -1.5937988510919257e-07f, -8.615030310837185e-07f, -1.0005647893843713e-06f, -1.3919637640134175e-05f, -6.895401138899601e-06f, -3.9151011979871136e-06f, -1.6985216625482064e-06f, -7.089431896263112e-06f, -1.1872843110840915e-06f, -1.3357652713370589e-06f, -5.943830005359537e-06f, 1.6342109024305375e-05f, 3.0227252673364476e-06f, 1.7534928489732397e-05f, 1.0541000621769407e-06f, 4.3268554330610275e-06f, 5.782651678194183e-07f, 8.299620839739266e-06f, 3.482687441983893e-07f, 1.5739535573417838e-08f, 3.3473373443624654e-06f, 4.3848346720273654e-06f, 1.5191221264661032e-05f, 4.045839047412082e-06f, 1.277486668697016e-07f, 0.00013059815392789303f};
float32_t finger_flexion_supportVectors[] = {43.757319f, 26.97189f, 18.81738f, 22.76935f, 9.87212f, 14.421838f, 43.929254f, 27.070165f, 18.886155f, 22.769814f, 9.910401f, 14.352145f, 185.558637f, 140.319937f, 77.252488f, 110.350059f, 38.191566f, 60.83731f, 6.783081f, 5.012163f, 4.886774f, 13.793773f, 8.325475f, 4.176881f, 6.809522f, 5.031857f, 4.899546f, 13.841108f, 8.348374f, 4.190914f, 19.848215f, 12.258481f, 15.511174f, 52.398117f, 30.707096f, 14.063045f, 98.444644f, 128.133178f, 15.105736f, 14.293339f, 6.66965f, 16.773896f, 98.72131f, 128.559421f, 15.162621f, 14.349238f, 6.688286f, 16.811955f, 335.070658f, 385.144653f, 50.007534f, 60.542063f, 24.809215f, 58.659709f, 73.788632f, 92.587937f, 15.350079f, 14.892324f, 7.635139f, 16.256837f, 74.078501f, 92.951331f, 15.386778f, 14.923077f, 7.664205f, 16.320009f, 194.753547f, 286.577522f, 63.692207f, 74.09439f, 28.511543f, 38.938028f, 114.783357f, 120.01407f, 62.629112f, 61.02108f, 19.277146f, 47.801139f, 115.192963f, 120.391464f, 62.869977f, 61.223307f, 19.27166f, 47.987084f, 400.973662f, 489.272184f, 206.958882f, 230.387994f, 56.037648f, 173.427587f, 146.12385f, 130.530768f, 76.572009f, 48.880107f, 25.089272f, 41.369291f, 146.684346f, 131.003652f, 76.866394f, 48.830918f, 25.150466f, 41.422569f, 554.594097f, 406.899896f, 240.438065f, 178.370137f, 104.385099f, 97.117162f, 103.991202f, 127.373685f, 102.768905f, 78.069336f, 28.577893f, 37.875578f, 104.363801f, 127.873285f, 103.156056f, 78.366278f, 28.685621f, 37.950683f, 236.192544f, 410.654451f, 533.383858f, 351.140045f, 98.584795f, 140.858515f, 117.738246f, 108.455826f, 49.73339f, 56.010166f, 27.62125f, 43.131374f, 118.143265f, 108.797457f, 49.90712f, 56.166584f, 27.678045f, 43.257623f, 318.840788f, 301.034574f, 178.817997f, 161.917946f, 83.168307f, 149.017857f, 71.271853f, 86.144942f, 49.781679f, 61.720474f, 28.198221f, 48.751195f, 71.498404f, 86.481783f, 49.963763f, 61.939729f, 28.308649f, 48.894274f, 179.697589f, 317.939042f, 164.240191f, 202.924821f, 93.729231f, 144.855407f, 58.299456f, 49.057056f, 31.631698f, 23.927002f, 9.774329f, 15.280264f, 58.054533f, 48.95963f, 31.752587f, 23.998486f, 9.768326f, 15.267717f, 176.784233f, 165.267437f, 115.143537f, 95.744499f, 37.683733f, 40.304654f, 71.557933f, 55.115218f, 92.308261f, 43.207864f, 15.574348f, 17.49831f, 71.800137f, 55.295492f, 92.637578f, 43.372443f, 15.631166f, 17.563008f, 201.937779f, 142.778081f, 330.252587f, 135.032862f, 48.350568f, 54.190538f, 83.896105f, 46.202064f, 68.303175f, 51.72987f, 15.825579f, 16.38841f, 84.220162f, 46.378469f, 68.534944f, 51.896469f, 15.885066f, 16.448458f, 238.465348f, 112.719057f, 202.790442f, 252.165359f, 46.540693f, 37.069573f, 76.416366f, 72.132864f, 37.573616f, 91.95045f, 26.895059f, 42.471748f, 76.707234f, 72.411621f, 37.712873f, 92.247924f, 26.99901f, 42.633536f, 253.202957f, 269.430269f, 90.189211f, 482.211874f, 84.684792f, 182.194786f, 81.847368f, 64.207927f, 39.106901f, 94.865016f, 29.966187f, 29.150219f, 82.155771f, 64.427079f, 39.235021f, 95.119372f, 30.041509f, 29.247752f, 252.703372f, 174.28751f, 144.772602f, 477.553678f, 82.981065f, 66.211186f, 60.818539f, 75.49555f, 35.542006f, 86.364652f, 26.021898f, 47.188286f, 61.041438f, 75.68967f, 35.65132f, 86.703996f, 26.123804f, 47.323836f, 185.219025f, 275.83755f, 93.841613f, 249.328254f, 78.541321f, 175.173739f, 78.361075f, 59.998209f, 48.177228f, 27.216148f, 16.270764f, 17.724873f, 78.657253f, 60.230567f, 48.366523f, 27.315244f, 16.334306f, 17.794504f, 185.995755f, 206.348748f, 178.307181f, 113.821504f, 73.239781f, 52.156907f, 15.115102f, 9.720967f, 40.669075f, 44.036704f, 47.917977f, 13.165142f, 15.173484f, 9.759145f, 40.825121f, 44.208497f, 48.105528f, 13.216625f, 37.377165f, 26.922215f, 110.635145f, 145.627942f, 133.184769f, 42.202784f, 30.062972f, 38.551932f, 7.749685f, 10.775142f, 11.81077f, 16.417831f, 30.168798f, 38.682638f, 7.777531f, 10.812215f, 11.794116f, 16.470319f, 84.621235f, 130.763336f, 21.379564f, 35.715015f, 47.833209f, 42.104923f, 107.822007f, 67.896517f, 47.884075f, 26.123447f, 15.252011f, 19.30916f, 108.152669f, 67.943297f, 48.067684f, 26.20009f, 15.305187f, 19.381587f, 473.874358f, 209.838341f, 203.505546f, 81.810805f, 58.366522f, 89.960352f, 20.435463f, 15.752565f, 80.644035f, 129.191243f, 30.811548f, 16.977644f, 20.506693f, 15.812108f, 80.919421f, 129.689668f, 30.926708f, 17.038434f, 83.25722f, 47.235181f, 253.163113f, 423.623859f, 95.878821f, 65.560683f, 69.141117f, 58.697024f, 13.62225f, 71.815167f, 15.831383f, 33.493857f, 69.376222f, 58.926812f, 13.671745f, 72.092913f, 15.885385f, 33.600985f, 201.002124f, 171.472401f, 44.872355f, 412.443849f, 36.048881f, 103.514103f, 103.581503f, 60.247661f, 54.943076f, 33.242305f, 9.096645f, 16.712636f, 103.987719f, 60.450442f, 55.137516f, 33.372146f, 9.132291f, 16.776735f, 266.172804f, 171.511225f, 192.125414f, 141.712936f, 23.249565f, 51.841698f, 106.772828f, 63.010994f, 61.732872f, 24.805124f, 9.038508f, 18.464092f, 107.085754f, 63.233479f, 61.975407f, 24.892266f, 9.067714f, 18.508989f, 349.779511f, 248.455419f, 164.34877f, 93.974647f, 28.911907f, 62.965791f, 121.325469f, 51.627005f, 30.695447f, 39.761415f, 36.085193f, 19.466097f, 121.801338f, 51.829821f, 30.801669f, 39.917418f, 36.126883f, 19.541453f, 443.139543f, 132.370093f, 103.469661f, 132.543708f, 165.212822f, 65.780589f, 105.870132f, 58.947783f, 55.3263f, 47.698259f, 26.244283f, 18.54684f, 106.28612f, 59.17921f, 55.542328f, 47.884881f, 26.287271f, 18.619567f, 352.588236f, 216.155355f, 214.376247f, 226.00421f, 81.062201f, 53.756062f, 133.773534f, 75.324269f, 32.498174f, 79.67452f, 17.726664f, 22.037511f, 134.290018f, 75.619205f, 32.625457f, 79.987113f, 17.794877f, 22.123024f, 549.837054f, 372.641752f, 110.358134f, 293.684172f, 53.211003f, 89.091744f, 83.934031f, 58.921599f, 37.662981f, 14.716295f, 5.704948f, 13.468226f, 84.223546f, 59.133523f, 37.790398f, 14.772858f, 5.725827f, 13.511817f, 202.390367f, 250.520965f, 115.747839f, 60.200853f, 14.810722f, 37.109354f, 43.692705f, 58.739743f, 10.517272f, 27.240954f, 18.466565f, 22.561247f, 43.822386f, 58.882237f, 10.556132f, 27.344084f, 18.532873f, 22.648246f, 122.36223f, 216.981859f, 31.942686f, 81.222964f, 53.76181f, 51.391944f, 96.436436f, 343.736672f, 39.133051f, 12.700607f, 17.348277f, 74.063176f, 96.743318f, 345.076037f, 39.2834f, 12.75051f, 17.409305f, 74.249919f, 308.284601f, 1385.338829f, 111.169629f, 44.371458f, 44.020655f, 196.185732f, 39.420029f, 49.298382f, 14.976678f, 25.664793f, 12.179072f, 22.311679f, 39.574594f, 49.484406f, 15.031869f, 25.755131f, 12.224593f, 22.385935f, 142.589384f, 152.274055f, 61.656468f, 105.548474f, 39.581777f, 82.32127f};


/* Trained model parameters of finger extension vs all binary classifier. */
arm_svm_polynomial_instance_f32 finger_extension_svm;
uint32_t finger_extension_nbSupportVectors = 16;
uint32_t finger_extension_degree = 2;
float32_t finger_extension_coef0 = 0.100000f;
float32_t finger_extension_gamma = 0.055556f;
float32_t finger_extension_intercept = 1.298171f;
int32_t finger_extension_classes[] = {5, 0};
float32_t finger_extension_dualCoefs[] = {-4.091347813250046e-06f, -2.2921052156935068e-07f, -9.339964559428306e-07f, -6.510307346109958e-07f, -3.9437104598121274e-07f, 7.459294208292644e-08f, 3.024418953081193e-07f, 1.2128141619068468e-07f, 1.7939912594354243e-08f, 3.014953983718518e-08f, 8.146703155237441e-07f, 4.667579072904599e-06f, 1.5157085553270625e-07f, 3.805804129122719e-08f, 7.297553730627226e-08f, 8.697042782622995e-09f};
float32_t finger_extension_supportVectors[] = {78.361075f, 59.998209f, 48.177228f, 27.216148f, 16.270764f, 17.724873f, 78.657253f, 60.230567f, 48.366523f, 27.315244f, 16.334306f, 17.794504f, 185.995755f, 206.348748f, 178.307181f, 113.821504f, 73.239781f, 52.156907f, 133.773534f, 75.324269f, 32.498174f, 79.67452f, 17.726664f, 22.037511f, 134.290018f, 75.619205f, 32.625457f, 79.987113f, 17.794877f, 22.123024f, 549.837054f, 372.641752f, 110.358134f, 293.684172f, 53.211003f, 89.091744f, 80.406924f, 44.541714f, 38.817959f, 14.791881f, 6.01056f, 11.000588f, 80.722787f, 44.714262f, 38.960168f, 14.849996f, 6.031809f, 11.041083f, 204.828506f, 117.17506f, 127.75797f, 65.002052f, 17.950861f, 29.003741f, 83.934031f, 58.921599f, 37.662981f, 14.716295f, 5.704948f, 13.468226f, 84.223546f, 59.133523f, 37.790398f, 14.772858f, 5.725827f, 13.511817f, 202.390367f, 250.520965f, 115.747839f, 60.200853f, 14.810722f, 37.109354f, 78.39048f, 43.52145f, 27.3056f, 19.124613f, 26.108745f, 13.845197f, 78.690155f, 43.69093f, 27.405021f, 19.198302f, 26.21125f, 13.898585f, 315.472405f, 134.617313f, 83.305933f, 71.905614f, 65.43807f, 38.311418f, 191.321211f, 111.502499f, 22.101588f, 8.547852f, 17.246402f, 88.6314f, 192.07012f, 111.940446f, 22.171277f, 8.572971f, 17.31033f, 88.967715f, 481.210314f, 316.696877f, 73.608767f, 24.489373f, 54.981428f, 260.732998f, 146.12385f, 130.530768f, 76.572009f, 48.880107f, 25.089272f, 41.369291f, 146.684346f, 131.003652f, 76.866394f, 48.830918f, 25.150466f, 41.422569f, 554.594097f, 406.899896f, 240.438065f, 178.370137f, 104.385099f, 97.117162f, 81.792296f, 54.411969f, 76.977354f, 65.75092f, 130.754751f, 18.048917f, 82.112738f, 54.621692f, 77.275718f, 66.009134f, 131.268502f, 18.119201f, 197.090127f, 160.560266f, 287.350117f, 203.322828f, 374.78906f, 47.720693f, 166.291021f, 156.339498f, 106.99142f, 79.392198f, 23.479672f, 49.324837f, 166.904745f, 156.953566f, 107.411788f, 79.564694f, 23.507885f, 49.518479f, 426.315214f, 439.402346f, 530.024272f, 197.326342f, 70.312729f, 153.099558f, 102.074207f, 68.958489f, 51.734352f, 48.293603f, 16.173695f, 24.901061f, 102.473168f, 69.229293f, 51.929806f, 48.427233f, 16.235891f, 24.974871f, 292.97149f, 206.036383f, 167.072216f, 132.479663f, 45.812021f, 110.097465f, 88.980623f, 83.264022f, 67.670942f, 48.781387f, 16.475484f, 22.375716f, 89.212685f, 83.322724f, 67.815205f, 48.972855f, 16.519306f, 22.443436f, 210.207209f, 279.545291f, 225.579978f, 134.129133f, 45.962584f, 47.180801f, 58.299456f, 49.057056f, 31.631698f, 23.927002f, 9.774329f, 15.280264f, 58.054533f, 48.95963f, 31.752587f, 23.998486f, 9.768326f, 15.267717f, 176.784233f, 165.267437f, 115.143537f, 95.744499f, 37.683733f, 40.304654f, 67.402737f, 43.414543f, 65.409664f, 54.398696f, 21.703682f, 14.710213f, 67.637867f, 43.506423f, 65.474298f, 54.551793f, 21.746419f, 14.740054f, 195.075074f, 161.037368f, 273.024023f, 155.677671f, 74.28974f, 50.236605f, 71.557933f, 55.115218f, 92.308261f, 43.207864f, 15.574348f, 17.49831f, 71.800137f, 55.295492f, 92.637578f, 43.372443f, 15.631166f, 17.563008f, 201.937779f, 142.778081f, 330.252587f, 135.032862f, 48.350568f, 54.190538f, 83.896105f, 46.202064f, 68.303175f, 51.72987f, 15.825579f, 16.38841f, 84.220162f, 46.378469f, 68.534944f, 51.896469f, 15.885066f, 16.448458f, 238.465348f, 112.719057f, 202.790442f, 252.165359f, 46.540693f, 37.069573f, 20.911467f, 86.136903f, 66.310523f, 56.266011f, 131.987114f, 14.580931f, 20.9831f, 86.451365f, 66.570388f, 56.485985f, 132.411591f, 14.638223f, 62.85086f, 480.680451f, 228.750258f, 150.544711f, 438.87306f, 40.585548f};


void convert_samples_to_float(const uint8_t *input_buf, uint8_t ch_num, float *output_buf) {
    for (uint8_t i = 0; i < ch_num; i++) {
		uint32_t val = (*(input_buf+i*3) << 16) | (*(input_buf+i*3+1) << 8) | *(input_buf+i*3+2);

		float res = 0;

		if (val & 0x800000) {
			res = -1.0f*CONVERSION_UNIT*((~val & 0xffffff)+1);
		} else {
			res = CONVERSION_UNIT*val;
		}

		*(output_buf+i) = res;
	}
}

void handle_new_samples(struct k_work *item) {
    filter_samples();

    if (!threshold_calibrated || !noise_passed) {
        if (segment_ID < DROP_WINDOWS) {   // Drop the first few windows allowing the filter to settle
            segment_ID++;
            return;
        } else if (segment_ID < DROP_WINDOWS+CALIBRARION_WINDOWS) {   // Accumulate the SD values of each channel for activation channel calculation
            static float32_t ch1_sd, ch2_sd, ch3_sd, ch4_sd, ch5_sd, ch6_sd;
            arm_std_f32(&filter_ch1_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP, &ch1_sd);
            arm_std_f32(&filter_ch2_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP, &ch2_sd);
            arm_std_f32(&filter_ch3_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP, &ch3_sd);
            arm_std_f32(&filter_ch4_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP, &ch4_sd);
            arm_std_f32(&filter_ch5_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP, &ch5_sd);
            arm_std_f32(&filter_ch6_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP, &ch6_sd);
            ch1_calibration_sum += ch1_sd;
            ch2_calibration_sum += ch2_sd;
            ch3_calibration_sum += ch3_sd;
            ch4_calibration_sum += ch4_sd;
            ch5_calibration_sum += ch5_sd;
            ch6_calibration_sum += ch6_sd;
            calibration_windows++;
            segment_ID++;
            return;
        } else if (segment_ID == DROP_WINDOWS+CALIBRARION_WINDOWS) {   // Calculate the activation threshold values of each channel and check noise level
            channel_activation_threshold[0] = 6.0f*ch1_calibration_sum/(float32_t)calibration_windows;
            channel_activation_threshold[1] = 6.0f*ch2_calibration_sum/(float32_t)calibration_windows;
            channel_activation_threshold[2] = 6.0f*ch3_calibration_sum/(float32_t)calibration_windows;
            channel_activation_threshold[3] = 6.0f*ch4_calibration_sum/(float32_t)calibration_windows;
            channel_activation_threshold[4] = 6.0f*ch5_calibration_sum/(float32_t)calibration_windows;
            channel_activation_threshold[5] = 6.0f*ch6_calibration_sum/(float32_t)calibration_windows;
            threshold_calibrated = true;

            noise_passed = true;

            for (uint8_t i = 0; i < 6; i++) {
                if (channel_activation_threshold[i] > 100.0f) {
                    noise_passed = false;
                }
            }

            if (!noise_passed) {
                stop_ads_reading(&ads_dev);
                printk("Calibration error .\n");
                segment_ID++;
                return;
            }

            printk("Calibration done.\n");
            segment_ID++;
            return;
        }
    }

    bool active_segment = segment_samples();

    if (active_segment) {
        feature_extract_segment();
        predict_segment();
        
        uint8_t sum = 0;
        uint8_t class = 0;
        for (size_t i = 0; i < NO_CLASSES; i++) {
            if (segment_predictions[i]) {
                sum++;
                class = i+1;
            }
        }
        
        if (sum > 1) {
            for (size_t i = 0; i < NO_CLASSES; i++) {
                if (segment_predictions[i] && (last_predictions[4] == i+1)) {
                    sum = 1;
                    class = i+1;
                }
            }
        }
        
        if (sum > 1) {
            for (size_t i = 0; i < NO_CLASSES; i++) {
                if (segment_predictions[i] && (last_predictions[3] == i+1)) {
                    sum = 1;
                    class = i+1;
                }
            }
        }
        
        if (sum > 1) {
            for (size_t i = 0; i < NO_CLASSES; i++) {
                if (segment_predictions[i] && (last_predictions[2] == i+1)) {
                    sum = 1;
                    class = i+1;
                }
            }
        }

        last_predictions[0] = last_predictions[1];
        last_predictions[1] = last_predictions[2];
        last_predictions[2] = last_predictions[3];
        last_predictions[3] = last_predictions[4];
        last_predictions[4] = class;
    } else {
        float32_t empty_segment_feature_vector[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        memcpy(&segment_feature_vector[0], &empty_segment_feature_vector[0], FEATURE_VECTOR_SIZE*sizeof(float32_t));

        prediction_outputs_offset++;
        for (size_t i = 0; i < NO_CLASSES; i++) {
            segment_predictions[i] = false;
            prediction_outputs[(NO_CLASSES*(prediction_outputs_offset%PREDICTION_HISTORY_LENGTH))+i] = false;
        }
        
        last_predictions[0] = last_predictions[1];
        last_predictions[1] = last_predictions[2];
        last_predictions[2] = last_predictions[3];
        last_predictions[3] = last_predictions[4];
        last_predictions[4] = 0;
    }

    uint16_t class_points[] = {0, 0, 0, 0, 0, 0};   // 0.: rest, 1.: wrist flexion ... 5.: finger extension
    
    class_points[output_class]++;

    for (size_t i = 0; i < 5; i++) {
        class_points[last_predictions[i]]++;
    }

    uint16_t max_class = 0;
    uint16_t max_class_points = 0;
    for (size_t i = 0; i < 6; i++) {
        if (class_points[i] > max_class_points) {
            max_class = i;
            max_class_points = class_points[i];
        }
    }

    if (max_class == 0) {
            output_class = 0;
    } else {
        if (output_class == 0 || output_class == max_class) {
            output_class = max_class;
        }
    }

    if (output_class == 1) {
        printk("wrist flexion\n");
    } else if (output_class == 2) {
        printk("wrist extension\n");
    } else if (output_class == 3) {
        printk("finger tap\n");
    } else if (output_class == 4) {
        printk("finger flexion\n");
    } else if (output_class == 5) {
        printk("finger extension\n");
    } else {
        printk("rest\n");
    }

    if (segments_to_flash) {
        write_segment_to_flash();
    }
    
    segment_ID++;
}

void filter_samples() {
    // Move the last segments of the previous segment to the begining in order to have overlapping window structure.
    memcpy(&segment_ch1_buffer[0], &segment_ch1_buffer[0]+(WINDOW_SIZE-WINDOW_OVERLAP), WINDOW_OVERLAP*sizeof(float32_t));
    memcpy(&segment_ch2_buffer[0], &segment_ch2_buffer[0]+(WINDOW_SIZE-WINDOW_OVERLAP), WINDOW_OVERLAP*sizeof(float32_t));
    memcpy(&segment_ch3_buffer[0], &segment_ch3_buffer[0]+(WINDOW_SIZE-WINDOW_OVERLAP), WINDOW_OVERLAP*sizeof(float32_t));
    memcpy(&segment_ch4_buffer[0], &segment_ch4_buffer[0]+(WINDOW_SIZE-WINDOW_OVERLAP), WINDOW_OVERLAP*sizeof(float32_t));
    memcpy(&segment_ch5_buffer[0], &segment_ch5_buffer[0]+(WINDOW_SIZE-WINDOW_OVERLAP), WINDOW_OVERLAP*sizeof(float32_t));
    memcpy(&segment_ch6_buffer[0], &segment_ch6_buffer[0]+(WINDOW_SIZE-WINDOW_OVERLAP), WINDOW_OVERLAP*sizeof(float32_t));

    // Move reader index to the beginning of the next window to be processed.
    sample_buffers_read_offset += WINDOW_SIZE-WINDOW_OVERLAP;

    // Get the last WINDOW_SIZE-WINDOW_OVERLAP new samples to apply filtration.
    for (size_t i = 0; i < WINDOW_SIZE-WINDOW_OVERLAP; i++) {
        filter_ch1_buffer[i] = sample_ch1_buffer[((sample_buffers_read_offset+i)%256)];
        filter_ch2_buffer[i] = sample_ch2_buffer[((sample_buffers_read_offset+i)%256)];
        filter_ch3_buffer[i] = sample_ch3_buffer[((sample_buffers_read_offset+i)%256)];
        filter_ch4_buffer[i] = sample_ch4_buffer[((sample_buffers_read_offset+i)%256)];
        filter_ch5_buffer[i] = sample_ch5_buffer[((sample_buffers_read_offset+i)%256)];
        filter_ch6_buffer[i] = sample_ch6_buffer[((sample_buffers_read_offset+i)%256)];
    }

    // Apply filter on all channels' next WINDOW_SIZE-WINDOW_OVERLAP number of samples.
    arm_iir_lattice_f32(&ch1_iir_instance, &filter_ch1_buffer[0], &filter_ch1_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_f32(&ch2_iir_instance, &filter_ch2_buffer[0], &filter_ch2_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_f32(&ch3_iir_instance, &filter_ch3_buffer[0], &filter_ch3_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_f32(&ch4_iir_instance, &filter_ch4_buffer[0], &filter_ch4_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_f32(&ch5_iir_instance, &filter_ch5_buffer[0], &filter_ch5_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_f32(&ch6_iir_instance, &filter_ch6_buffer[0], &filter_ch6_buffer[0], WINDOW_SIZE-WINDOW_OVERLAP);

    // Move the filtered samples into the segment buffer for further processing.
    memcpy(&segment_ch1_buffer[0]+WINDOW_OVERLAP, &filter_ch1_buffer[0], (WINDOW_SIZE-WINDOW_OVERLAP)*sizeof(float32_t));
    memcpy(&segment_ch2_buffer[0]+WINDOW_OVERLAP, &filter_ch2_buffer[0], (WINDOW_SIZE-WINDOW_OVERLAP)*sizeof(float32_t));
    memcpy(&segment_ch3_buffer[0]+WINDOW_OVERLAP, &filter_ch3_buffer[0], (WINDOW_SIZE-WINDOW_OVERLAP)*sizeof(float32_t));
    memcpy(&segment_ch4_buffer[0]+WINDOW_OVERLAP, &filter_ch4_buffer[0], (WINDOW_SIZE-WINDOW_OVERLAP)*sizeof(float32_t));
    memcpy(&segment_ch5_buffer[0]+WINDOW_OVERLAP, &filter_ch5_buffer[0], (WINDOW_SIZE-WINDOW_OVERLAP)*sizeof(float32_t));
    memcpy(&segment_ch6_buffer[0]+WINDOW_OVERLAP, &filter_ch6_buffer[0], (WINDOW_SIZE-WINDOW_OVERLAP)*sizeof(float32_t));
}

bool segment_samples() {
    arm_rms_f32(&segment_ch1_buffer[0], WINDOW_SIZE, &segment_rms_values[0]);
    arm_rms_f32(&segment_ch2_buffer[0], WINDOW_SIZE, &segment_rms_values[1]);
    arm_rms_f32(&segment_ch3_buffer[0], WINDOW_SIZE, &segment_rms_values[2]);
    arm_rms_f32(&segment_ch4_buffer[0], WINDOW_SIZE, &segment_rms_values[3]);
    arm_rms_f32(&segment_ch5_buffer[0], WINDOW_SIZE, &segment_rms_values[4]);
    arm_rms_f32(&segment_ch6_buffer[0], WINDOW_SIZE, &segment_rms_values[5]);

    uint8_t activity = (segment_rms_values[5] > channel_activation_threshold[5]) << 5U | (segment_rms_values[4] > channel_activation_threshold[4]) << 4U | (segment_rms_values[3] > channel_activation_threshold[3]) << 3U | (segment_rms_values[2] > channel_activation_threshold[2]) << 2U | (segment_rms_values[1] > channel_activation_threshold[1]) << 1U | (segment_rms_values[0] > channel_activation_threshold[0]);

    channel_activation_states_offset++;
    channel_activation_sates[channel_activation_states_offset%SEGMENT_HISTORY_LENGTH] = activity;

    if (activity > 0 && channel_activation_sates[(channel_activation_states_offset-1)%SEGMENT_HISTORY_LENGTH] == 0b00000000) {
        return false;
    } else if (activity == 0) {
        uint8_t prev_channel_activation_sates = channel_activation_sates[(channel_activation_states_offset-1)%SEGMENT_HISTORY_LENGTH];
        if ((((prev_channel_activation_sates >> 5U) & 0x01) + ((prev_channel_activation_sates >> 4U) & 0x01) + ((prev_channel_activation_sates >> 3U) & 0x01) + ((prev_channel_activation_sates >> 2U) & 0x01) + ((prev_channel_activation_sates >> 1U) & 0x01) + ((prev_channel_activation_sates & 0x01))) > 2) {
            return true;
        }
    }

    return (activity > 0);
}

void feature_extract_segment() {
    float32_t sd, abs_max;
    uint32_t pMax;

    segment_feature_vector[0] = segment_rms_values[0];
    segment_feature_vector[1] = segment_rms_values[1];
    segment_feature_vector[2] = segment_rms_values[2];
    segment_feature_vector[3] = segment_rms_values[3];
    segment_feature_vector[4] = segment_rms_values[4];
    segment_feature_vector[5] = segment_rms_values[5];

    arm_std_f32(&segment_ch1_buffer[0], WINDOW_SIZE, &sd);
    segment_feature_vector[6] = sd;
    arm_std_f32(&segment_ch2_buffer[0], WINDOW_SIZE, &sd);
    segment_feature_vector[7] = sd;
    arm_std_f32(&segment_ch3_buffer[0], WINDOW_SIZE, &sd);
    segment_feature_vector[8] = sd;
    arm_std_f32(&segment_ch4_buffer[0], WINDOW_SIZE, &sd);
    segment_feature_vector[9] = sd;
    arm_std_f32(&segment_ch5_buffer[0], WINDOW_SIZE, &sd);
    segment_feature_vector[10] = sd;
    arm_std_f32(&segment_ch6_buffer[0], WINDOW_SIZE, &sd);
    segment_feature_vector[11] = sd;

    arm_absmax_f32(&segment_ch1_buffer[0], WINDOW_SIZE, &abs_max, &pMax);
    segment_feature_vector[12] = abs_max;
    arm_absmax_f32(&segment_ch2_buffer[0], WINDOW_SIZE, &abs_max, &pMax);
    segment_feature_vector[13] = abs_max;
    arm_absmax_f32(&segment_ch3_buffer[0], WINDOW_SIZE, &abs_max, &pMax);
    segment_feature_vector[14] = abs_max;
    arm_absmax_f32(&segment_ch4_buffer[0], WINDOW_SIZE, &abs_max, &pMax);
    segment_feature_vector[15] = abs_max;
    arm_absmax_f32(&segment_ch5_buffer[0], WINDOW_SIZE, &abs_max, &pMax);
    segment_feature_vector[16] = abs_max;
    arm_absmax_f32(&segment_ch6_buffer[0], WINDOW_SIZE, &abs_max, &pMax);
    segment_feature_vector[17] = abs_max;
}

void predict_segment() {
    int32_t prediction_output[NO_CLASSES];

    arm_svm_polynomial_predict_f32(&wrist_flexion_svm, &segment_feature_vector[0], &prediction_output[0]);
    arm_svm_polynomial_predict_f32(&wrist_extension_svm, &segment_feature_vector[0], &prediction_output[1]);
    arm_svm_polynomial_predict_f32(&finger_tap_svm, &segment_feature_vector[0], &prediction_output[2]);
    arm_svm_polynomial_predict_f32(&finger_flexion_svm, &segment_feature_vector[0], &prediction_output[3]);
    arm_svm_polynomial_predict_f32(&finger_extension_svm, &segment_feature_vector[0], &prediction_output[4]);

    for (size_t i = 0; i < NO_CLASSES; i++) {
        segment_predictions[i] = (prediction_output[i] != 0);
    }

    prediction_outputs_offset++;
    for (size_t i = 0; i < NO_CLASSES; i++) {
        prediction_outputs[(NO_CLASSES*(prediction_outputs_offset%PREDICTION_HISTORY_LENGTH))+i] = segment_predictions[i];
    }
}

char buf_data[64];

void receive_samples(struct k_work *item) {
    while(k_poll(new_samples_to_read_events, 1, K_NO_WAIT) != 0) {   // Waiting till the end of the SPI reading.
        k_usleep(5);
    }

    convert_samples_to_float(ads_dev.data_buffer+3U, 6, sample_readings_buffer);

    sample_ch1_buffer[sample_buffers_write_offset] = sample_readings_buffer[0];
    sample_ch2_buffer[sample_buffers_write_offset] = sample_readings_buffer[1];
    sample_ch3_buffer[sample_buffers_write_offset] = sample_readings_buffer[2];
    sample_ch4_buffer[sample_buffers_write_offset] = sample_readings_buffer[3];
    sample_ch5_buffer[sample_buffers_write_offset] = sample_readings_buffer[4];
    sample_ch6_buffer[sample_buffers_write_offset] = sample_readings_buffer[5];

    unprocessed_samples++;
    sample_buffers_write_offset++;

    /*uint8_t buf[24];
    memcpy(buf, &sample_readings_buffer[0], 24U);
    uint8_t len = sprintf(buf_data, "%02x%02x%02x%02x,%02x%02x%02x%02x,%02x%02x%02x%02x,%02x%02x%02x%02x,%02x%02x%02x%02x,%02x%02x%02x%02x\n", buf[3], buf[2], buf[1], buf[0], buf[7], buf[6], buf[5], buf[4], buf[11], buf[10], buf[9], buf[8], buf[15], buf[14], buf[13], buf[12], buf[19], buf[18], buf[17], buf[16], buf[23], buf[22], buf[21], buf[20]);

    tx_start();
    send_UART_data(buf_data, len);*/

    // Start processing the samples
    if (unprocessed_samples == WINDOW_SIZE-WINDOW_OVERLAP) {
        k_work_submit_to_queue(&handle_new_samples_work_q, &handle_new_samples_work);
        unprocessed_samples = 0;
    }

    new_samples_to_read_events[0].signal->signaled = 0;
    new_samples_to_read_events[0].state = K_POLL_STATE_NOT_READY;
}

int flash_init() {
	int ret = flash_erase(flash_device, 0x00, 180*4096);
	if (ret < 0) {
		printk("Flash erase failed with error %d\n", ret);
		return ret;
	}

	flash_offset = 0;

	return 0;
}

void enable_write_segment_to_flash(const struct device *flash_dev) {
    if (flash_dev != NULL) {
        flash_device = flash_dev;
        if (flash_init() >= 0) {
            segments_to_flash = true;
        }
    }
}

void write_segment_to_flash() {}

int adc_init() {
    gpio_pin_configure_dt(&adc_pwdn_spec, GPIO_OUTPUT_HIGH);
    gpio_pin_configure_dt(&adc_reset_spec, GPIO_OUTPUT_HIGH);

    k_msleep(500);
	init_ads(&ads_dev, spi4_dev, &drdy_spec, spi_cfg);
	enum gain gains[] = {G_1, G_1, G_1, G_1, G_1, G_1, G_1, G_1};
	int err = config_ads(&ads_dev, true, DR_1kSPS, false, 0x3f, gains);

	if (err < 0) {
		printk("ADS1298 configuration failed with error %d\n", err);
		return err;
	}

	//err = write_register(&ads_dev, CONFIG2_REGISTER, BIT(4) | BIT(0));	// Enable test signal at ~2 Hz

	return 0;
}

void adc_read_data_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	_get_ads_data_async(&ads_dev);
    k_work_submit_to_queue(&receive_samples_work_q, &receive_samples_work);
}

bool get_recording_state() {
    return recording_in_progress;
}

void start_recording() {
    if (!recording_in_progress) {
        if (segments_to_flash) {
            flash_init();
        }
        segment_ID = 0;
        stop_system_auto_sleep_counter();
        recording_in_progress = true;
        start_ads_reading(&ads_dev, adc_read_data_callback);
    }
}

void stop_recording() {
    if (recording_in_progress) {
        stop_ads_reading(&ads_dev);
        recording_in_progress = false;
        start_system_auto_sleep_counter();
    }
}

static const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

void processing_init() {
    gpio_pin_configure_dt(&osc_out_spec, GPIO_OUTPUT_INACTIVE);

    arm_iir_lattice_init_f32(&ch1_iir_instance, FILTER_STAGES, iir_k_coeffs, iir_v_coeffs, ch1_iir_state, WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_init_f32(&ch2_iir_instance, FILTER_STAGES, iir_k_coeffs, iir_v_coeffs, ch2_iir_state, WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_init_f32(&ch3_iir_instance, FILTER_STAGES, iir_k_coeffs, iir_v_coeffs, ch3_iir_state, WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_init_f32(&ch4_iir_instance, FILTER_STAGES, iir_k_coeffs, iir_v_coeffs, ch4_iir_state, WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_init_f32(&ch5_iir_instance, FILTER_STAGES, iir_k_coeffs, iir_v_coeffs, ch5_iir_state, WINDOW_SIZE-WINDOW_OVERLAP);
    arm_iir_lattice_init_f32(&ch6_iir_instance, FILTER_STAGES, iir_k_coeffs, iir_v_coeffs, ch6_iir_state, WINDOW_SIZE-WINDOW_OVERLAP);

    arm_svm_polynomial_init_f32(&wrist_extension_svm, wrist_extension_nbSupportVectors, 18, wrist_extension_intercept, wrist_extension_dualCoefs, wrist_extension_supportVectors, wrist_extension_classes, wrist_extension_degree, wrist_extension_coef0, wrist_extension_gamma);
    arm_svm_polynomial_init_f32(&finger_extension_svm, finger_extension_nbSupportVectors, 18, finger_extension_intercept, finger_extension_dualCoefs, finger_extension_supportVectors, finger_extension_classes, finger_extension_degree, finger_extension_coef0, finger_extension_gamma);
    arm_svm_polynomial_init_f32(&wrist_flexion_svm, wrist_flexion_nbSupportVectors, 18, wrist_flexion_intercept, wrist_flexion_dualCoefs, wrist_flexion_supportVectors, wrist_flexion_classes, wrist_flexion_degree, wrist_flexion_coef0, wrist_flexion_gamma);
    arm_svm_polynomial_init_f32(&finger_tap_svm, finger_tap_nbSupportVectors, 18, finger_tap_intercept, finger_tap_dualCoefs, finger_tap_supportVectors, finger_tap_classes, finger_tap_degree, finger_tap_coef0, finger_tap_gamma);
    arm_svm_polynomial_init_f32(&finger_flexion_svm, finger_flexion_nbSupportVectors, 18, finger_flexion_intercept, finger_flexion_dualCoefs, finger_flexion_supportVectors, finger_flexion_classes, finger_flexion_degree, finger_flexion_coef0, finger_flexion_gamma);

    adc_init();
    
    k_work_queue_init(&receive_samples_work_q);
	k_work_queue_start(&receive_samples_work_q, receive_samples_work_q_stack_area, K_THREAD_STACK_SIZEOF(receive_samples_work_q_stack_area), RECEIVESAMPLES_WORK_Q_PRIORITY, NULL);
	k_work_init(&receive_samples_work, receive_samples);

    k_work_queue_init(&handle_new_samples_work_q);
	k_work_queue_start(&handle_new_samples_work_q, handle_new_samples_work_q_stack_area, K_THREAD_STACK_SIZEOF(handle_new_samples_work_q_stack_area), NEWSAMPLES_WORK_Q_PRIORITY, NULL);
	k_work_init(&handle_new_samples_work, handle_new_samples);
}

void tx_start() {
    gpio_pin_set_dt(&osc_out_spec, 1);
}

void tx_done() {
    gpio_pin_set_dt(&osc_out_spec, 0);
}