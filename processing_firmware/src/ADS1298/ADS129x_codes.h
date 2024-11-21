/**
 * @file
 * @brief Command, register map, flag and mask definitions
 *  to work with TI ADS129x series ADC.
 * @author Balázs Kráz @ PPKE ITK - Allonic
 * @date 2024
 */

#include <zephyr/types.h>
#include <stddef.h>


/** ----------------------------------------
 *  ADS1298 OPCODE COMMANDS
 *  ----------------------------------------- */

/** Wakeup from standby mode opcode
 */
static const uint8_t WAKEUP_COMMAND = 0x02;

/** Enter standby mode opcode
 */
static const uint8_t STANDBY_COMMAND = 0x04;

/** Reset the device opcode.
 *  Reset registers to default values.
 */
static const uint8_t RESET_COMMAND = 0x06;

/** Start/restart (synchronize) conversions opcode
 */
static const uint8_t START_COMMAND = 0x08;

/** Stop conversion opcode
 */
static const uint8_t STOP_COMMAND = 0x0A;

/** Enable Read Data Continuous mode opcode.
 *  This mode is the default mode at power up,
 *  when in RDATAC mode, the RREG command is ignored.
 */
static const uint8_t RDATAC_COMMAND = 0x10;

/** Stop Read Data Continuous mode opcode
 */
static const uint8_t SDATAC_COMMAND = 0x11;

/** Read data by command opcode.
 *  Supports multiple read back.
 */
static const uint8_t RDATA_COMMAND = 0x12;

/** Read n nnnn-1 registers starting at address r rrrr opcode.
 *  When in RDATAC mode, the RREG command is ignored.
 */
static const uint8_t RREG_COMMAND = 0x20;

/** Write n nnnn-1 registers starting at address r rrrr opcode
 */
static const uint8_t WREG_COMMAND = 0x40;



/** ----------------------------------------
 *  ADS1298 REGISTERS
 *  ----------------------------------------- */

/** ID control register.
 *  Read-only register.
 * 
 *  Address 0x00
 * 
 *  Bit  | Field        | Type  | Description
 *  :--- | :----------: | :---: | -----------------------:
 *  7:5  | _DEV_ID_     | _R_   | Device ID
 *  4:3  |              | _R_   | Reserved [0x02]
 *  2:0  | _DEV_ID_     | _R_   | Channel number ID
 */
static const uint8_t ID_REGISTER = 0x00;

/** Configuration register 1
 * 
 *  Address 0x01; Reset 0x06
 * 
 *  Bit  | Field        | Type  | Description
 *  :--- | :----------: | :---: | -----------------------------------------------:
 *  7    | _HR_         | _R/W_ | High-resolution (1) or low-power mode (0)
 *  6    | _DAISY_EN_   | _R/W_ | Daisy-chain (0) or multiple readback mode (0)
 *  5    | _CLK_EN_     | _R/W_ | Clock output enabled (1) or disabled (0)
 *  4:3  |              | _R/W_ | Reserved [0x00]
 *  2:0  | _DR_         | _R/W_ | Output data rate
 */
static const uint8_t CONFIG1_REGISTER = 0x01;

/** Configuration register 2
 * 
 *  Address 0x02; Reset 0x40
 * 
 *  Bit  | Field        | Type  | Description
 *  :--- | :----------: | :---: | -----------------------------------------------:
 *  7:6  |              | _R/W_ | Reserved [0x01]
 *  5    | _WCT_CHOP_   | _R/W_ | WCT chopping frequency varies (0) or fixed (1)
 *  4    | _INT_TEST_   | _R/W_ | Test signal source internal (1) or external (0)
 *  3    |              | _R/W_ | Reserved [0x00]
 *  2    | _TEST_AMP_   | _R/W_ | Test signal amplitude
 *  1:0  | _TEST_FREQ_  | _R/W_ | Test signal frequency
 */
static const uint8_t CONFIG2_REGISTER = 0x02;

/** Configuration register 3
 * 
 *  Address 0x03; Reset 0x40
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD_REFBUF_     | _R/W_ | Power-down (0) or enable (1) reference buffer
 *  6    |                 | _R/W_ | Reserved [0x01]
 *  5    | _VREF_4V_       | _R/W_ | Reference voltage 2.4V (0) or 4V (1)
 *  4    | _RLD_MEAS_      | _R/W_ | RLD measurement enabled (1) or disabled (0)
 *  3    | _RLDREF_INT_    | _R/W_ | RLDREF signal external (0) or internal (1)
 *  2    | _PD_RLD_        | _R/W_ | RLD buffer powered down (0) or enabled (1)
 *  1    | _RLD_LOFF_SENS_ | _R/W_ | RLD sense is disabled (0) or enabled (1)
 *  0    | _RLD_STAT_      | _R  _ | RLD sense is disabled (0) or enabled (1)
 */
static const uint8_t CONFIG3_REGISTER = 0x03;

/** Lead-off control register
 * 
 *  Address 0x04; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7:5  | _COMP_TH_       | _R/W_ | Lead-off comparator threshold
 *  4    | _VLEAD_OFF_EN_  | _R/W_ | Lead-off detection mode
 *  3:2  | _ILEAD_OFF_     | _R/W_ | Lead-off current magnitude
 *  1:0  | _FLEAD_OFF_     | _R/W_ | Lead-off frequency
 */
static const uint8_t LOFF_REGISTER = 0x04;

/** Channel 1 settings register
 * 
 *  Address 0x05; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD1_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN1_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX1_          | _R/W_ | Channel input
 */
static const uint8_t CH1SET_REGISTER = 0x05;

/** Channel 2 settings register
 * 
 *  Address 0x06; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD2_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN2_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX2_          | _R/W_ | Channel input
 */
static const uint8_t CH2SET_REGISTER = 0x06;

/** Channel 3 settings register
 * 
 *  Address 0x07; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD3_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN3_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX3_          | _R/W_ | Channel input
 */
static const uint8_t CH3SET_REGISTER = 0x07;

/** Channel 4 settings register
 * 
 *  Address 0x08; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD4_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN4_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX4_          | _R/W_ | Channel input
 */
static const uint8_t CH4SET_REGISTER = 0x08;

/** Channel 5 settings register
 * 
 *  Address 0x09; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD5_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN5_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX5_          | _R/W_ | Channel input
 */
static const uint8_t CH5SET_REGISTER = 0x09;

/** Channel 6 settings register
 * 
 *  Address 0x0A; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD6_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN6_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX6_          | _R/W_ | Channel input
 */
static const uint8_t CH6SET_REGISTER = 0x0A;

/** Channel 7 settings register
 * 
 *  Address 0x0B; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD7_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN7_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX7_          | _R/W_ | Channel input
 */
static const uint8_t CH7SET_REGISTER = 0x0B;

/** Channel 8 settings register
 * 
 *  Address 0x0C; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _PD8_           | _R/W_ | Power-down (1) or normal operation (0)
 *  6:4  | _GAIN8_         | _R/W_ | PGA gain
 *  3    |                 | _R/W_ | Reserved [0x00]
 *  2:0  | _MUX8_          | _R/W_ | Channel input
 */
static const uint8_t CH8SET_REGISTER = 0x0C;

/** RLD positive signal derivation register
 * 
 *  Address 0x0D; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _RLD8P_         | _R/W_ | IN8P to RLD enabled (1) or disabled(0)
 *  6    | _RLD7P_         | _R/W_ | IN7P to RLD enabled (1) or disabled(0)
 *  5    | _RLD6P_         | _R/W_ | IN6P to RLD enabled (1) or disabled(0)
 *  4    | _RLD5P_         | _R/W_ | IN5P to RLD enabled (1) or disabled(0)
 *  3    | _RLD4P_         | _R/W_ | IN4P to RLD enabled (1) or disabled(0)
 *  2    | _RLD3P_         | _R/W_ | IN3P to RLD enabled (1) or disabled(0)
 *  1    | _RLD2P_         | _R/W_ | IN2P to RLD enabled (1) or disabled(0)
 *  0    | _RLD1P_         | _R/W_ | IN1P to RLD enabled (1) or disabled(0)
 */
static const uint8_t RLD_SENSP_REGISTER = 0x0D;

/** RLD negative signal derivation register
 * 
 *  Address 0x0E; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _RLD8N_         | _R/W_ | IN8N to RLD enabled (1) or disabled(0)
 *  6    | _RLD7N_         | _R/W_ | IN7N to RLD enabled (1) or disabled(0)
 *  5    | _RLD6N_         | _R/W_ | IN6N to RLD enabled (1) or disabled(0)
 *  4    | _RLD5N_         | _R/W_ | IN5N to RLD enabled (1) or disabled(0)
 *  3    | _RLD4N_         | _R/W_ | IN4N to RLD enabled (1) or disabled(0)
 *  2    | _RLD3N_         | _R/W_ | IN3N to RLD enabled (1) or disabled(0)
 *  1    | _RLD2N_         | _R/W_ | IN2N to RLD enabled (1) or disabled(0)
 *  0    | _RLD1N_         | _R/W_ | IN1N to RLD enabled (1) or disabled(0)
 */
static const uint8_t RLD_SENSN_REGISTER = 0x0E;

/** Positive signal lead-off detection register
 * 
 *  Address 0x0F; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _LOFF8P_        | _R/W_ | IN8P lead off enabled (1) or disabled (0)
 *  6    | _LOFF7P_        | _R/W_ | IN7P lead off enabled (1) or disabled (0)
 *  5    | _LOFF6P_        | _R/W_ | IN6P lead off enabled (1) or disabled (0)
 *  4    | _LOFF5P_        | _R/W_ | IN5P lead off enabled (1) or disabled (0)
 *  3    | _LOFF4P_        | _R/W_ | IN4P lead off enabled (1) or disabled (0)
 *  2    | _LOFF3P_        | _R/W_ | IN3P lead off enabled (1) or disabled (0)
 *  1    | _LOFF2P_        | _R/W_ | IN2P lead off enabled (1) or disabled (0)
 *  0    | _LOFF1P_        | _R/W_ | IN1P lead off enabled (1) or disabled (0)
 */
static const uint8_t LOFF_SENSP_REGISTER = 0x0F;

/** Negative signal lead-off detection register
 * 
 *  Address 0x10; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7    | _LOFF8N_        | _R/W_ | IN8N lead off enabled (1) or disabled (0)
 *  6    | _LOFF7N_        | _R/W_ | IN7N lead off enabled (1) or disabled (0)
 *  5    | _LOFF6N_        | _R/W_ | IN6N lead off enabled (1) or disabled (0)
 *  4    | _LOFF5N_        | _R/W_ | IN5N lead off enabled (1) or disabled (0)
 *  3    | _LOFF4N_        | _R/W_ | IN4N lead off enabled (1) or disabled (0)
 *  2    | _LOFF3N_        | _R/W_ | IN3N lead off enabled (1) or disabled (0)
 *  1    | _LOFF2N_        | _R/W_ | IN2N lead off enabled (1) or disabled (0)
 *  0    | _LOFF1N_        | _R/W_ | IN1N lead off enabled (1) or disabled (0)
 */
static const uint8_t LOFF_SENSN_REGISTER = 0x10;

/** Lead-off flip register
 * 
 *  Address 0x11; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | ---------------------------------------------:
 *  7    | _LOFF_FLIP8_    | _R/W_ | Channel 8 LOFF polarity flipped (1) or not (0)
 *  6    | _LOFF_FLIP7_    | _R/W_ | Channel 7 LOFF polarity flipped (1) or not (0)
 *  5    | _LOFF_FLIP6_    | _R/W_ | Channel 6 LOFF polarity flipped (1) or not (0)
 *  4    | _LOFF_FLIP5_    | _R/W_ | Channel 5 LOFF polarity flipped (1) or not (0)
 *  3    | _LOFF_FLIP4_    | _R/W_ | Channel 4 LOFF polarity flipped (1) or not (0)
 *  2    | _LOFF_FLIP3_    | _R/W_ | Channel 3 LOFF polarity flipped (1) or not (0)
 *  1    | _LOFF_FLIP2_    | _R/W_ | Channel 2 LOFF polarity flipped (1) or not (0)
 *  0    | _LOFF_FLIP1_    | _R/W_ | Channel 1 LOFF polarity flipped (1) or not (0)
 */
static const uint8_t LOFF_FLIP_REGISTER = 0x11;

/** Lead-off positive signal status register.
 *  Read-only register.
 * 
 *  Address 0x12; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | ------------------------------------------------:
 *  7    | _IN8P_OFF_      | _R_   | Channel 8 positive electrode is on (0) or off (1)
 *  6    | _IN7P_OFF_      | _R_   | Channel 7 positive electrode is on (0) or off (1)
 *  5    | _IN6P_OFF_      | _R_   | Channel 6 positive electrode is on (0) or off (1)
 *  4    | _IN5P_OFF_      | _R_   | Channel 5 positive electrode is on (0) or off (1)
 *  3    | _IN4P_OFF_      | _R_   | Channel 4 positive electrode is on (0) or off (1)
 *  2    | _IN3P_OFF_      | _R_   | Channel 3 positive electrode is on (0) or off (1)
 *  1    | _IN2P_OFF_      | _R_   | Channel 2 positive electrode is on (0) or off (1)
 *  0    | _IN1P_OFF_      | _R_   | Channel 1 positive electrode is on (0) or off (1)
 */
static const uint8_t LOFF_STATP_REGISTER = 0x12;

/** Lead-off negative signal status register.
 *  Read-only register.
 * 
 *  Address 0x13 Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | ------------------------------------------------:
 *  7    | _IN8N_OFF_      | _R_   | Channel 8 negative electrode is on (0) or off (1)
 *  6    | _IN7N_OFF_      | _R_   | Channel 7 negative electrode is on (0) or off (1)
 *  5    | _IN6N_OFF_      | _R_   | Channel 6 negative electrode is on (0) or off (1)
 *  4    | _IN5N_OFF_      | _R_   | Channel 5 negative electrode is on (0) or off (1)
 *  3    | _IN4N_OFF_      | _R_   | Channel 4 negative electrode is on (0) or off (1)
 *  2    | _IN3N_OFF_      | _R_   | Channel 3 negative electrode is on (0) or off (1)
 *  1    | _IN2N_OFF_      | _R_   | Channel 2 negative electrode is on (0) or off (1)
 *  0    | _IN1N_OFF_      | _R_   | Channel 1 negative electrode is on (0) or off (1)
 */
static const uint8_t LOFF_STATN_REGISTER = 0x13;

/** General-purpose I/O register
 * 
 *  Address 0x14; Reset 0x0F
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7:4  | _GPIOD_         | _R/W_ | GPIO data pin state for reading and writing
 *  3:0  | _GPIOC_         | _R/W_ | GPIO control input (1) or output (0) pin
 */
static const uint8_t GPIO_REGISTER = 0x14;

/** Pace detect register
 * 
 *  Address 0x15; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | --------------------------------------------:
 *  7:5  |                 | _R/W_ | Reserved [0x00]
 *  4:3  | _PACEE_         | _R/W_ | Pace even channels
 *  2:1  | _PACEO_         | _R/W_ | Pace odd channels
 *  0    | _PD_PACE_       | _R/W_ | Pace detect buffer turned on (1) or off (0)
 */
static const uint8_t PACE_REGISTER = 0x15;

/** Respiration control register.
 *  Respiration circuitry is not available for ADS1298.
 * 
 *  Address 0x16; Reset 0x00
 * 
 *  Bit  | Field            | Type  | Description
 *  :--- | :--------------: | :---: | ---------------------------------------------:
 *  7    | _RESP_DEMOD_EN1_ | _R/W_ | RESP demodulation circuitry state [0x00]
 *  6    | _RESP_MOD_EN1_   | _R/W_ | RESP modulation circuitry state [0x00]
 *  5    |                  | _R/W_ | Reserved [0x00]
 *  4:2  | _RESP_PH_        | _R/W_ | Respiration phase
 *  1:0  | _RESP_CTRL_      | _R/W_ | Respiration control
 */
static const uint8_t RESP_REGISTER = 0x16;

/** Configuration register 4
 * 
 *  Address 0x17; Reset 0x00
 * 
 *  Bit  | Field           | Type  | Description
 *  :--- | :-------------: | :---: | ---------------------------------------------:
 *  7:5  | _RESP_FREQ_     | _R/W_ | Respiration demodulation frequency
 *  4    |                 | _R/W_ | Reserved [0x00]
 *  3    | _SINGLE_SHOT_   | _R/W_ | Continuous (0) or single-shot (1) conversion 
 *  2    | _WCT_TO_RLD_    | _R/W_ | ECT to RLD connection on (1) or off (0)
 *  1    | _PD_LOFF_COMP_  | _R/W_ | LOFF comparators enabled (1) or disabled (0)
 *  0    |                 | _R/W_ | Reserved [0x00]
 */
static const uint8_t CONFIG4_REGISTER = 0x17;

/** Wilson Central Terminal and augmented lead control register
 * 
 *  Address 0x18; Reset 0x00
 */
static const uint8_t WCT1_REGISTER = 0x18;

/** Wilson Central Terminal control register
 * 
 *  Address 0x19; Reset 0x00
 */
static const uint8_t WCT2_REGISTER = 0x19;


/** ----------------------------------------
 *  ADS1298 REGISTER FLAGS AND MASKS
 *  ----------------------------------------- */

/** Sets high-resolution mode.
 *  Low-power mode is default.
 */
#define CONFIG1_HR		BIT(7)

/** Enables multiple read back mode.
 *  Daisy-chain mode is default.
 */
#define CONFIG1_DAISY_EN		BIT(6)

/** Enables oscillator clock output.
 *  Clock output is disabled by default.
 */
#define CONFIG1_CLK_EN		BIT(5)

/** Output data rate
 *  HR mode: 32 kSPS, LP mode: 16 kSPS.
 * 
 *  This is the default setting.
 */
#define CONFIG1_DR_32kSPS		0x00

/** Output data rate
 *  HR mode: 16 kSPS, LP mode: 8 kSPS.
 */
#define CONFIG1_DR_16kSPS		0x01

/** Output data rate
 *  HR mode: 8 kSPS, LP mode: 4 kSPS.
 */
#define CONFIG1_DR_8kSPS		0x02

/** Output data rate
 *  HR mode: 4 kSPS, LP mode: 2 kSPS.
 */
#define CONFIG1_DR_4kSPS		0x03

/** Output data rate
 *  HR mode: 2 kSPS, LP mode: 1 kSPS.
 */
#define CONFIG1_DR_2kSPS		0x04

/** Output data rate
 *  HR mode: 1 kSPS, LP mode: 500 SPS.
 */
#define CONFIG1_DR_1kSPS		0x05

/** Output data rate
 *  HR mode: 500 SPS, LP mode: 250 SPS.
 */
#define CONFIG1_DR_500SPS		0x06

/** WCT chopping scheme
 *  Sets chopping frequency constant at fMOD/16.
 *  The chopping frequency varies by default.
 */
#define CONFIG2_WCT_CHOP		BIT(5)

/** Test source
 *  Enables internal test signal generation.
 *  Test signals are driven externally by default.
 */
#define CONFIG2_INT_TEST		BIT(4)

/** Test signal amplitude
 *  Sets calibration signal amolitude to 2×–(VREFP–VREFN)/2400 V.
 *  1×–(VREFP–VREFN)/2400 V is the default amplitude.
 */
#define CONFIG2_TEST_AMP		BIT(2)

/** Test signal frequency
 *  Sets calibration signal frequency at fclk/2^21.
 * 
 *  This is the default setting.
 */
#define CONFIG2_TEST_FREQ_LOW		0x00

/** Test signal frequency
 *  Sets calibration signal frequency at fclk/2^20.
 */
#define CONFIG2_TEST_FREQ_HIGH		0x01

/** Test signal frequency
 *  Sets calibration signal frequency at DC.
 */
#define CONFIG2_TEST_FREQ_DC		0x03

/** Enables internal reference buffer.
 *  The buffer is powered-down by default.
 */
#define CONFIG3_PD_REFBUF		BIT(7)

/** Sets VREFP to 4 V.
 *  VREFP is set to 2.4 V by default.
 */
#define CONFIG3_VREF_4V		BIT(5)

/** Enables RLD measurement.
 *  RLD measurement is disabled by default.
 */
#define CONFIG3_RLD_MEAS		BIT(4)

/** Sets RLDREF signal source to (AVDD-AVSS)/2 generated internally.
 *  RLDREF signal is fed externally by default.
 */
#define CONFIG3_RLDREF_INT      BIT(3)

/** Enables RLD buffer.
 *  RLD buffer is disabled by default.
 */
#define CONFIG3_PD_RLD      BIT(2)

/** Enables RLD sense function.
 *  RLD sense function is disbaled by default.
 */
#define CONFIG3_RLD_LOFF_SENS      BIT(1)

/** RLD lead-off status mask
 *  RLD is connected (0), RLD is not connected (1).
 */
#define CONFIG3_RLD_STAT      0x01

/** Lead-off comparator threshold
 *  Comparator positive side 95%.
 *  Comparator negative side 5%.
 * 
 *  This is the default setting.
 */
#define LOFF_COMP_TH_95      0x00

/** Lead-off comparator threshold
 *  Comparator positive side 92.5%.
 *  Comparator negative side 7.5%.
 */
#define LOFF_COMP_TH_92_5      0x01

/** Lead-off comparator threshold
 *  Comparator positive side 90%.
 *  Comparator negative side 10%.
 */
#define LOFF_COMP_TH_90     0x02

/** Lead-off comparator threshold
 *  Comparator positive side 87.5%.
 *  Comparator negative side 12.5%.
 */
#define LOFF_COMP_TH_87_5      0x03

/** Lead-off comparator threshold
 *  Comparator positive side 85%.
 *  Comparator negative side 15%.
 */
#define LOFF_COMP_TH_85      0x04

/** Lead-off comparator threshold
 *  Comparator positive side 80%.
 *  Comparator negative side 20%.
 */
#define LOFF_COMP_TH_80      0x05

/** Lead-off comparator threshold
 *  Comparator positive side 75%.
 *  Comparator negative side 25%.
 */
#define LOFF_COMP_TH_75      0x06

/** Lead-off comparator threshold
 *  Comparator positive side 70%.
 *  Comparator negative side 30%.
 */
#define LOFF_COMP_TH_70      0x07

/** Sets lead-off detection mode to pullup or pulldown resistor mode.
 *  Current source lead-off detection mode is the default.
 */
#define LOFF_VLEAD_OFF_EN      BIT(4)

/** Lead-off current magnitude
 *  Current for the lead-off mode is 6 nA. 
 * 
 *  This is teh default setting.
 */
#define LOFF_ILEAD_OFF_6      0x00

/** Lead-off current magnitude
 *  Current for the lead-off mode is 12 nA. 
 */
#define LOFF_ILEAD_OFF_12      0x01 << 2

/** Lead-off current magnitude
 *  Current for the lead-off mode is 18 nA. 
 */
#define LOFF_ILEAD_OFF_18      0x02 << 2

/** Lead-off current magnitude
 *  Current for the lead-off mode is 24 nA. 
 */
#define LOFF_ILEAD_OFF_24      0x03 << 2

/** Lead-off frequency
 *  Lead-off detection is turned off. 
 * 
 *  This is the default setting.
 */
#define LOFF_FLEAD_OFF_DISABLED      0x00

/** Lead-off frequency
 *  AC lead-off detection at fDR/4. 
 */
#define LOFF_FLEAD_OFF_AC      0x01

/** Lead-off frequency
 *  DC lead-off detection. 
 */
#define LOFF_FLEAD_OFF_DC      0x03

/** Powers down the channel.
 *  Normal operation is the default.
 */
#define CHSET_PD      BIT(7)

/** PCA gain
 *  Gain value is 6.
 *  
 *  This is the default setting.
 */
#define CHSET_GAIN_6    0x00

/** PCA gain
 *  Gain value is 1.
 */
#define CHSET_GAIN_1    0x01 << 4

/** PCA gain
 *  Gain value is 2.
 */
#define CHSET_GAIN_2    0x02 << 4

/** PCA gain
 *  Gain value is 3.
 */
#define CHSET_GAIN_3    0x03 << 4

/** PCA gain
 *  Gain value is 4.
 */
#define CHSET_GAIN_4    0x04 << 4

/** PCA gain
 *  Gain value is 8.
 */
#define CHSET_GAIN_8    0x05 << 4

/** PCA gain
 *  Gain value is 12.
 */
#define CHSET_GAIN_12    0x06 << 4

/** Channel input MUX
 *  Normal electrode input.
 * 
 *  This is the default setting.
 */
#define CHSET_MUX_NORMAL    0x00   

/** Channel input MUX
 *  Input shorted.
 */
#define CHSET_MUX_SHORT    0x01

/** Channel input MUX
 *  Used for RLD measurements.
 */
#define CHSET_MUX_RLD    0x02

/** Channel input MUX
 *  MVDD for supply measurement.
 */
#define CHSET_MUX_MVDD    0x03

/** Channel input MUX
 *  Temperature sensor.
 */
#define CHSET_MUX_TEMP      0x04

/** Channel input MUX
 *  Test signal.
 */
#define CHSET_MUX_TEST      0x05

/** Channel input MUX
 *  RLD_DRP - positive electrode is the driver.
 */
#define CHSET_MUX_RLD_DRP      0x06

/** Channel input MUX
 *  RLD_DRN - negative electrode is the driver.
 */
#define CHSET_MUX_RLD_DRN      0x07

/** Routes channel 8 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD8P      BIT(7)

/** Routes channel 7 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD7P      BIT(6)

/** Routes channel 6 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD6P      BIT(5)

/** Routes channel 5 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD5P      BIT(4)

/** Routes channel 4 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD4P      BIT(3)

/** Routes channel 3 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD3P      BIT(2)

/** Routes channel 2 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD2P      BIT(1)

/** Routes channel 1 positive signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSP_RLD1P      BIT(0)

/** Routes channel 8 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD8N      BIT(7)

/** Routes channel 7 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD7N      BIT(6)

/** Routes channel 6 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD6N      BIT(5)

/** Routes channel 5 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD5N      BIT(4)

/** Routes channel 4 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD4N      BIT(3)

/** Routes channel 3 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD3N      BIT(2)

/** Routes channel 2 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD2N      BIT(1)

/** Routes channel 1 negative signal.
 *  into RLD derivation.
 *  Disabled by default.
 */
#define RLD_SENSN_RLD1N      BIT(0)

/** Enables lead-off detection 
 *  on channel 8 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF8P      BIT(7)

/** Enables lead-off detection 
 *  on channel 7 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF7P      BIT(6)

/** Enables lead-off detection 
 *  on channel 6 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF6P      BIT(5)

/** Enables lead-off detection 
 *  on channel 5 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF5P      BIT(4)

/** Enables lead-off detection 
 *  on channel 4 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF4P      BIT(3)

/** Enables lead-off detection 
 *  on channel 3 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF3P      BIT(2)

/** Enables lead-off detection 
 *  on channel 2 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF2P      BIT(1)

/** Enables lead-off detection 
 *  on channel 1 positive signal.
 *  Disabled by default.
 */
#define LOFF_SENSP_LOFF1P      BIT(0)

/** Enables lead-off detection 
 *  on channel 8 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF8N      BIT(7)

/** Enables lead-off detection 
 *  on channel 7 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF7N      BIT(6)

/** Enables lead-off detection 
 *  on channel 6 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF6N      BIT(5)

/** Enables lead-off detection 
 *  on channel 5 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF5N      BIT(4)

/** Enables lead-off detection 
 *  on channel 4 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF4N      BIT(3)

/** Enables lead-off detection 
 *  on channel 3 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF3N      BIT(2)

/** Enables lead-off detection 
 *  on channel 2 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF2N      BIT(1)

/** Enables lead-off detection 
 *  on channel 1 negative signal.
 *  Disabled by default.
 */
#define LOFF_SENSN_LOFF1N      BIT(0)

/** Flips the pullup/pulldown polarity on channel 8.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP8     BIT(7)

/** Flips the pullup/pulldown polarity on channel 7.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP7     BIT(6)

/** Flips the pullup/pulldown polarity on channel 6.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP6     BIT(5)

/** Flips the pullup/pulldown polarity on channel 5.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP5     BIT(4)

/** Flips the pullup/pulldown polarity on channel 4.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP4     BIT(3)

/** Flips the pullup/pulldown polarity on channel 3.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP3     BIT(2)

/** Flips the pullup/pulldown polarity on channel 2.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP2     BIT(1)

/** Flips the pullup/pulldown polarity on channel 1.
 *  No flip by default.
 */
#define LOFF_FLIP_FLIP1     BIT(0)

/** Channel 8 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN8P_OFF     BIT(7)

/** Channel 7 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN7P_OFF     BIT(6)

/** Channel 6 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN6P_OFF     BIT(5)

/** Channel 5 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN5P_OFF     BIT(4)

/** Channel 4 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN4P_OFF     BIT(3)

/** Channel 3 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN3P_OFF     BIT(2)

/** Channel 2 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN2P_OFF     BIT(1)

/** Channel 1 positive lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATP_IN1P_OFF     BIT(0)

/** Channel 8 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN8N_OFF     BIT(7)

/** Channel 7 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN7N_OFF     BIT(6)

/** Channel 6 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN6N_OFF     BIT(5)

/** Channel 5 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN5N_OFF     BIT(4)

/** Channel 4 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN4N_OFF     BIT(3)

/** Channel 3 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN3N_OFF     BIT(2)

/** Channel 2 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN2N_OFF     BIT(1)

/** Channel 8 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN8N_OFF     BIT(7)

/** Channel 7 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN7N_OFF     BIT(6)

/** Channel 6 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN6N_OFF     BIT(5)

/** Channel 5 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN5N_OFF     BIT(4)

/** Channel 4 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN4N_OFF     BIT(3)

/** Channel 3 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN3N_OFF     BIT(2)

/** Channel 2 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN2N_OFF     BIT(1)

/** Channel 1 negative lead-off status mask.
 *  Electrode is on (0), electrode is off (1).
 */
#define LOFF_STATN_IN1N_OFF     BIT(0)

/** GPIO4 data mask.
 *  Pin is low (0), pin is high (1).
 */
#define GPIO_GPIOD_4    BIT(7)

/** GPIO3 data mask.
 *  Pin is low (0), pin is high (1).
 */
#define GPIO_GPIOD_3    BIT(6)

/** GPIO1 data mask.
 *  Pin is low (0), pin is high (1).
 */
#define GPIO_GPIOD_2    BIT(5)

/** GPIO1 data mask.
 *  Pin is low (0), pin is high (1).
 */
#define GPIO_GPIOD_1    BIT(4)

/** Sets the GPIO4 as input.
 *  Input is the default configuration. 
 */
#define GPIO_GPIOC_4    BIT(3)

/** Sets the GPIO3 as input.
 *  Input is the default configuration. 
 */
#define GPIO_GPIOC_3    BIT(2)

/** Sets the GPIO2 as input.
 *  Input is the default configuration. 
 */
#define GPIO_GPIOC_2    BIT(1)

/** Sets the GPIO1 as input.
 *  Input is the default configuration. 
 */
#define GPIO_GPIOC_1    BIT(0)

/** Pace even channels
 *  Channel 2 is available on TEST_PACE_OUT1.
 * 
 *  This is the default setting. 
 */
#define PACE_PACEE_CH2      0x00

/** Pace even channels
 *  Channel 4 is available on TEST_PACE_OUT1.
 */
#define PACE_PACEE_CH4      0x01 << 3

/** Pace even channels
 *  Channel 6 is available on TEST_PACE_OUT1.
 */
#define PACE_PACEE_CH6      0x02 << 3

/** Pace even channels
 *  Channel 8 is available on TEST_PACE_OUT1.
 */
#define PACE_PACEE_CH8      0x03 << 3

/** Pace odd channels
 *  Channel 1 is available on TEST_PACE_OUT2.
 * 
 *  This is the default setting. 
 */
#define PACE_PACEO_CH1      0x00

/** Pace odd channels
 *  Channel 3 is available on TEST_PACE_OUT2.
 */
#define PACE_PACEO_CH3      0x01 << 1

/** Pace odd channels
 *  Channel 5 is available on TEST_PACE_OUT2.
 */
#define PACE_PACEO_CH5      0x02 << 1

/** Pace odd channels
 *  Channel 7 is available on TEST_PACE_OUT2.
 */
#define PACE_PACEO_CH7      0x03 << 1

/** Turns on pace detect buffer.
 *  The buffer is turned off by default.
 */
#define PACE_PD_PACE      BIT(0)

/** Enables respiration demodulation circuitry
 *  on channel 1.
 *  RESP demodulation is turned off by default.
 */
#define RESP_DEMOD_EN1      BIT(7)

/** Enables respiration modulation circuitry
 *  on channel 1.
 *  RESP modulation is turned off by default.
 */
#define RESP_MOD_EN1      BIT(6)

/** Respiration phase
 *  Phase is 22.5°.
 * 
 *  This is the default setting.
 */
#define RESP_PH_22_5      0x00

/** Respiration phase
 *  Phase is 45°.
 */
#define RESP_PH_45      0x01 << 2

/** Respiration phase
 *  Phase is 67.5°.
 */
#define RESP_PH_67_5      0x02 << 2

/** Respiration phase
 *  Phase is 90°.
 */
#define RESP_PH_90      0x03 << 2

/** Respiration phase
 *  Phase is 112.5°.
 */
#define RESP_PH_112_5      0x04 << 2

/** Respiration phase
 *  Phase is 135°.
 */
#define RESP_PH_135     0x05 << 2

/** Respiration phase
 *  Phase is 157.5°.
 */
#define RESP_PH_157_5      0x06 << 2

/** Respiration control
 *  No respiration.
 * 
 *  This is the default setting.
 */
#define RESP_CTRL_DISABLED      0x00

/** Respiration control
 *  External respiration.
 */
#define RESP_CNTRL_EXT      0x01

/** Respiration control
 *  Internal respiration with internal signals.
 */
#define RESP_CNTRL_INT      0x02

/** Respiration control
 *  Internal respiration with user-generated signals.
 */
#define RESP_CNTRL_USER      0x03

/** Respiration modulation frequency
 *  64 kHZ modulation clock.
 * 
 *  This is the default setting.
 */
#define CONFIG4_RESP_FREQ_64k       0x00

/** Respiration modulation frequency
 *  32 kHZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_32k       0x01 << 5

/** Respiration modulation frequency
 *  16 kHZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_16k       0x02 << 5

/** Respiration modulation frequency
 *  8 kHZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_8k       0x03 << 5

/** Respiration modulation frequency
 *  4 kHZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_4k       0x04 << 5

/** Respiration modulation frequency
 *  2 kHZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_2k       0x05 << 5

/** Respiration modulation frequency
 *  1 kHZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_1k       0x06 << 5

/** Respiration modulation frequency
 *  500 HZ modulation clock.
 */
#define CONFIG4_RESP_FREQ_500       0x07 << 5

/** Sets single-shot conversion mode.
 *  Continous conversion mode is default.
 */
#define CONFIG4_SINGLE_SHOT     BIT(3)

/** Connects the WCT to the RLD.
 *  WCT to RLD connection is off by default.
 */
#define CONFIG4_WCT_TO_RLD     BIT(2)

/** Enables lead-off comparators.
 *  Lead-off comparators are powered-down by default.
 */
#define CONFIG4_PD_LOFF_COMP     BIT(1)

/** Enables (WCTA+WCTB)/2 
 *  to the negative input of channel 6.
 *  Disabled by default.
 */
#define WCT1_aVF_CH6     BIT(7)

/** Enables (WCTA+WCTB)/2 
 *  to the negative input of channel 5.
 *  Disabled by default.
 */
#define WCT1_aVF_CH5     BIT(6)

/** Enables (WCTA+WCTB)/2 
 *  to the negative input of channel 7.
 *  Disabled by default.
 */
#define WCT1_aVF_CH7     BIT(5)

/** Enables (WCTA+WCTB)/2 
 *  to the negative input of channel 4.
 *  Disabled by default.
 */
#define WCT1_aVF_CH4     BIT(4)

/** Powers on WCTA.
 *  WCTA is powered down by default.
 */
#define WCT1_PD_WCTA     BIT(3)

/** WCT Amplifier A channel selection
 *  Channel 1 positive input is connected to WCTA amplifier.
 * 
 *  This is the default setting.
 */
#define WCT1_WCTA_CH1P     0x00

/** WCT Amplifier A channel selection
 *  Channel 1 negative input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH1N     0x01

/** WCT Amplifier A channel selection
 *  Channel 2 positive input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH2P     0x02

/** WCT Amplifier A channel selection
 *  Channel 2 negative input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH2N     0x03

/** WCT Amplifier A channel selection
 *  Channel 3 positive input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH3P     0x04

/** WCT Amplifier A channel selection
 *  Channel 3 negative input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH3N     0x05

/** WCT Amplifier A channel selection
 *  Channel 4 positive input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH4P     0x06

/** WCT Amplifier A channel selection
 *  Channel 4 negative input is connected to WCTA amplifier.
 */
#define WCT1_WCTA_CH4N     0x07

/** Powers on WCTC.
 *  WCTC is powered down by default.
 */
#define WCT2_PD_WCTC     BIT(7)

/** Powers on WCTB.
 *  BCTC is powered down by default.
 */
#define WCT2_PD_WCTB     BIT(6)

/** WCT Amplifier B channel selection
 *  Channel 1 positive input is connected to WCTB amplifier.
 * 
 *  This is the default setting.
 */
#define WCT2_WCTB_CH1P     0x00

/** WCT Amplifier B channel selection
 *  Channel 1 negative input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH1N     0x01 << 3

/** WCT Amplifier B channel selection
 *  Channel 2 positive input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH2P     0x02 << 3

/** WCT Amplifier B channel selection
 *  Channel 2 negative input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH2N     0x03 << 3

/** WCT Amplifier B channel selection
 *  Channel 3 positive input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH3P     0x04 << 3

/** WCT Amplifier B channel selection
 *  Channel 3 negative input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH3N     0x05 << 3

/** WCT Amplifier B channel selection
 *  Channel 4 positive input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH4P     0x06 << 3

/** WCT Amplifier B channel selection
 *  Channel 4 negative input is connected to WCTB amplifier.
 */
#define WCT2_WCTB_CH4N     0x07 << 3

/** WCT Amplifier C channel selection
 *  Channel 1 positive input is connected to WCTC amplifier.
 * 
 *  This is the default setting.
 */
#define WCT2_WCTC_CH1P     0x00

/** WCT Amplifier C channel selection
 *  Channel 1 negative input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH1N     0x01

/** WCT Amplifier C channel selection
 *  Channel 2 positive input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH2P     0x02

/** WCT Amplifier C channel selection
 *  Channel 2 negative input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH2N     0x03

/** WCT Amplifier C channel selection
 *  Channel 3 positive input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH3P     0x04

/** WCT Amplifier C channel selection
 *  Channel 3 negative input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH3N     0x05

/** WCT Amplifier C channel selection
 *  Channel 4 positive input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH4P     0x06

/** WCT Amplifier C channel selection
 *  Channel 4 negative input is connected to WCTC amplifier.
 */
#define WCT2_WCTC_CH4N     0x07