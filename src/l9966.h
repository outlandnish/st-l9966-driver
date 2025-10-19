#include "Arduino.h"
#include "SPI.h"
#include "functional"

#define L9966_WHOAMI 0x5A

// (GEN_STATUS) general status register
#define L9966_GEN_STATUS_REG 0x01

// (DEV_V) device version register
#define L9966_DEV_V_REG 0x02

// (HV_REV) HV revision register
#define L9966_HV_REV_REG 0x03

// (DEV_ID) device ID register
#define L9966_DEV_ID_REG 0x04

// CURR_SRC_CTRL_[1:4] current source control registers (0b0010_0001 up to 0b0010_0100)
#define L9966_CURR_SRC_CTRL_REG_1 0x21
#define L9966_CURR_SRC_CTRL_REG_2 0x22
#define L9966_CURR_SRC_CTRL_REG_3 0x23
#define L9966_CURR_SRC_CTRL_REG_4 0x24

// CURR_SRC_CTRL_[5:8] current source control registers (0b0010_0101 up to 0b0010_1000)
#define L9966_CURR_SRC_CTRL_REG_5 0x25
#define L9966_CURR_SRC_CTRL_REG_6 0x26
#define L9966_CURR_SRC_CTRL_REG_7 0x27
#define L9966_CURR_SRC_CTRL_REG_8 0x28

// CURR_SRC_CTRL_[9:12] current source control registers (0b0010_1001 up to 0b0010_1100)
#define L9966_CURR_SRC_CTRL_REG_9  0x29
#define L9966_CURR_SRC_CTRL_REG_10 0x2A
#define L9966_CURR_SRC_CTRL_REG_11 0x2B
#define L9966_CURR_SRC_CTRL_REG_12 0x2C

// CURR_SRC_CTRL_[13:15] current source control registers (0010_1101 up to 0010_1111)
#define L9966_CURR_SRC_CTRL_REG_13 0x2D
#define L9966_CURR_SRC_CTRL_REG_14 0x2E
#define L9966_CURR_SRC_CTRL_REG_15 0x2F

// SWITCH_ROUTE / GTM_AOX_RSENT_CONF Current Source Control Register (0b0011_0000)
#define L9966_SWITCH_ROUTE_REG 0x30

// DWT_VOLT_SRC_LSF_CTRL Dewetting Voltage Source Control Register (0b0011_0001)
#define L9966_DWT_VOLT_SRC_LSF_CTRL_REG 0x31

// DIG_IN_STAT_LTC Channel output digital value during last polling (0b0011_0011)
#define L9966_DIG_IN_STAT_LTC_REG 0x33

// GTM_TO_SENT_ROUTE_1_2 GTM to SENT configuration for channel 1 and 2 (0b0011_0100)
#define L9966_GTM_TO_SENT_ROUTE_1_2_REG 0x34

// GTM_TO_SENT_ROUTE_3_4 GTM to SENT configuration for channel 3 and 4 (0b0011_0101)
#define L9966_GTM_TO_SENT_ROUTE_3_4_REG 0x35

// ACTIVE_DISCHARGE_LSF_CTRL Active Discharge Control Register (0b0011_0110)
#define L9966_ACTIVE_DISCHARGE_LSF_CTRL_REG 0x36

// WAK_MSK Wake-up Source Mask Register (0b0100_0000)
#define L9966_WAK_MSK_REG 0x40

// SLEEP_CONFIG Wake-up source value before sleep (0b0100_0001)
#define L9966_SLEEP_CONFIG_REG 0x41

// WAK_CONFIG Wake-up source register (0b0100_0010)
#define L9966_WAK_CONFIG_REG 0x42

// SOFT_RST_CMD Software Reset Command Register (0b0100_0011)
#define L9966_SOFT_RST_CMD_REG 0x43

// VRS (0b0101_0001)
#define L9966_VRS_REG 0x51

// SQNCR_INT_MSK_FLG Sequencer interrupt register (0b1000_0000)
#define L9966_SQNCR_INT_MSK_FLG_REG 0x80

// SC_CONF Single conversion module register (0b1000_0001)
#define L9966_SC_CONF_REG 0x81

// ADC_TIMING ADC timing register (0b1000_0010)
#define L9966_ADC_TIMING_REG 0x82

// SC_RESULT Single conversion result register (0b1000_0011)
#define L9966_SC_RESULT_REG 0x83

// SQNCR_CMD_[1:15] Sequencer configuration table register channel 1 to 15 (0b1100_0001 up to 0b1100_1111)
#define L9966_SQNCR_CMD_1  0xC1
#define L9966_SQNCR_CMD_2  0xC2
#define L9966_SQNCR_CMD_3  0xC3
#define L9966_SQNCR_CMD_4  0xC4
#define L9966_SQNCR_CMD_5  0xC5
#define L9966_SQNCR_CMD_6  0xC6
#define L9966_SQNCR_CMD_7  0xC7
#define L9966_SQNCR_CMD_8  0xC8
#define L9966_SQNCR_CMD_9  0xC9
#define L9966_SQNCR_CMD_10 0xCA
#define L9966_SQNCR_CMD_11 0xCB
#define L9966_SQNCR_CMD_12 0xCC
#define L9966_SQNCR_CMD_13 0xCD
#define L9966_SQNCR_CMD_14 0xCE
#define L9966_SQNCR_CMD_15 0xCF

// SQNCR_CTRL Sequencer control register (0b1101_0000)
#define L9966_SQNCR_CTRL_REG 0xD0

// SQNCR_RSLT_COPY_CMD Sequencer result register copy CMD 14-0 (0b1101_1111)
#define L9966_SQNCR_RSLT_COPY_CMD_REG 0xDF

// DIG_IN_STAT Channel output digital value (0b1110_0000)
#define L9966_DIG_IN_STAT_REG 0xE0

// SQNCR_RESULT_[1:15] Sequencer result register channel 1 to 15 (0b1110_0001 up to 0b1110_1111)
#define L9966_SQNCR_RESULT_1  0xE1
#define L9966_SQNCR_RESULT_2  0xE2
#define L9966_SQNCR_RESULT_3  0xE3
#define L9966_SQNCR_RESULT_4  0xE4
#define L9966_SQNCR_RESULT_5  0xE5
#define L9966_SQNCR_RESULT_6  0xE6
#define L9966_SQNCR_RESULT_7  0xE7
#define L9966_SQNCR_RESULT_8  0xE8
#define L9966_SQNCR_RESULT_9  0xE9
#define L9966_SQNCR_RESULT_10 0xEA
#define L9966_SQNCR_RESULT_11 0xEB
#define L9966_SQNCR_RESULT_12 0xEC
#define L9966_SQNCR_RESULT_13 0xED
#define L9966_SQNCR_RESULT_14 0xEE
#define L9966_SQNCR_RESULT_15 0xEF

// Enums for register fields
enum class L9966_ComparatorThreshold : uint8_t {
  UTH1 = 0,
  UTH2 = 1,
  UTH3 = 2,
  UTH_RATIOMETRIC = 3
};

enum class L9966_CurrentValue : uint8_t {
  I_7_5uA_PU_1uA_VVAR_600uA_PD = 0,
  I_20uA = 1,
  I_250uA_PU_100uA_PD = 2,
  I_500uA = 3,
  I_1mA = 4,
  I_5mA = 5,
  I_10mA = 6,
  I_20mA = 7
};

enum class L9966_DewettingCurrent : uint8_t {
  DWT_5mA = 0,
  DWT_10mA = 1,
  DWT_20mA = 2,
  DWT_USE_CV = 3
};

enum class L9966_PullMode : uint8_t {
  HIZ = 0,
  PULLUP_VPRE = 1,
  PULLUP_5V_REF = 2,
  PULLUP_VVAR = 3,
  PULLDOWN = 4,
  PULLUP_VPRE_PULLDOWN = 5,
  PULLUP_5V_REF_PULLDOWN = 6,
  PULLUP_VVAR_PULLDOWN = 7
};

enum class L9966_ADCChannel : uint8_t {
  DEBUG_VOLTAGE = 0,
  IO1 = 1, IO2 = 2, IO3 = 3, IO4 = 4,
  IO5 = 5, IO6 = 6, IO7 = 7, IO8 = 8,
  IO9 = 9, IO10 = 10, IO11 = 11, IO12 = 12,
  UBSW = 13,
  VI5V = 14,
  VIX = 15,
  IO13 = 16,
  IO14 = 17,
  IO15 = 18,
  BG_DIV_2 = 19,
  TJ = 20
};

enum class L9966_PullupDivider : uint8_t {
  NO_PULLUP_5V = 0,
  PULLUP_RR1_20V = 1,
  PULLUP_RR2_40V = 2,
  PULLUP_RR3_1_25V = 3
};

// Structs for register data
struct L9966_GSR {
  bool configuration_reset;
  bool using_calibrated_adc;
  bool calibration_fault;
  bool trim_fault;
  bool over_temperature_fault_mask_enabed;
  bool entered_wakeup_event;
  bool voltage_supply_fault;
  bool over_temperature_fault;
};

struct L9966_DeviceInfo {
  uint8_t device_version;
  uint8_t hardware_revision;
  uint8_t device_id;
};

struct L9966_CurrentSourceConfig {
  uint8_t control_channel;          // 0-15 or 0 for force to 0
  L9966_ComparatorThreshold threshold;
  L9966_CurrentValue current_value;
  L9966_DewettingCurrent dewetting_current;
  L9966_PullMode pull_mode;
  bool invert_control;
};

struct L9966_ADCResult {
  bool new_result;
  uint16_t value;  // 12-bit for voltage, 15-bit for resistance
};

struct L9966_DigitalInputStatus {
  uint16_t channel_states;  // Bits 0-14 for channels 1-15
};

class L9966 {
  private:
    uint16_t cs, interrupt, reset;
    bool hardware_address_high;

    SPIClass *spi;
    std::function<void(void)> take_spi;
    std::function<void(void)> release_spi;

    void packFrame(uint8_t address, uint16_t data, bool write, bool burst_mode, uint32_t &frame);

  public:
    L9966(SPIClass *spi, uint16_t cs, uint16_t interrupt, uint16_t reset, bool hardware_address_high, std::function<void(void)> take_spi = nullptr, std::function<void(void)> release_spi = nullptr)
      : spi(spi), cs(cs), interrupt(interrupt), reset(reset), hardware_address_high(hardware_address_high), take_spi(take_spi), release_spi(release_spi) {}
    void begin();

    uint16_t transfer(uint8_t address, uint16_t data, bool write, bool burst_mode);

    // Device identification
    bool whoami();
    L9966_DeviceInfo getDeviceInfo();

    // Status
    L9966_GSR getGeneralStatus();

    // Current source configuration (channels 1-15)
    void setCurrentSourceConfig(uint8_t channel, const L9966_CurrentSourceConfig& config);
    L9966_CurrentSourceConfig getCurrentSourceConfig(uint8_t channel);

    // Digital input status
    L9966_DigitalInputStatus getDigitalInputStatus();
    uint16_t getDigitalInputStatusLastPolling();

    // Single conversion ADC
    void startSingleConversion(L9966_ADCChannel channel, bool voltage_mode, L9966_PullupDivider pullup = L9966_PullupDivider::NO_PULLUP_5V);
    L9966_ADCResult getSingleConversionResult();

    // Sequencer configuration (channels 1-15)
    void setSequencerCommand(uint8_t channel, uint8_t next_channel, bool voltage_mode, L9966_PullupDivider pullup);
    void startSequencer(uint8_t start_channel, bool use_eu1 = true);
    void stopSequencer(bool eu1 = true);
    L9966_ADCResult getSequencerResult(uint8_t channel);
    void copySequencerResults();

    // Software reset
    void softwareReset();

    // Wake/Sleep configuration
    void setWakeMask(uint16_t channel_mask);
    void setSleepConfig(uint16_t target_levels);
};