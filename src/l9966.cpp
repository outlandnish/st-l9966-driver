#include "l9966.h"

void L9966::begin() {
  // setup chip select
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);

  // attach input interrupt
  pinMode(interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt), []() {
    // do something
  }, FALLING);

  // setup SYNC pin if provided (optional - for sequencer synchronization)
  // SYNC has internal pull-down, so configure as output to drive high when needed
  if (sync != NC) {
    pinMode(sync, OUTPUT);
    digitalWrite(sync, LOW);  // Start low (L9966 has internal pull-down)
  }

  // reset device
  pinMode(reset, OUTPUT);
  digitalWrite(reset, LOW);
  delay(10);
  digitalWrite(reset, HIGH);
  delay(10);

  if (whoami()) {
    Serial.println("L9966 detected");

    // check general status
    auto gsr = getGeneralStatus();
    Serial.print("Configuration reset: ");
    Serial.println(gsr.configuration_reset);
    Serial.print("Using calibrated ADC: ");
    Serial.println(gsr.using_calibrated_adc);
    Serial.print("Calibration fault: ");
    Serial.println(gsr.calibration_fault);
    Serial.print("Trim fault: ");
    Serial.println(gsr.trim_fault);
    Serial.print("Over temperature fault mask enabled: ");
    Serial.println(gsr.over_temperature_fault_mask_enabed);
    Serial.print("Entered wakeup event: ");
    Serial.println(gsr.entered_wakeup_event);
    Serial.print("Voltage supply fault: ");
    Serial.println(gsr.voltage_supply_fault);
    Serial.print("Over temperature fault: ");
    Serial.println(gsr.over_temperature_fault);
  } else {
    Serial.println("L9966 not detected");
  }
}

void L9966::packFrame(uint8_t address, uint16_t data, bool write, bool burst_mode, uint32_t &frame) {
  // assert((write && data != NULL) || !write);
  frame = 0;
  
  // bit 31 = 1, bit 30 = CTRL_CFG, bit 29 = R/W, bit 28 = CLK_MON (0=burst mode, 1=normal), bit 27 - 20 = address, bit 19 - 17: X, bit 16: odd parity bit for instruction
  // bit 15: odd parity bit for data, bit 14 - 0: data / ignored if a read
  frame |= 1 << 31;
  frame |= ctrl_cfg << 30;
  frame |= write << 29;
  frame |= (!burst_mode) << 28;  // Inverted: burst_mode=false -> CLK_MON=1 (normal single frame)
  frame |= address << 20;

  // calculate odd parity of bits 31:16 (including bit 16 itself) -> store in bit 16
  // For odd parity: if bits 31:17 have odd count, bit 16 = 0; if even count, bit 16 = 1
  frame |= (!__builtin_parity(frame >> 17)) << 16;

  // add data if write
  if (write) {
    frame |= data;
  }

  // Always calculate data parity (even for reads - datasheet says parity is always checked)
  // For odd parity: if bits 14:0 have odd count, bit 15 = 0; if even count, bit 15 = 1
  frame |= (!__builtin_parity(frame & 0x7FFF)) << 15;
}

uint16_t L9966::transfer(uint8_t address, uint16_t data, bool write, bool burst_mode) {
  // take SPI interface (if callback provided)
  if (take_spi) {
    take_spi();
  }

  // take SPI interface
  spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  delay(1);

  // send command
  uint32_t tx, rx;
  packFrame(address, data, write, burst_mode, tx);

  // receive response
  #if defined(ESP32)
  rx = spi->transfer32(tx);
  #else
  // Swap bytes for MSB first transmission (STM32 is little-endian)
  uint32_t tx_swapped = __builtin_bswap32(tx);
  uint32_t rx_swapped;
  spi->transfer(&tx_swapped, &rx_swapped, sizeof(uint32_t));
  rx = __builtin_bswap32(rx_swapped);
  #endif

  // check rx for TRANS_F on bit 24
  if (rx & (1 << 24)) {
    Serial.println("L9966 transfer fault");
  }

  // check register address in blind echo (bits 23 - 16)
  if (((rx >> 16) & 0xFF) != address) {
    Serial.printf("L9966 address mismatch: %02X != %02X\n", (rx >> 16) & 0xFF, address);
  }

  // check odd parity of bits 15:0 (including bit 15 which is the parity bit)
  // For odd parity, the total number of 1s in bits 15:0 should be odd
  if (!__builtin_parity(rx & 0xFFFF)) {
    Serial.println("L9966 parity mismatch");
  }

  // return bits 15 - 0 of rx;
  auto response = rx & 0xFFFF;

  // release SPI interface
  delay(1);
  digitalWrite(cs, HIGH);
  spi->endTransaction();

  // Ensure CS remains high for tSPICS-high before next transaction
  delay(1);

  // release SPI interface (if callback provided)
  if (release_spi) {
    release_spi();
  }

  return response;
}


bool L9966::whoami() {
  auto response = transfer(L9966_DEV_ID_REG, NULL, false, false);

  // print response
  return (response & 0xFF) == 0x5A;
}

L9966_GSR L9966::getGeneralStatus() {
  auto response = transfer(L9966_GEN_STATUS_REG, NULL, false, false);
  L9966_GSR gsr;

  // bits 14-13: configuration reset if equal to b10
  // bit 6: using calibrated ADC
  // bit 5: calibration fault
  // bit 4: trim fault
  // bit 3: over temperature fault mask enabled
  // bit 2: entered wakeup event
  // bit 1: voltage supply fault
  // bit 0: over temperature fault
  gsr.configuration_reset = (response & (1 << 14)) && !(response & (1 << 13));
  gsr.using_calibrated_adc = response & (1 << 6);
  gsr.calibration_fault = response & (1 << 5);
  gsr.trim_fault = response & (1 << 4);
  gsr.over_temperature_fault_mask_enabed = response & (1 << 3);
  gsr.entered_wakeup_event = response & (1 << 2);
  gsr.voltage_supply_fault = response & (1 << 1);
  gsr.over_temperature_fault = response & (1 << 0);

  return gsr;
}

L9966_DeviceInfo L9966::getDeviceInfo() {
  L9966_DeviceInfo info;

  info.device_version = transfer(L9966_DEV_V_REG, NULL, false, false) & 0xFF;
  info.hardware_revision = transfer(L9966_HV_REV_REG, NULL, false, false) & 0xFF;
  info.device_id = transfer(L9966_DEV_ID_REG, NULL, false, false) & 0xFF;

  return info;
}

void L9966::setCurrentSourceConfig(uint8_t channel, const L9966_CurrentSourceConfig& config) {
  if (channel < 1 || channel > 15) return;

  uint16_t reg_value = 0;

  // Pack configuration into register format
  reg_value |= (config.control_channel & 0x0F) << 11;  // bits 14:11
  reg_value |= (static_cast<uint8_t>(config.threshold) & 0x03) << 9;  // bits 10:9
  reg_value |= (static_cast<uint8_t>(config.current_value) & 0x07) << 6;  // bits 8:6
  reg_value |= (static_cast<uint8_t>(config.dewetting_current) & 0x03) << 4;  // bits 5:4
  reg_value |= (static_cast<uint8_t>(config.pull_mode) & 0x07) << 1;  // bits 3:1
  reg_value |= config.invert_control ? 1 : 0;  // bit 0

  // Write to appropriate register
  transfer(L9966_CURR_SRC_CTRL_REG_1 + (channel - 1), reg_value, true, false);
}

L9966_CurrentSourceConfig L9966::getCurrentSourceConfig(uint8_t channel) {
  L9966_CurrentSourceConfig config;

  if (channel < 1 || channel > 15) return config;

  uint16_t reg_value = transfer(L9966_CURR_SRC_CTRL_REG_1 + (channel - 1), NULL, false, false);

  // Unpack register into configuration struct
  config.control_channel = (reg_value >> 11) & 0x0F;
  config.threshold = static_cast<L9966_ComparatorThreshold>((reg_value >> 9) & 0x03);
  config.current_value = static_cast<L9966_CurrentValue>((reg_value >> 6) & 0x07);
  config.dewetting_current = static_cast<L9966_DewettingCurrent>((reg_value >> 4) & 0x03);
  config.pull_mode = static_cast<L9966_PullMode>((reg_value >> 1) & 0x07);
  config.invert_control = reg_value & 0x01;

  return config;
}

L9966_DigitalInputStatus L9966::getDigitalInputStatus() {
  L9966_DigitalInputStatus status;
  uint16_t response = transfer(L9966_DIG_IN_STAT_REG, NULL, false, false);
  status.channel_states = response & 0x7FFF;  // bits 14:0
  return status;
}

uint16_t L9966::getDigitalInputStatusLastPolling() {
  uint16_t response = transfer(L9966_DIG_IN_STAT_LTC_REG, NULL, false, false);
  return response & 0x7FFF;  // bits 14:0
}

void L9966::startSingleConversion(L9966_ADCChannel channel, bool voltage_mode, L9966_PullupDivider pullup) {
  uint16_t reg_value = 0;

  reg_value |= 1 << 8;  // ADC_RUN bit
  reg_value |= (static_cast<uint8_t>(channel) & 0x1F) << 3;  // ADC_MUX bits 7:3
  reg_value |= (static_cast<uint8_t>(pullup) & 0x03) << 1;  // PUP_DIV bits 2:1
  reg_value |= voltage_mode ? 1 : 0;  // R_VOLT_MEAS_SELECT bit 0

  transfer(L9966_SC_CONF_REG, reg_value, true, false);
}

L9966_ADCResult L9966::getSingleConversionResult() {
  L9966_ADCResult result;
  uint16_t response = transfer(L9966_SC_RESULT_REG, NULL, false, false);

  result.new_result = (response >> 14) & 0x01;  // bit 14 for voltage mode
  result.value = response & 0x7FFF;  // bits 14:0 for resistance, 11:0 for voltage

  return result;
}

void L9966::setSequencerCommand(uint8_t channel, uint8_t next_channel, bool voltage_mode, L9966_PullupDivider pullup) {
  if (channel < 1 || channel > 15) return;

  uint16_t reg_value = 0;

  reg_value |= (next_channel & 0x0F) << 3;  // NXT_PC bits 6:3
  reg_value |= (static_cast<uint8_t>(pullup) & 0x03) << 1;  // PUP_DIV bits 2:1
  reg_value |= voltage_mode ? 1 : 0;  // R_VOLT_MEAS_SELECT bit 0

  transfer(L9966_SQNCR_CMD_1 + (channel - 1), reg_value, true, false);
}

void L9966::startSequencer(uint8_t start_channel, bool use_eu1, bool sync_enable) {
  if (start_channel < 1 || start_channel > 15) return;

  uint16_t ctrl_value = 0;

  if (use_eu1) {
    ctrl_value |= (start_channel & 0x0F) << 1;  // INIT_PC_EU1 bits 4:1
    if (sync_enable) {
      ctrl_value |= 1 << 6;  // EU1_SYNC_EN - wait for SYNC pin rising edge to start
    }
    ctrl_value |= 1;  // EU1_EN bit 0 - enable sequencer
  } else {
    ctrl_value |= (start_channel & 0x0F) << 9;  // INIT_PC_EU2 bits 12:9
    if (sync_enable) {
      ctrl_value |= 1 << 14;  // EU2_SYNC_EN - wait for SYNC pin rising edge to start
    }
    ctrl_value |= 1 << 8;  // EU2_EN bit 8 - enable sequencer
  }

  transfer(L9966_SQNCR_CTRL_REG, ctrl_value, true, false);
}

void L9966::stopSequencer(bool eu1) {
  uint16_t ctrl_value = transfer(L9966_SQNCR_CTRL_REG, NULL, false, false);

  if (eu1) {
    ctrl_value &= ~(1);  // Clear EU1_EN
  } else {
    ctrl_value &= ~(1 << 8);  // Clear EU2_EN
  }

  transfer(L9966_SQNCR_CTRL_REG, ctrl_value, true, false);
}

void L9966::triggerSync() {
  if (sync != NC) {
    // Generate rising edge pulse on SYNC pin
    // Pulse must be > 5.2μs to pass glitch filter (typical 4.7μs)
    digitalWrite(sync, HIGH);
    delayMicroseconds(10);  // 10μs pulse width
    digitalWrite(sync, LOW);
  }
}

L9966_ADCResult L9966::getSequencerResult(uint8_t channel) {
  L9966_ADCResult result;

  if (channel < 1 || channel > 15) {
    result.new_result = false;
    result.value = 0;
    return result;
  }

  uint16_t response = transfer(L9966_SQNCR_RESULT_1 + (channel - 1), NULL, false, false);

  result.new_result = (response >> 14) & 0x01;  // bit 14 for voltage mode
  result.value = response & 0x7FFF;  // bits 14:0 for resistance, 11:0 for voltage

  return result;
}

void L9966::copySequencerResults() {
  // Reading this register triggers the copy operation
  transfer(L9966_SQNCR_RSLT_COPY_CMD_REG, NULL, false, false);
}

void L9966::softwareReset() {
  // Software reset requires 3 consecutive writes with specific codes
  transfer(L9966_SOFT_RST_CMD_REG, 0x09, true, false);  // First frame: 1001
  delay(1);
  transfer(L9966_SOFT_RST_CMD_REG, 0x06, true, false);  // Second frame: 0110
  delay(1);
  transfer(L9966_SOFT_RST_CMD_REG, 0x03, true, false);  // Third frame: 0011
  delay(100);  // Wait for reset to complete
}

void L9966::setWakeMask(uint16_t channel_mask) {
  transfer(L9966_WAK_MSK_REG, channel_mask & 0x7FFF, true, false);
}

void L9966::setSleepConfig(uint16_t target_levels) {
  transfer(L9966_SLEEP_CONFIG_REG, target_levels & 0x7FFF, true, false);
}