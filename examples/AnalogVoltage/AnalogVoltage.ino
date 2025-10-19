/**
 * AnalogVoltage.ino
 *
 * Demonstrates analog voltage reading with the L9966 driver.
 * This example shows how to:
 * - Initialize the L9966
 * - Configure a channel for voltage measurement
 * - Perform single ADC conversions
 * - Convert raw ADC values to voltage
 */

#include <SPI.h>
#include <l9966.h>

// Pin definitions
#define INPUT_CS 10
#define INPUT_INTERRUPT 2
#define INPUT_RESET 3

// Create L9966 instance (ctrl_cfg=false means CTRL_CFG pin tied to GND)
SPIClass spi;
L9966 l9966(&spi, INPUT_CS, INPUT_INTERRUPT, INPUT_RESET, false);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  Serial.println("L9966 Analog Voltage Example");

  // Initialize L9966
  l9966.begin();

  // Configure channel 1 for high-impedance voltage measurement
  L9966_CurrentSourceConfig config = {
    .control_channel = 0,  // Always on
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_7_5uA_PU_1uA_VVAR_600uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::HIZ,  // High impedance for voltage measurement
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(1, config);

  Serial.println("Setup complete!");
}

void loop() {
  // Start single conversion on channel 1 in voltage mode
  l9966.startSingleConversion(L9966_ADCChannel::IO1, true, L9966_PullupDivider::NO_PULLUP_5V);

  // Wait for conversion to complete
  delay(10);

  // Read result
  L9966_ADCResult result = l9966.getSingleConversionResult();

  if (result.new_result) {
    // Convert 12-bit ADC value to voltage (assuming 5V reference)
    float voltage = (result.value & 0x0FFF) / 4095.0f * 5.0f;

    Serial.print("Channel 1 Voltage: ");
    Serial.print(voltage, 3);
    Serial.println(" V");
  } else {
    Serial.println("No new result available");
  }

  delay(500);
}
