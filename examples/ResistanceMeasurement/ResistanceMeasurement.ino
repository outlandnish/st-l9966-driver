/**
 * ResistanceMeasurement.ino
 *
 * Demonstrates resistance measurement with the L9966 driver.
 * This example shows how to:
 * - Initialize the L9966
 * - Configure a channel for resistance measurement
 * - Read resistance values (useful for NTC thermistors, resistor ladders)
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

  Serial.println("L9966 Resistance Measurement Example");

  // Initialize L9966
  l9966.begin();

  // Configure channel 1 for resistance measurement with pull-up
  // Ideal for NTC thermistors
  L9966_CurrentSourceConfig config = {
    .control_channel = 0,  // Always on
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_250uA_PU_100uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(1, config);

  Serial.println("Setup complete!");
  Serial.println("Reading resistance from channel 1 (e.g., NTC thermistor)");
}

void loop() {
  // Start single conversion on channel 1 in resistance mode
  l9966.startSingleConversion(L9966_ADCChannel::IO1, false, L9966_PullupDivider::NO_PULLUP_5V);

  // Wait for conversion to complete
  delay(10);

  // Read result
  L9966_ADCResult result = l9966.getSingleConversionResult();

  if (result.new_result) {
    // Raw 15-bit resistance value
    uint16_t raw_resistance = result.value & 0x7FFF;

    Serial.print("Channel 1 Raw Resistance: ");
    Serial.print(raw_resistance);
    Serial.println(" (raw ADC value)");

    // TODO: Convert to actual resistance using calibration/lookup table
    // For NTC thermistors, you would then use a Steinhart-Hart equation
    // or lookup table to convert resistance to temperature
  } else {
    Serial.println("No new result available");
  }

  delay(500);
}
