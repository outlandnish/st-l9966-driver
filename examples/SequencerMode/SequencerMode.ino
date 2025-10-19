/**
 * SequencerMode.ino
 *
 * Demonstrates the ADC sequencer mode of the L9966 driver.
 * This example shows how to:
 * - Initialize the L9966
 * - Configure multiple channels for sequencer operation
 * - Set up automatic sequential ADC conversions
 * - Read sequencer results
 *
 * The sequencer automatically cycles through configured channels,
 * which is more efficient than manual single conversions.
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

  Serial.println("L9966 Sequencer Mode Example");

  // Initialize L9966
  l9966.begin();

  // Configure channels 1-3 for voltage measurement
  for (uint8_t i = 1; i <= 3; i++) {
    L9966_CurrentSourceConfig config = {
      .control_channel = 0,
      .threshold = L9966_ComparatorThreshold::UTH1,
      .current_value = L9966_CurrentValue::I_7_5uA_PU_1uA_VVAR_600uA_PD,
      .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
      .pull_mode = L9966_PullMode::HIZ,
      .invert_control = false
    };
    l9966.setCurrentSourceConfig(i, config);
  }

  // Set up sequencer commands
  // Channel 1 -> Channel 2 -> Channel 3 -> Channel 1 (loop)
  l9966.setSequencerCommand(1, 2, true, L9966_PullupDivider::NO_PULLUP_5V);  // Ch1 -> Ch2
  l9966.setSequencerCommand(2, 3, true, L9966_PullupDivider::NO_PULLUP_5V);  // Ch2 -> Ch3
  l9966.setSequencerCommand(3, 1, true, L9966_PullupDivider::NO_PULLUP_5V);  // Ch3 -> Ch1 (loop)

  // Start sequencer from channel 1, using EU1
  l9966.startSequencer(1, true);

  Serial.println("Sequencer started!");
  Serial.println("Reading channels 1-3 in continuous mode");
}

void loop() {
  // Copy latest sequencer results to result registers
  l9966.copySequencerResults();

  delay(10);  // Small delay for copy operation

  // Read results from all three channels
  for (uint8_t ch = 1; ch <= 3; ch++) {
    L9966_ADCResult result = l9966.getSequencerResult(ch);

    if (result.new_result) {
      float voltage = (result.value & 0x0FFF) / 4095.0f * 5.0f;
      Serial.print("Ch");
      Serial.print(ch);
      Serial.print(": ");
      Serial.print(voltage, 3);
      Serial.print("V  ");
    }
  }

  Serial.println();
  delay(1000);
}
