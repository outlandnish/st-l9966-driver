/**
 * DigitalInput.ino
 *
 * Demonstrates basic digital input reading with the L9966 driver.
 * This example shows how to:
 * - Initialize the L9966
 * - Configure a channel as a digital input with pull-up
 * - Read the digital input state
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

  Serial.println("L9966 Digital Input Example");

  // Initialize L9966
  l9966.begin();

  // Configure channel 1 as digital input with pull-up
  L9966_CurrentSourceConfig config = {
    .control_channel = 0,  // Always on
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(1, config);

  Serial.println("Setup complete!");
}

void loop() {
  // Read digital input status
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();

  // Extract channel 1 state (bit 0 corresponds to channel 1)
  bool channel1_state = (status.channel_states >> 0) & 0x01;

  Serial.print("Channel 1: ");
  Serial.println(channel1_state ? "HIGH" : "LOW");

  delay(500);
}
