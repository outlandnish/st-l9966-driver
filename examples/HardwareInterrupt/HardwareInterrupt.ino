/**
 * HardwareInterrupt.ino
 *
 * Demonstrates hardware interrupt-based input monitoring with the L9966 driver.
 * This example shows how to:
 * - Configure digital input channels
 * - Register an interrupt callback
 * - Enable hardware interrupts for specific channels
 * - Detect and respond to input state changes efficiently
 *
 * Hardware interrupts are more efficient than polling, as the L9966
 * will trigger the MCU's interrupt line only when monitored inputs change.
 */

#include <SPI.h>
#include <l9966.h>

// Pin definitions
#define INPUT_CS 10
#define INPUT_INTERRUPT 2  // Must be an interrupt-capable pin
#define INPUT_RESET 3

// Channel assignments
#define CLUTCH_SWITCH 4         // Clutch switch
#define AC_PRESSURE_SWITCH 5    // AC pressure switch
#define BRAKE_SWITCH_NC 10      // Brake switch (normally closed)
#define BRAKE_SWITCH_NO 11      // Brake switch (normally open)

// Create L9966 instance (ctrl_cfg=false means CTRL_CFG pin tied to GND)
SPIClass spi;
L9966 l9966(&spi, INPUT_CS, INPUT_INTERRUPT, INPUT_RESET, false);

// Track input states for change detection
uint16_t last_input_state = 0;

// Helper function to check if a specific channel changed
bool checkChannelChange(uint8_t channel, uint16_t changes) {
  return (changes & (1 << (channel - 1))) != 0;
}

// Helper function to get the current state of a channel
bool getChannelState(uint8_t channel, uint16_t current_state) {
  return (current_state >> (channel - 1)) & 0x01;
}

// Interrupt callback - called when any enabled channel changes state
void onInputChange(uint16_t wake_sources) {
  // Read current state of all inputs
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  uint16_t current_state = status.channel_states;

  // Detect which channels actually changed
  uint16_t changes = current_state ^ last_input_state;

  Serial.print("Interrupt! Wake sources: 0x");
  Serial.print(wake_sources, HEX);
  Serial.print(", Changes: 0x");
  Serial.println(changes, HEX);

  // Check each monitored channel
  if (checkChannelChange(CLUTCH_SWITCH, changes)) {
    bool state = getChannelState(CLUTCH_SWITCH, current_state);
    Serial.print("Clutch Switch: ");
    Serial.println(state ? "PRESSED" : "RELEASED");
  }

  if (checkChannelChange(AC_PRESSURE_SWITCH, changes)) {
    bool state = getChannelState(AC_PRESSURE_SWITCH, current_state);
    Serial.print("AC Pressure Switch: ");
    Serial.println(state ? "ON" : "OFF");
  }

  if (checkChannelChange(BRAKE_SWITCH_NC, changes)) {
    bool state = getChannelState(BRAKE_SWITCH_NC, current_state);
    Serial.print("Brake Switch NC: ");
    Serial.println(state ? "HIGH" : "LOW");
  }

  if (checkChannelChange(BRAKE_SWITCH_NO, changes)) {
    bool state = getChannelState(BRAKE_SWITCH_NO, current_state);
    Serial.print("Brake Switch NO: ");
    Serial.println(state ? "HIGH" : "LOW");
  }

  // Update last known state
  last_input_state = current_state;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  SPI.begin();
  Serial.println("L9966 Hardware Interrupt Example");

  // Initialize L9966
  l9966.begin();

  // Configure digital input channels
  Serial.println("Configuring input channels...");

  // Channel 4: Clutch Switch (digital input with pull-up)
  L9966_CurrentSourceConfig clutch_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(CLUTCH_SWITCH, clutch_config);

  // Channel 5: AC Pressure Switch (digital input with pull-down)
  L9966_CurrentSourceConfig ac_pressure_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLDOWN,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(AC_PRESSURE_SWITCH, ac_pressure_config);

  // Channel 10: Brake Switch NC (normally closed, pull-up)
  L9966_CurrentSourceConfig brake_nc_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(BRAKE_SWITCH_NC, brake_nc_config);

  // Channel 11: Brake Switch NO (normally open, pull-down)
  L9966_CurrentSourceConfig brake_no_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLDOWN,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(BRAKE_SWITCH_NO, brake_no_config);

  // Read initial state
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  last_input_state = status.channel_states;

  Serial.print("Initial state: 0x");
  Serial.println(last_input_state, HEX);

  // Set up interrupt callback
  l9966.setInterruptCallback(onInputChange);

  // Enable hardware interrupts for our channels
  // Build channel mask (bit 0 = channel 1, bit 3 = channel 4, etc.)
  uint16_t interrupt_mask = 0;
  interrupt_mask |= (1 << (CLUTCH_SWITCH - 1));        // Channel 4
  interrupt_mask |= (1 << (AC_PRESSURE_SWITCH - 1));   // Channel 5
  interrupt_mask |= (1 << (BRAKE_SWITCH_NC - 1));      // Channel 10
  interrupt_mask |= (1 << (BRAKE_SWITCH_NO - 1));      // Channel 11

  Serial.print("Enabling interrupts for mask: 0x");
  Serial.println(interrupt_mask, HEX);

  l9966.enableInterrupts(interrupt_mask);

  Serial.println("Hardware interrupts enabled!");
  Serial.println("Waiting for input changes...");
}

void loop() {
  // No polling needed! The interrupt callback will be triggered automatically
  // when any monitored input changes state.

  // You can do other work here without worrying about missing input events
  delay(1000);

  // Optional: Periodically print status
  static unsigned long last_print = 0;
  if (millis() - last_print > 10000) {
    Serial.println("Still monitoring... (no changes)");
    last_print = millis();
  }
}
