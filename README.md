# L9966 FlexInput Driver Library

A comprehensive Arduino/PlatformIO library for the STMicroelectronics L9966 FlexInput analog input device designed for automotive applications.

## Features

- **15 Configurable Input Channels** - Each channel can be independently configured
- **Dual-Mode ADC** - Voltage mode (12-bit) and resistance mode (15-bit)
- **Programmable Current Sources** - 7.5µA to 20mA with multiple preset values
- **Flexible Pull Configurations** - Pull-up (VPRE, 5V_REF, VVAR), pull-down, or high-impedance
- **Digital Input Reading** - Read all 15 channels as digital inputs
- **ADC Sequencer** - Automatic sequential conversions for improved efficiency
- **SYNC Pin Support** - Hardware synchronization for sequencer start (optional)
- **SENT Interface** - Channels IO1-IO4 support SENT (Single Edge Nibble Transmission) protocol output
- **Device Diagnostics** - General status, calibration, temperature, and voltage supply monitoring
- **SPI Interface** - Fast communication with optional mutex support for thread safety
- **Low Power Modes** - Wake/sleep configuration with mask support

**Note:** SENT functionality is available in hardware but not yet implemented in this library. Contributions welcome!

## Installation

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    L9966
```

### Arduino IDE
1. Download this repository as a ZIP file
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file

## Hardware Setup

### Minimal Connection
```
L9966          Microcontroller
-----          ---------------
MOSI     --->  MOSI (SPI)
MISO     <---  MISO (SPI)
SCK      <---  SCK  (SPI)
CS       <---  Any GPIO (e.g., D10)
INT      --->  Any GPIO (e.g., D2)
RST      <---  Any GPIO (e.g., D3)
SYNC     <---  Any GPIO (optional, for sequencer sync)
CTRL_CFG --->  GND or VDD (sets chip address)
```

**Note:** SYNC pin is optional and only needed if you want external hardware synchronization of sequencer start. The pin has an internal 105kΩ pull-down resistor and 4.7µs glitch filter.

### Power Supply
- VDD: 5V (typically)
- VBSW: Battery voltage input
- VI5V: 5V reference input
- VIX: Variable voltage reference

### SENT Interface (Optional)
The L9966 has built-in SENT (Single Edge Nibble Transmission) support:
- **IO_1 through IO_4** - Can function as SENT1-SENT4 channels
- **SENT1_GTM1 through SENT4_GTM4** - Digital outputs for SENT channels

SENT is an automotive sensor interface protocol for transmitting sensor data. The L9966 can route GTM (Generic Timer Module) signals to SENT outputs for sensor emulation. This functionality is not yet implemented in the library.

## Basic Usage

```cpp
#include <SPI.h>
#include <l9966.h>

#define INPUT_CS 10
#define INPUT_INTERRUPT 2
#define INPUT_RESET 3

SPIClass spi;
L9966 l9966(&spi, INPUT_CS, INPUT_INTERRUPT, INPUT_RESET, false);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // Initialize L9966
  l9966.begin();

  // Configure channel 1 for voltage measurement
  L9966_CurrentSourceConfig config = {
    .control_channel = 0,  // Always on
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_7_5uA_PU_1uA_VVAR_600uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::HIZ,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(1, config);
}

void loop() {
  // Read voltage on channel 1
  l9966.startSingleConversion(L9966_ADCChannel::IO1, true);
  delay(10);

  L9966_ADCResult result = l9966.getSingleConversionResult();
  float voltage = (result.value & 0x0FFF) / 4095.0f * 5.0f;

  Serial.println(voltage);
  delay(500);
}
```

## API Reference

### Initialization

#### `L9966(SPIClass *spi, uint16_t cs, uint16_t interrupt, uint16_t reset, bool hardware_address_high, std::function<void(void)> take_spi = nullptr, std::function<void(void)> release_spi = nullptr, uint16_t sync = NC)`
Constructor for L9966 instance.

**Parameters:**
- `spi` - Pointer to SPI instance
- `cs` - Chip select pin
- `interrupt` - Interrupt pin
- `reset` - Reset pin
- `hardware_address_high` - Set to match CTRL_CFG pin (false=GND, true=VDD)
- `take_spi` - Optional mutex acquire function for thread safety
- `release_spi` - Optional mutex release function for thread safety
- `sync` - Optional SYNC pin for hardware sequencer synchronization (default: NC)

#### `void begin()`
Initialize the L9966 device. Performs hardware reset and device detection. If SYNC pin is provided, configures it as an output (starts LOW).

### Configuration

#### `void setCurrentSourceConfig(uint8_t channel, const L9966_CurrentSourceConfig& config)`
Configure a channel's current source, pull mode, and comparator settings.

**Parameters:**
- `channel` - Channel number (1-15)
- `config` - Configuration struct containing:
  - `control_channel` - Digital control channel (0-15, 0=always on)
  - `threshold` - Comparator threshold selection
  - `current_value` - Current source value
  - `dewetting_current` - Dewetting protection current
  - `pull_mode` - Pull-up/pull-down configuration
  - `invert_control` - Invert control logic

#### `L9966_CurrentSourceConfig getCurrentSourceConfig(uint8_t channel)`
Read current configuration of a channel.

### ADC Operations

#### `void startSingleConversion(L9966_ADCChannel channel, bool voltage_mode, L9966_PullupDivider pullup = NO_PULLUP_5V)`
Start a single ADC conversion.

**Parameters:**
- `channel` - ADC channel to measure
- `voltage_mode` - true for voltage (12-bit), false for resistance (15-bit)
- `pullup` - Pull-up divider selection (optional)

#### `L9966_ADCResult getSingleConversionResult()`
Get the result of the last single conversion.

**Returns:** `L9966_ADCResult` struct with:
- `new_result` - true if conversion complete
- `value` - ADC result value

### Sequencer Operations

#### `void setSequencerCommand(uint8_t channel, uint8_t next_channel, bool voltage_mode, L9966_PullupDivider pullup)`
Configure a step in the sequencer.

#### `void startSequencer(uint8_t start_channel, bool use_eu1 = true, bool sync_enable = false)`
Start the automatic sequencer from specified channel.

**Parameters:**
- `start_channel` - Starting channel (1-15)
- `use_eu1` - Use execution unit 1 (true) or unit 2 (false)
- `sync_enable` - Enable SYNC pin synchronization (false=start immediately, true=wait for SYNC pulse)

**SYNC Mode:**
When `sync_enable=true`, the sequencer waits for a rising edge on the SYNC pin before starting. This allows external hardware synchronization. Call `triggerSync()` to start the sequencer, or provide an external signal to the SYNC pin.

#### `void stopSequencer(bool eu1 = true)`
Stop the sequencer.

#### `void triggerSync()`
Generate a rising edge pulse on the SYNC pin to trigger sequencer start (when sync_enable=true). Pulse width is 10µs, exceeding the 5.2µs glitch filter requirement.

**Note:** Only effective when startSequencer() was called with sync_enable=true. SYNC pulses are ignored if sequencer is already running.

#### `void copySequencerResults()`
Copy sequencer results to result registers for reading.

#### `L9966_ADCResult getSequencerResult(uint8_t channel)`
Read sequencer result for a specific channel.

### Digital Input

#### `L9966_DigitalInputStatus getDigitalInputStatus()`
Read digital state of all channels.

**Returns:** `L9966_DigitalInputStatus` with `channel_states` bitmask (bits 0-14 = channels 1-15)

#### `uint16_t getDigitalInputStatusLastPolling()`
Read digital states from last automatic polling.

### Device Information & Diagnostics

#### `bool whoami()`
Check if device is responding correctly.

**Returns:** true if device ID matches (0x5A)

#### `L9966_DeviceInfo getDeviceInfo()`
Read device version, hardware revision, and ID.

#### `L9966_GSR getGeneralStatus()`
Read general status register with diagnostic information.

### System Control

#### `void softwareReset()`
Perform software reset of the device.

#### `void setWakeMask(uint16_t channel_mask)`
Configure which channels can wake the device from sleep.

#### `void setSleepConfig(uint16_t target_levels)`
Configure wake-up source levels before sleep.

## Enumerations

### L9966_ComparatorThreshold
- `UTH1`, `UTH2`, `UTH3` - Fixed thresholds
- `UTH_RATIOMETRIC` - Ratiometric threshold

### L9966_CurrentValue
- `I_7_5uA_PU_1uA_VVAR_600uA_PD` - 7.5µA pull-up, 1µA VVAR, 600µA pull-down
- `I_20uA` - 20µA
- `I_250uA_PU_100uA_PD` - 250µA pull-up, 100µA pull-down
- `I_500uA`, `I_1mA`, `I_5mA`, `I_10mA`, `I_20mA`

### L9966_PullMode
- `HIZ` - High impedance (for voltage measurement)
- `PULLUP_VPRE` - Pull-up to VPRE
- `PULLUP_5V_REF` - Pull-up to 5V reference
- `PULLUP_VVAR` - Pull-up to VVAR
- `PULLDOWN` - Pull-down to GND
- `PULLUP_VPRE_PULLDOWN`, `PULLUP_5V_REF_PULLDOWN`, `PULLUP_VVAR_PULLDOWN` - Combined modes

## Examples

See the `examples/` directory for complete working examples:

- **DigitalInput** - Read digital switch states with pull-up
- **AnalogVoltage** - Measure analog voltages (0-5V sensors)
- **ResistanceMeasurement** - Read resistance values (NTC thermistors, resistor ladders)
- **SequencerMode** - Continuous multi-channel ADC conversions

## Common Applications

- **Accelerator Pedal Position** - Dual voltage sensors with validation
- **Temperature Sensors** - NTC thermistors via resistance measurement
- **Switch Detection** - Clutch, brake, cruise control switches
- **Resistor Ladders** - Multi-position switches (e.g., cruise control)
- **Diagnostic Pins** - DLC diagnostic mode detection

## Thread Safety

The library supports optional mutex functions for thread-safe SPI access in RTOS environments:

```cpp
// Define mutex functions
void takeSPI() {
  xSemaphoreTake(spi_mutex, portMAX_DELAY);
}

void releaseSPI() {
  xSemaphoreGive(spi_mutex);
}

// Pass to constructor
L9966 l9966(&spi, CS_PIN, INT_PIN, RST_PIN, false, takeSPI, releaseSPI);
```

## License

MIT License - See LICENSE file for details

## Author

outlandnish

## Links

- [STMicroelectronics L9966 Product Page](https://www.st.com/en/automotive-analog-and-power/l9966.html)
- [Datasheet](https://www.st.com/resource/en/datasheet/l9966.pdf)
