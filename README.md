# RFM69LPL
RFM69HW custom radio library for OOK/FSK continuous mode only.

Functions: 
- `init()` initializes radio in default mode (OOK base settings) and sets radio in standby;
- `updateSettings()`
- `poll()` and `send()` read and write the DIO2 pin;
- `txBegin()`, `rxBegin()` and `standby()` write radio mode registers;
- `setFrequencyMHz()`, `setFrequency()` and `getFrequency()` change and/or read the frequency regs appropriately;
- `setModulationType()` sets either OOK or FSK modulation;
- `setBitrate()` is used when using bit synchronizer (see datasheet);
- `setBandwidth()`, `setRSSIThreshold()`, `setFixedThreshold()`, `setSensitivityBoost()` and `readRSSI()` change receiver settings (see datasheet);
- `sleep()` puts radio in sleep mode;
- `setPowerLevel()`, `setHighPower()` and `setHighPowerRegs()` change transmitter settings regarding the PA;
- `readReg()`, `writeReg()`, `select()`, `unselect()` and `readAllRegs()` are the SPI functions to write/read RFM69's registers.

    init(): This function initializes the radio module by setting the necessary parameters and putting it in standby mode. It configures the communication settings, such as the baud rate for serial communication and SPI interface, and sets the pin mode for the slave select pin. It also sets the radio mode to standby.

    updateSettings(): This function updates various settings of the radio module. It configures the threshold type for OOK (On-Off Keying) modulation, sets the transmit power level, bandwidth, fixed threshold, frequency, RSSI (Received Signal Strength Indicator) threshold, LNA (Low-Noise Amplifier) gain, modulation type, frequency deviation, and bit rate. It adjusts the necessary registers to reflect the updated settings.

    threshTypeFixed(bool fixed): This function sets the threshold type for OOK modulation to either fixed or peak. If the fixed parameter is true, it configures the threshold type as fixed, otherwise, it sets it as peak. It adjusts the corresponding register accordingly.

    setTransmitPower(byte dbm, int PA_modes, int OCP): This function sets the transmit power level of the radio module. It takes the desired power level in dBm, PA (Power Amplifier) modes, and OCP (Over Current Protection) configuration as parameters. Based on the PA mode, it writes the appropriate values to the power level register, which determines the transmit power level. It also configures the OCP based on the provided parameter.

    rxBegin(): This function prepares the radio module to receive data by configuring the necessary settings and putting it into receive mode. It sets the pin mode for the interrupt pin and sets the radio mode to receive mode.

    txBegin(): This function prepares the radio module to transmit data by configuring the necessary settings and putting it into transmit mode. It sets the pin mode for the interrupt pin and sets the radio mode to transmit mode.

    poll(): This function reads the state of the interrupt pin, which is used to indicate the availability of data or an event. It returns a boolean value indicating whether the interrupt pin is high or low.

    send(bool signal): This function writes the provided signal value to the interrupt pin, which is used for transmitting data. It sets the interrupt pin to the specified signal level.

    getFrequency(): This function retrieves the frequency setting of the radio module. It reads the frequency registers and calculates the frequency in MHz based on the stored values. It returns the frequency as a floating-point value.

    setFrequencyMHz(float f): This function sets the frequency of the radio module using a floating-point value in MHz. It converts the MHz value to the appropriate frequency format and calls the setFrequency() function to configure the frequency registers.

    setModulationType(uint8_t mod): This function sets the modulation type of the radio module. It takes a modulation type parameter (MOD_OOK or MOD_FSK) and configures the modulation settings accordingly. It adjusts the data modulation register to reflect the selected modulation type.

    setFrequency(uint32_t freqHz): This function sets the frequency of the radio module in Hz. It calculates the frequency values for the frequency registers based on the provided frequency in Hz and writes the values to the corresponding registers.
