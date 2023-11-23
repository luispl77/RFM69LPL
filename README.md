# RFM69LPL
RFM69HW custom radio library for OOK/FSK continuous mode only.

Function's descriptions (AI generated): 

1. `init()`: This function initializes the radio module by setting the necessary parameters and putting it in standby mode. It configures the communication settings, such as the baud rate for serial communication and SPI interface, and sets the pin mode for the slave select pin. It also sets the radio mode to standby.

2. `updateSettings()`: This function updates various settings of the radio module. It configures the threshold type for OOK (On-Off Keying) modulation, sets the transmit power level, bandwidth, fixed threshold, frequency, RSSI (Received Signal Strength Indicator) threshold, LNA (Low-Noise Amplifier) gain, modulation type, frequency deviation, and bit rate. It adjusts the necessary registers to reflect the updated settings.

3. `threshTypeFixed(bool fixed)`: This function sets the threshold type for OOK modulation to either fixed or peak. If the `fixed` parameter is true, it configures the threshold type as fixed, otherwise, it sets it as peak. It adjusts the corresponding register accordingly.

4. `setTransmitPower(byte dbm, int PA_modes, int OCP)`: This function sets the transmit power level of the radio module. It takes the desired power level in dBm, PA (Power Amplifier) modes, and OCP (Over Current Protection) configuration as parameters. Based on the PA mode, it writes the appropriate values to the power level register, which determines the transmit power level. It also configures the OCP based on the provided parameter.

5. `rxBegin()`: This function prepares the radio module to receive data by configuring the necessary settings and putting it into receive mode. It sets the pin mode for the interrupt pin and sets the radio mode to receive mode.

6. `txBegin()`: This function prepares the radio module to transmit data by configuring the necessary settings and putting it into transmit mode. It sets the pin mode for the interrupt pin and sets the radio mode to transmit mode.

7. `poll()`: This function reads the state of the interrupt pin, which is used to indicate the availability of data or an event. It returns a boolean value indicating whether the interrupt pin is high or low.

8. `send(bool signal)`: This function writes the provided signal value to the interrupt pin, which is used for transmitting data. It sets the interrupt pin to the specified signal level.

9. `getFrequency()`: This function retrieves the frequency setting of the radio module. It reads the frequency registers and calculates the frequency in MHz based on the stored values. It returns the frequency as a floating-point value.

10. `setFrequencyMHz(float f)`: This function sets the frequency of the radio module using a floating-point value in MHz. It converts the MHz value to the appropriate frequency format and calls the `setFrequency()` function to configure the frequency registers.

11. `setModulationType(uint8_t mod)`: This function sets the modulation type of the radio module. It takes a modulation type parameter (MOD_OOK or MOD_FSK) and configures the modulation settings accordingly. It adjusts the data modulation register to reflect the selected modulation type.

12. `setFrequency(uint32_t freqHz)`: This function sets the frequency of the radio module in Hz. It calculates the frequency values for the frequency registers based on the provided frequency in Hz and writes the values to the corresponding registers.

13. `setLNAGain(byte lna_gain)`: This function sets the LNA (Low-Noise Amplifier) gain of the radio module. It configures the LNA settings by writing the LNA gain value and the input impedance to the corresponding register. It keeps track of the updated LNA gain value.

14. `setBandwidth(uint8_t bw)`: This function sets the bandwidth of the OOK/FSK modulation for the radio module. It adjusts the bandwidth settings by modifying the necessary register, preserving other register values. It keeps track of the updated bandwidth value.

15. `getBandwidthIndex()`: This function determines the index of the bandwidth setting in the register. It extracts the bandwidth information from the register and calculates the index based on the extracted values. It returns the index as a byte value.

16. `setRSSIThreshold(byte rssi)`: This function sets the RSSI (Received Signal Strength Indicator) threshold of the radio module. It configures the RSSI threshold by writing the provided value to the corresponding register. The provided value is not in dBm, but rather a raw threshold value. It keeps track of the updated RSSI threshold.

17. `setFixedThreshold(uint8_t threshold)`: This function sets the fixed threshold for OOK modulation of the radio module. It configures the OOK fixed threshold by writing the provided threshold value to the corresponding register. It keeps track of the updated fixed threshold value.

18. `setSensitivityBoost(bool sensitivity_boost)`: This function sets the sensitivity boost mode of the radio module. It takes a boolean parameter, `sensitivity_boost`, to enable or disable the sensitivity boost mode. It adjusts the necessary register to activate or deactivate the sensitivity boost mode based on the provided parameter. It keeps track of the updated sensitivity boost mode.

19. `setMode(byte newMode)`: This function sets the mode of the radio module. It takes a mode parameter, `newMode`, to specify the desired mode (RF69OOK_MODE_TX, RF69OOK_MODE_RX, RF69OOK_MODE_SYNTH, RF69OOK_MODE_STANDBY, RF69OOK_MODE_SLEEP). It adjusts the necessary register to switch to the specified mode. It also waits for the mode to be ready if transitioning from sleep mode. It keeps track of the updated mode.

20. `readRSSI(bool forceTrigger)`: This function reads the RSSI (Received Signal Strength Indicator) value of the radio module. It takes a boolean parameter, `forceTrigger`, to determine if an RSSI trigger is required before reading the value. It triggers the RSSI measurement if necessary, waits for the measurement to be ready, and returns the RSSI value in dBm.

21. `readReg(byte addr)`: This function reads the value of a register in the radio module. It selects the module, sends the register address with the read bit set, and receives the value from the module. It then deselects the module and returns the register value.

22. `setFrequencyDev(uint32_t deviation)`: This function sets the frequency deviation for the radio module. It takes the deviation value in Hz and adjusts the frequency deviation registers accordingly. The deviation value is divided by 61 to convert it to the appropriate format.

23. `getFrequencyDev()`: This function retrieves the frequency deviation setting of the radio module. It reads the frequency deviation registers and calculates the deviation value in Hz based on the stored values. It returns the deviation value as a uint16_t.

24. `writeReg(byte addr, byte value)`: This function writes a value to a register in the radio module. It selects the module, sends the register address with the write bit set, and sends the value to be written. It then deselects the module.
    
25. `void RFM69LPL::select()`: This function selects the transceiver by setting the necessary SPI settings and pulling the slave select pin low. It configures the SPI data mode, bit order, and clock divider. It prepares the RFM69 module for SPI communication.

26. `void RFM69LPL::unselect()`: This function unselects the transceiver chip by releasing the slave select pin. It sets the pin mode of the slave select pin to OUTPUT and sets the pin high.

27. `void RFM69LPL::readAllRegs()`: This function reads all the registers of the RFM69 module for debugging purposes. It iterates through each register address and reads the corresponding register value. It prints the register address, value in hexadecimal, and binary format.

28. `void RFM69LPL::readAllSettings()`: This function reads and prints all the settings of the RFM69 module. It displays the power level in dBm, fixed threshold, threshold type (fixed or peak), bandwidth, frequency, RSSI threshold, LNA gain, PA mode, OCP configuration, modulation type, sensitivity boost mode, and deviation.

29. `void RFM69LPL::standby()`: This function puts the RFM69 module in standby mode. It calls the `setMode()` function to set the module's mode to standby mode.


