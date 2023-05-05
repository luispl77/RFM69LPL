# RFM69LPL
RFM69HW custom radio library for OOK/FSK continuous mode only - register setups and radio control. 

Functions: 
- `init()` initializes radio in default mode (OOK base settings) and sets radio in standby;
- `poll()` and `send()` read and write the DIO2 pin;
- `txBegin()`, `transmitEnd()`, rxBegin()` and `standby()` write radio mode registers;
- `setFrequencyMHz()`, `setFrequency()` and `getFrequency()` change and/or read the frequency regs appropriately;
- `setModulationType()` sets either OOK or FSK modulation;
- `setBitrate()` is used when using bit synchronizer (see datasheet);
- `setBandwidth()`, `setRSSIThreshold()`, `setFixedThreshold()`, `setSensitivityBoost()` and `readRSSI()` change receiver settings (see datasheet);
- `sleep()` puts radio in sleep mode;
- `setPowerLevel()`, `setHighPower()` and `setHighPowerRegs()` change transmitter settings regarding the PA;
- `readReg()`, `writeReg()`, `select()`, `unselect()` and `readAllRegs()` are the SPI functions to write/read RFM69's registers.
