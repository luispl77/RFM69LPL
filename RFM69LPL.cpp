/* This is a library for the RFM69HW, RFM69HCW and RFM69CW (RFM69 variants). 

Made by Luis Pedro Lopes (LPL).

The purpose of this library is to run these radios in continuous mode (no packets), both in OOK and FSK. 
There are a lot of settings in this radio, so correct configuration is of most importance. This is done 
by writing to registers in the radio. After that, the interaction with the radio is done by either writing
to DIO2 pin for transmitting, or reading from DIO2 for receiving, for both OOK or FSK.

All of the settings are kept in memory inside the class.
This library is made primarily for ESP32 and ESP8266 based boards: baud rate for serial is set on init() to 115200.
But apart from that, it should work on other boards.

The main tenet of this library design is to enable simple and robust register writing and reading, allowing changing every setting available in the radio.

*/



#include <RFM69LPL.h>
#include <RFM69LPLregisters.h>
#include <SPI.h>

volatile byte RFM69LPL::_mode;  // current transceiver state
volatile int RFM69LPL::RSSI; 	// most accurate RSSI during reception (closest to the reception)

void RFM69LPL::init(){ //initialize radio with default regs and put in standby
  Serial.begin(115200);
  SPI.begin();
  pinMode(_slaveSelectPin, OUTPUT);

  threshTypeFixed(_thresh_type_fixed);
  setTransmitPower(_dbm, _pa_mode, _ocp);
  setBandwidth(_bandwidth);
  setFixedThreshold(_fixed_threshold); 
  setFrequencyMHz(_frequency);
  threshTypeFixed(_thresh_type_fixed);
  setRSSIThreshold(_rssi_threshold);
  setLNAGain(_lna_gain);
  setModulationType(_modulation);

  setMode(RF69OOK_MODE_STANDBY);
}


void RFM69LPL::threshTypeFixed(bool fixed){
	if(fixed){
		writeReg(REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_FIXED | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000);
	}
	else{
		writeReg(REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000);
	}
  _thresh_type_fixed = fixed;
}

void RFM69LPL::setTransmitPower(byte dbm, int PA_modes, int OCP) { //keep a minimum of -11 dbm to avoid writing negative values into the palevel reg.
  pinMode(_interruptPin, OUTPUT);
  setMode(RF69OOK_MODE_TX); //put in transmit mode
  switch(PA_modes){
    case PA_MODE_PA0:
      writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | (dbm > 13 ? 31 : (dbm + 18)) ); //RegOutputPower max: 31, min: 0; formula: Pout = -18 + Reg_OutputPower
      break;                                                                                                              // Pout: -18 to +13 dBm   (and its on the wrong pin (RFIO) instead of being on PA_BOOST pin, becoming useless?)
    case PA_MODE_PA1:
      writeReg(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_OFF | (dbm > 13 ? 31 : (dbm + 18)) ); //RegOutputPower max: 31, min: 0; formula: Pout = -18 + Reg_OutputPower
      break;                                                                                                              // Pout: -2 to +13 dBm
    case PA_MODE_PA1_PA2:
      writeReg(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON  | (dbm > 17 ? 31 : (dbm + 14)) ); //RegOutputPower max: 31, min: 0; formula: Pout = -14 + Reg_OutputPower
      break;                                                                                                              // Pout: +2 to +17 dBm
    case PA_MODE_PA1_PA2_20dbm:
      writeReg(REG_TESTPA1, 0x5D); //highest power test regs (allows absolute maximum of +20dbm output power)
      writeReg(REG_TESTPA2, 0x7C);
      writeReg(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON  | (dbm > 20 ? 31 : (dbm + 11)) ); //RegOutputPower max: 31, min: 0; formula: Pout = -11 + Reg_OutputPower
      break;                                                                                                              // Pout: +5 to +20 dBm
  }
  writeReg(REG_OCP, OCP ? RF_OCP_ON : RF_OCP_OFF); //OCP_ON = 1, OCP_OFF = 0 (over current protection enable/disable using these defines)
  //note: highest power test regs must be turned off during receive mode, also OCP must be turned on during receive mode.
  _dbm = dbm;
  _pa_mode = PA_modes;
  _ocp = OCP;
}

void RFM69LPL::rxBegin(){ 
  pinMode(_interruptPin, INPUT);
  setMode(RF69OOK_MODE_RX); //put in receive mode
}

void RFM69LPL::txBegin(){ 
  pinMode(_interruptPin, OUTPUT);
  setMode(RF69OOK_MODE_TX); //put in transmit mode
}

bool RFM69LPL::poll(){
  return digitalRead(_interruptPin);
}

void RFM69LPL::send(bool signal){
  digitalWrite(_interruptPin, signal);
}

uint32_t RFM69LPL::getFrequency(){ 
  // return the frequency (in Hz) by transforming to uint32_t and shifting the 3 bytes accordingly
  return RF69OOK_FSTEP * (((uint32_t)readReg(REG_FRFMSB)<<16) + ((uint16_t)readReg(REG_FRFMID)<<8) + readReg(REG_FRFLSB)); 
}

void RFM69LPL::setFrequencyMHz(float f){
  // Set literal frequency using floating point MHz value
  setFrequency(f * 1000000);
  _frequency = f;
}

void RFM69LPL::setModulationType(uint8_t mod){
  // 
  if(mod == MOD_OOK){
    writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00);
  }
  else if(mod == MOD_FSK){
    writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
  }

  _modulation = mod;
}

void RFM69LPL::setFrequency(uint32_t freqHz){
  // set the frequency (in Hz)
  // TODO: p38 hopping sequence may need to be followed in some cases
  freqHz /= RF69OOK_FSTEP; // divide down by FSTEP to get FRF
  writeReg(REG_FRFMSB, freqHz >> 16);
  writeReg(REG_FRFMID, freqHz >> 8);
  writeReg(REG_FRFLSB, freqHz);
}

void RFM69LPL::setLNAGain(byte lna_gain){
	writeReg(REG_LNA, RF_LNA_ZIN_50 | lna_gain);
  _lna_gain = lna_gain;
}

void RFM69LPL::setBandwidth(uint8_t bw){
  // set OOK/FSK bandwidth
  writeReg(REG_RXBW, readReg(REG_RXBW) & 0xE0 | bw);
  _bandwidth = bw;
}

void RFM69LPL::setRSSIThreshold(byte rssi){ // RSSI threshold in dBm = -(REG_RSSITHRESH / 2) [this function's argument in not in dBm]
  // set RSSI threshold
  //writeReg(REG_RSSITHRESH, (-rssi << 1));
  writeReg(REG_RSSITHRESH, rssi);
  _rssi_threshold = rssi;
}

void RFM69LPL::setFixedThreshold(uint8_t threshold){
  // set OOK fixed threshold
  writeReg(REG_OOKFIX, threshold);
  _fixed_threshold = threshold;
}

void RFM69LPL::setSensitivityBoost(bool sensitivity_boost){
  /*High sensitivity or normal sensitivity mode:
0x1B → Normal mode
0x2D → High sensitivity mode*/
  if (sensitivity_boost)
    writeReg(REG_TESTLNA, 0x2D);
  else
    writeReg(REG_TESTLNA, 0x1B);

  _sensitivity_boost = sensitivity_boost;
}

void RFM69LPL::setMode(byte newMode){
    if (newMode == _mode) return;

    switch (newMode) {
        case RF69OOK_MODE_TX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            break;
        case RF69OOK_MODE_RX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            writeReg(REG_TESTPA1, 0x55); //deactivate transmitter high power regs is necessary for receiving
            writeReg(REG_TESTPA2, 0x70);
            writeReg(REG_OCP, RF_OCP_ON); //OCP is necessary for receiving
            break;
        case RF69OOK_MODE_SYNTH:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69OOK_MODE_STANDBY:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69OOK_MODE_SLEEP:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default: return;
    }

    // waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
    while (_mode == RF69OOK_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

    _mode = newMode;
}

int8_t RFM69LPL::readRSSI(bool forceTrigger) {
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // Wait for RSSI_Ready
  }
  return -(readReg(REG_RSSIVALUE) >> 1);
}

byte RFM69LPL::readReg(byte addr){
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

void RFM69LPL::setFrequencyDev(uint32_t deviation){
  writeReg(REG_FDEVMSB, (deviation/61) >> 8);
  writeReg(REG_FDEVLSB, (deviation/61));
}

void RFM69LPL::writeReg(byte addr, byte value){
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

void RFM69LPL::select() {
  // Select the transceiver
  // set RFM69 SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); //decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  digitalWrite(_slaveSelectPin, LOW);
}

void RFM69LPL::unselect() {
  // UNselect the transceiver chip
  digitalWrite(_slaveSelectPin, HIGH);
}

void RFM69LPL::readAllRegs(){
  // for debugging
  byte regVal;
  for (byte regAddr = 1; regAddr <= 0x4F; regAddr++) {
    regVal = readReg(regAddr);
    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal,HEX);
    Serial.print(" - ");
    Serial.println(regVal,BIN);
  }
}

void RFM69LPL::readAllSettings() {
  
}