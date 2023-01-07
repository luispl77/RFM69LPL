#include <RFM69LPL.h>
#include <RFM69LPLregisters.h>
#include <SPI.h>

volatile byte RFM69LPL::_mode;  // current transceiver state
volatile int RFM69LPL::RSSI; 	// most accurate RSSI during reception (closest to the reception)



void RFM69LPL::threshTypeFixed(bool fixed){
	if(fixed){
		writeReg(REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_FIXED | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000);
	}
	else{
		writeReg(REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000);
	}
}




void RFM69LPL::initializeTransmit(byte dbm, int PA_modes, int OCP) { //keep a minimum of -11 dbm to avoid writing negative values into the palevel reg.
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
}

void RFM69LPL::initializeReceive(){ 


  pinMode(_interruptPin, INPUT);
  
  setBandwidth(_bandwidth);
  setFixedThreshold(_fixed_threshold); 
  setFrequencyMHz(_frequency);
  threshTypeFixed(_thresh_type_fixed);
  setRSSIThreshold(_rssi_threshold);
  setLNAGain(_lna_gain);
  setModulationType(MOD_OOK);

  setMode(RF69OOK_MODE_RX); //put in receive mode
}

bool RFM69LPL::poll(){
  // Poll for OOK signal
  return digitalRead(_interruptPin);
}

void RFM69LPL::send(bool signal){
  // Send a 1 or 0 signal in OOK mode
  digitalWrite(_interruptPin, signal);
}

void RFM69LPL::transmitBegin(){
  // Turn the radio into transmission mode
  setMode(RF69OOK_MODE_TX);
  pinMode(_interruptPin, OUTPUT);
}

void RFM69LPL::transmitEnd(){
  // Turn the radio back to standby
  pinMode(_interruptPin, INPUT);
  setMode(RF69OOK_MODE_STANDBY);
}

void RFM69LPL::receiveBegin(){
  // Turn the radio into OOK listening mode
  pinMode(_interruptPin, INPUT);
  setMode(RF69OOK_MODE_RX);
}

void RFM69LPL::receiveEnd(){
  // Turn the radio back to standby
  setMode(RF69OOK_MODE_STANDBY);
}

uint32_t RFM69LPL::getFrequency(){ 
  // return the frequency (in Hz) by transforming to uint32_t and shifting the 3 bytes accordingly
  return RF69OOK_FSTEP * (((uint32_t)readReg(REG_FRFMSB)<<16) + ((uint16_t)readReg(REG_FRFMID)<<8) + readReg(REG_FRFLSB)); 
}

void RFM69LPL::setFrequencyMHz(float f){
  // Set literal frequency using floating point MHz value
  setFrequency(f * 1000000);
}

void RFM69LPL::setModulationType(uint8_t mod){
  // 
  if(mod == MOD_OOK){
    writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00);
  }
  else if(mod == MOD_FSK){
    writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
  }
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
}

void RFM69LPL::setBandwidth(uint8_t bw){
  // set OOK/FSK bandwidth
  writeReg(REG_RXBW, readReg(REG_RXBW) & 0xE0 | bw);
  _bandwidth = bw;
}

void RFM69LPL::setRSSIThreshold(uint8_t rssi){ // RSSI threshold in dBm = -(REG_RSSITHRESH / 2)
  // set RSSI threshold
  //writeReg(REG_RSSITHRESH, (-rssi << 1));
  writeReg(REG_RSSITHRESH, rssi);
  _rssi_threshold = rssi;
}

void RFM69LPL::setFixedThreshold(uint8_t threshold){
  // set OOK fixed threshold
  writeReg(REG_OOKFIX, threshold);
}

void RFM69LPL::setSensitivityBoost(uint8_t value){
  // set sensitivity boost in REG_TESTLNA
  // see: http://www.sevenwatt.com/main/rfm69-ook-dagc-sensitivity-boost-and-modulation-index
  writeReg(REG_TESTLNA, value);
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

void RFM69LPL::sleep() {
  //power saving sleep mode
  setMode(RF69OOK_MODE_SLEEP);
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
  if(_isReceiver) digitalWrite(5, HIGH); //pull transmitter high to avoid interference with receiver SPI
  else digitalWrite(4, HIGH); //pull receiver high to avoid interference with transmitter SPI
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
  if(_isReceiver) digitalWrite(5, HIGH); //pull transmitter high to avoid interference with receiver SPI
  else digitalWrite(4, HIGH); //pull receiver high to avoid interference with transmitter SPI
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

void RFM69LPL::setHighPowerRegs(bool onOff) {
  //PA regs
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
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

byte RFM69LPL::readTemperature(byte calFactor){
  // returns centigrade
  setMode(RF69OOK_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
}                                                            // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void RFM69LPL::rcCalibration(){
  //RC calibration mode
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}
