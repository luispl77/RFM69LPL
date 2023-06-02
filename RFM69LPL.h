#ifndef RFM69LPL_h
#define RFM69LPL_h
#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater
#include <RFM69LPLregisters.h>

#define RF69OOK_SPI_CS  SS // SS is the SPI slave select pin, for instance D10 on atmega328

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on Atmega328 it's D2, on Atmega644/1284 it's D2)
#define RF69OOK_MODE_SLEEP       0 // XTAL OFF
#define RF69OOK_MODE_STANDBY     1 // XTAL ON
#define RF69OOK_MODE_SYNTH       2 // PLL ON
#define RF69OOK_MODE_RX          3 // RX MODE
#define RF69OOK_MODE_TX          4 // TX MODE

#define MOD_OOK 1 //on off keying modulation
#define MOD_FSK 0 //frequency shit keying modulation

#define PA_MODE_PA0 1
#define PA_MODE_PA1 2
#define PA_MODE_PA1_PA2 3
#define PA_MODE_PA1_PA2_20dbm 4

#define OCP_ON 1
#define OCP_OFF 0

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69OOK_FSTEP 61.03515625 // == FXOSC/2^19 = 32mhz/2^19 (p13 in DS)

class RFM69LPL {
  public:

    RFM69LPL(byte slaveSelectPin, byte interruptPin) { //constructor for default values

      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _mode = RF69OOK_MODE_STANDBY;
      _dbm = 20;
	  	_fixed_threshold = 115; 
	  	_bandwidth = OOK_BW_100_0; 
	  	_frequency = 433.920; 
	  	_thresh_type_fixed = false;
	  	_rssi_threshold = 255;
	  	_lna_gain = RF_LNA_GAINSELECT_AUTO; 
	  	_pa_mode = PA_MODE_PA1_PA2_20dbm;
	  	_ocp = OCP_OFF;
	  	_modulation = MOD_OOK;
	  	_sensitivity_boost = false;
	  	_deviation = 5002; //default value 5khz
	  }


	  void init();
	  void updateSettings();
		//common functions
		void setFrequency(uint32_t freqHz);
    void setFrequencyMHz(float f);
    void setFrequencyDev(uint32_t deviation);
    void setModulationType(uint8_t mod);
		float getFrequency();
		byte getBandwidthIndex();
		void setMode(byte mode);
		void standby();
		uint16_t getFrequencyDev();
	
		//transmitter functions
  	void txBegin();
		void setTransmitPower(byte dbm, int PA_modes, int OCP); //also puts in TX mode
		void send(bool signal);
	
	
		//receiver functions
  	void rxBegin();
		void threshTypeFixed(bool fixed);
		void setLNAGain(byte lna_gain);
 	 	int8_t readRSSI(bool forceTrigger=false);
		void setBandwidth(uint8_t bw);
		void setRSSIThreshold(uint8_t rssi);
		void setFixedThreshold(uint8_t threshold);
		void setSensitivityBoost(bool sensitivity_boost);
		bool poll();
	
	
 	 	// SPI register writing and reading functions
  	byte readReg(byte addr);
  	void writeReg(byte addr, byte val);
  	void readAllRegs();
  	void select();
  	void unselect();
  	void readAllSettings();


  	byte _mode;
  	byte _slaveSelectPin;
    byte _interruptPin;
 	  byte _dbm;
	  byte _fixed_threshold;
		byte _rssi_threshold;
		byte _bandwidth;
		byte _lna_gain;
		byte _pa_mode;
		byte _ocp;
		byte _modulation;
		bool _sensitivity_boost;
		bool _thresh_type_fixed;
		float _frequency;
		uint32_t _deviation;

	
	
  	protected:

  	//not needed
    
};

#endif
