#ifndef logger_h //load header if we have not defined RFIDuino_h
#define logger_h //define RFIDuino_h, to prevent code being loaded if this library is loaded a second time
#include "Arduino.h"
#define DELAYVAL    320   //384 //standard delay for manchster decode
#define TIMEOUT     1000  //standard timeout for manchester decode


//
//LOGGER
//
class logger{
	public:

		logger();
        logger (int idemod,int imod,int irdyclk);
		//
		//void boot();
		//low level deocde functions
		bool decodeTag(unsigned char *buf);	//low level tag decode
		void transferToBuffer(byte *tagData, byte *tagDataBuffer);	//transfer data from one array to another
		bool compareTagData(byte *tagData1, byte *tagData2);			//compate 2 arrays

		//higher level tag scanning / reporting
		bool scanForTag(byte *tagData);
		void load_settings();
		void set_setting(String setting, String value);
		void boot();
		int get_RFID_READ_FREQ(){ 
			return RFID_READ_FREQ;
		}
		void capture_command();
		String time_stamp_string();
		//user output pin values, variables as they will change depending on which hardware is used
	private:
		//pins to connect Arduino to the EM4095 chip, variables as they will change depending on which hardware is used
		int demodOut;
		int shd;
		int mod;
		int rdyClk;

		//real time clock
		//clock rtc;

		//settings and their default values
		int BOOT_TO_MODE = 0;
		int CURRENT_MODE = 0;
		String POWER_SCHEDULE = "00:00-24:00";
		bool JSON_RECORD = true;
		bool CSV_RECORD = true;
		int RFID_READ_FREQ = 200;
		int LOW_BATTERY = 500;
		int LONGITUDE = -80.60301463292694;
		int LATITUDE = 28.60739886098215;
		String FILE_PREFIX = "datalog";
		int HOURS_PER_FILE = 24;
		int COMMAND_TIMEOUT = 500;
};
#endif



