/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#include <Arduino.h>
#include <ESP8266WiFi.h>

typedef struct{
	int32_t KN;
	int32_t KD;
	int64_t C;
} CalibCoef_t;

typedef struct{
	uint32_t Value;
	uint16_t RowADC;
	uint16_t RowDAC;
} CalibRecord_t;

typedef struct{
	CalibRecord_t Uin1;
	CalibRecord_t Uin2;
	CalibRecord_t Uout1;
	CalibRecord_t Uout2;
	CalibRecord_t Iout1;
	CalibRecord_t Iout2;
	CalibRecord_t Tmp2;
	CalibRecord_t Tmp1; //  !!!   Последовательность не менять !!!!!   в EEPROM на месте Tmp1 находится CRC16
} CalibData_t;


typedef struct {
	uint16_t USET; //  always format 00.00
	uint16_t ISET; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t SOVP; //  always format 00.00
	uint16_t SOCP; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t SOPP;  //  always format 0000.0
	uint16_t BLED;
	uint16_t PRFS; 	// Set of parameters      before ver 4.5 SOFT Soft Front //MPRE;	// Memory Preset Number
	uint16_t SINI;	// Power output switch
	uint16_t OTIM;	// overtime
	uint16_t d09;
	uint16_t d10;
	uint16_t d11;
	uint16_t d12;
	uint16_t d13;
	uint16_t d14;
	uint16_t d15;
} Profile_t;

typedef enum {
	calib_mode_none = 0,
	calib_mode_start,	// Start DoCalibration()
	calib_mode_fix, 	// Applay Value
	calib_mode_save,	// exit & save
	calib_mode_exit 	// exit
} calib_mode_t;


//  Bitmask for gData.STATE
#define STA_ANA_TEMP_SENSOR 0x01   // Analog temperature sensor in the model
#define STA_DIG_TEMP_SENSOR 0x02   // Digital temperature sensor LM75 found at startup
#define STA_GYRO_SENSOR 	0x04   // Optional MPU6050 gyro sensor found at startup
#define STA_CUR_DIV			0x08	// Current divider is 10
#define STA_MODEL_WUZHI		0x10	//This is WUZHI

//  Bitmask for gData.PARAM
#define PARAM_C_OR_F  0x01  		// hex for 0000 0001
#define PARAM_RESET_COUNTERS  0x02  // hex for 0000 0010
#define PARAM_SMART_DISPLAY   0x04  // hex for 0000 0100
#define PARAM_BATTERY_SOURCE  0x08  // hex for 0000 1000
#define PARAM_POWER_ON_START  0x10  // hex for 0001 0000
#define PARAM_LOCK_ON_START   0x20  // hex for 0010 0000
#define PARAM_SLEEP_ON_START  0x40	// hex for 0100 0000
#define PARAM_TURN_OFF_FILTER 0x80  //  turn off filter for Uout, Iout, Uin

// Bitmask for gData.GYRO
#define GYRO_ROTATE  0x01  		// hex for 0000 0001   rotation is on
#define GYRO_DATA  0xFFF0  		// XYZ-rotaion

// Bitmask for gData.COMM
#define COMM_BAUD  0x07  		// hex for 0000 0111   baudrate 0,1,2,3,5,6
#define COMM_ONLINE  0x18  		// hex for 0001 1000	no, usb, bt1, bt2, wifi
#define COMM_DEBUG  0x40  		// hex for 0100 0000	wifi reset / BT debug
#define COMM_PROTOCOL  0x80  		// hex for 1000 0000	0-DPS, 1- WUZHI
#define COMM_ADDR  0x7F00 		// hex for 0111 1111 0000 0000

// Bitmask for gData.Profile[].PRFS
#define PROFILE_SOFT_FRONT 0x1
#define PROFILE_TRIGGER_OVP 0x2
#define PROFILE_TRIGGER_OCP 0x4

typedef struct {
	uint16_t USET; //  always format 00.00
	uint16_t ISET; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t UOUT; //  always format 00.00
	uint16_t IOUT; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t POWER; //  always format 0000.0 // PC apps  dont use it //  it is used for OPP
	uint16_t UIN; //  always format 00.00
	uint16_t LOCK;
	uint16_t PROTECT;
	uint16_t CVCC;
	uint16_t ONOFF;
	uint16_t BLED;   // Хранится в EEPROM вместe c MMAX
	uint16_t MODEL;
	uint16_t VERSION;
	uint16_t TMP; // temperature
	uint16_t STATE; //Set of bites before ver 4.2 SPCN; // Source percent
	uint16_t DEBUG_DATA; //d16;

	uint16_t MGIC; // Magic key
	uint16_t DVID; // Model, device ID
	uint16_t COMM; // online-offline + Baud Rate + Modbus Address
	uint16_t GYRO; // 0xFFF0 - XYZ-rotaion / 0x000F bit 1 means Rotation on, bit 2 means Gyro sensor present only for Modbus !!!
	uint16_t MMAX; // Max memory number   //   Хранится в EEPROM вместe c BLED
	uint16_t PVER; // Protocol version //d22, 	// before ver 4.2 RCNT; // Reset counters
	uint16_t BCKL; 	// Backlight from BLED // before ver 4.2  C or F
	uint16_t OHP;  // overheat temperature
	uint16_t d25; 	// before ver 4.2 SMART; // Smart display
	uint16_t PARAM; // Set of parameters      before ver 4.2 BSRC; // battery source
	uint16_t MINS;	// minimum source battery 0%
	uint16_t MAXS;	// maximum source battary 100%
	uint16_t CLR1; //  PowerOff Color
	uint16_t CLR2;	// CV color
	uint16_t CLR3;	// CC color
	uint16_t BEEP;	// use as CRC16 in EEPROM

	uint16_t CMD;	// Jump to system bootloader
	uint16_t TIME_L;	// Time counter low word
	uint16_t TIME_H;	// Time counter high word
	uint16_t MEM;  // Memory  Number  for Chinese firmware
	uint16_t AHCNT_L;	// Amper/hour counter low word
	uint16_t AHCNT_H;	// Amper/hour counter high word
	uint16_t WHCNT_L;	// Watt/hour counter low word
	uint16_t WHCNT_H;	// Watt/hour counter high word
	uint16_t CLB_CMD;		//   Calibration Command  enumeration  calib_mode_t;
	uint16_t CLB_IDX;		// Calibration Index of operation for DoCalibration(uint8_t aIdx)
	uint16_t CLB_DATA_L;	// Calibration Data (uint32_t Value) Low Word for CalibRecord_t.Value
	uint16_t CLB_DATA_H;	// Calibration Data (uint32_t Value) High Word for CalibRecord_t.Value
	uint16_t d45;
	uint16_t d46;
	uint16_t d47;
	uint16_t d48;

	uint16_t Dummy[32];
	Profile_t Profile[20];
	CalibData_t Calib;
	CalibRecord_t Iout1_v10;
	CalibRecord_t Iout2_v10;
	CalibRecord_t Iout1_v15;
	CalibRecord_t Iout2_v15;
} CurrentSet_t;

#define DPS_VER 45U
#define DPS_PVER 45U // Protocol version
#define MAGIC_KEY 15014 //  XX Day - XX Month - X Year
#define SCLR1 0xFFFF  //  PowerOff Color   white
#define SCLR2 0x07E0	// CV color
#define SCLR3 0xB6FF	// CC color
#define MAX_TIM 5999
#define DEF_COMM 0x102	// default 102	//  10A - USB+9600bps   10E - USB+115200bps
#define DEF_GYRO 0x5550

#define DPS_MOD 5020
#define MAX_U  5000
#define MAX_I  2000
#define MIN_I 1
#define OVP  5100
#define OCP 2020
#define OCPC 202  // OCP for charging
#define OPP 9999	//  Max Power
#define NCR 200  //  Typical current
#define MAX_MEM 20
#define DEF_STATE STA_ANA_TEMP_SENSOR


CurrentSet_t gData = {
500, 	//uint16_t USET;
100,	//uint16_t ISET;
0,		//uint16_t UOUT;
0,		//uint16_t IOUT;
0,		//uint16_t POWER;
3600,		//uint16_t UIN;
0,		//uint16_t LOCK;
0,		//uint16_t PROTECT;
0,		//uint16_t CVCC;
0,		//uint16_t ONOFF;
4,		//uint16_t BLED;
DPS_MOD,//uint16_t MODEL;
DPS_VER,//uint16_t VERSION;
20,		//uint16_t TMP; // temperature
DEF_STATE,		//uint16_t STATE; //Set of bites before ver 4.2 SPCN; // Source percent
0,		//uint16_t DEBUG_DATA;//d16;

MAGIC_KEY,	//uint16_t MGIC; // Magic number
DPS_MOD,//uint16_t DVID; // Model, device ID
DEF_COMM,	//uint16_t COMM; // online-offline(8=on) + Baud Rate(2=9600) + Modbus Address(0x0100 = 1)
DEF_GYRO,	//uint16_t GYRO; // XYZ-axis
MAX_MEM,//uint16_t MMAX; // Max Memory number
DPS_PVER, 		//uint16_t PVER; 	//  before ver 4.2 RCNT; // Reset counters
4,		//uint16_t BCKL; // Backlight from BLED // before ver 4.2  C or F
65,		//uint16_t OHP;  // overheat temperature
0,		//uint16_t d25; // before ver 4.2 SMART; // Smart display
PARAM_SMART_DISPLAY,	//uint16_t PARAM; // Set of parameters      before ver 4.2 BSRC; // battery source
3000,	//uint16_t MINS;	// minimum source battery 0%
4200,	//uint16_t MAXS;	// maximum source battary 100%
SCLR1,	//uint16_t CLR1; //  PowerOff Color   white
SCLR2,	//uint16_t CLR2;	// CV color
SCLR3,	//uint16_t CLR3;	// CC color
0,		//uint16_t BEEP;
0, 		//int16_t CMD;	// Command register
0,		//uint16_t TIME_L;	// Time counter low byte
0, 		//uint16_t TIME_H;	// Time counter high byte
0,		//uint16_t MEM;  // Memory  Number  for Chinese firmware
0,		//uint16_t AHCNT_L;	// Amper/hour counter low byte
0,		//uint16_t AHCNT_H;	// Amper/hour counter high byte
0,		//uint16_t WHCNT_L;	// Watt/hour counter low byte
0,		//uint16_t WHCNT_H;	// Watt/hour counter high byte
0,		//uint16_t CLB_CMD;		//   Calibration Command  enumeration  calib_mode_t;
0,		//uint16_t CLB_IDX;		// Calibration Index of operation for DoCalibration(uint8_t aIdx)
0,		//uint16_t CLB_DATA_L;	// Calibration Data (uint32_t Value) Low Word for CalibRecord_t.Value
0,		//uint16_t CLB_DATA_H;	// Calibration Data (uint32_t Value) High Word for CalibRecord_t.Value
0,		//uint16_t d12;
0,		//uint16_t d13;
0,		//uint16_t d14;
0,		//uint16_t d15;
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//uint16_t Dummy[32];
//		Profile_t Profile[20];
//	USET	ISET	SOVP	SOCP	SOPP	BLED	PRFS		SINI	OTIM
{	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	OVP,	OCP,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
//  --------------  battery charge  --------------------
	{420,	NCR,	420,	OCPC,	85,		5,		0,			0,		300,		0,0,0,0,0,0,0},
	{840,	NCR,	840,	OCPC,	170,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{1260,	NCR,	1260,	OCPC,	255,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{1680,	NCR,	1680,	OCPC,	340,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{2100,	NCR,	2100,	OCPC,	425,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{2520,	NCR,	2520,	OCPC,	510,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{2940,	NCR,	2940,	OCPC,	595,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{3360,	NCR,	3360,	OCPC,	680,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{3780,	NCR,	3780,	OCPC,	765,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{4200,	NCR,	4200,	OCPC,	850,	5,		0,			0,		300,		0,0,0,0,0,0,0} },
//  ---------------- Calibration Data -----------------
	{{5270, 4978, 0},		// Uin1
	{31010, 29432, 0},		// Uin2
	{1010, 1417, 1328},		// Uout1
	{34990, 42601, 42528},	// Uout2
	{102, 1387, 2080},		// Iout1
	{4999, 12848, 14864},	// Iout2
	{60, 16000,0},			// Tmp2
	{29, 27728,0}},			// Tmp1
//  ---------------- Old HW Version Calib Data
	{103, 876, 5000},		// Iout1_v10
	{4997, 11855, 15360},	// Iout2_v10
	{103, 876, 3008},		// Iout1_v15
	{4997, 11855, 15360}	// Iout2_v15
	};

// modbus defines
#define MB_PORT 502
#define MB_BUFFER_SIZE 256
#define MB_RTU_SLAVE_RESPONSE_TIMEOUT_MS 1000

#define  MB_FC_NONE                     0
#define  MB_FC_READ_REGISTERS           3
#define  MB_FC_WRITE_REGISTER           6
#define  MB_FC_WRITE_MULTIPLE_REGISTERS 16
#define  MB_FC_ERROR_MASK 				128

// uncomment the line below if you need debug via serial
//#define MB_DEBUG

WiFiServer mbServer(MB_PORT);
WiFiClient client;
uint8_t mbByteArray[MB_BUFFER_SIZE]; // send and recieve buffer

/*=================================================================================================*/
void mb_debug(String str, uint16_t start, uint16_t len)
{
#ifdef MB_DEBUG
	Serial.print(str);
	for (uint16_t i = 0; i < len; i++) 
	{
		Serial.printf(" %X", mbByteArray[i + start]);
	}
	Serial.println("");
#endif
}

/*=================================================================================================*/
void modbusTcpServerTask(void)
{
//  uint8_t i;
  uint8_t mb_func = MB_FC_NONE;
  uint16_t * mbData = (uint16_t *)&gData;
  uint16_t start, wordDataLength, byteDataLength, messageLength = 0;
  
	if (mbServer.hasClient())
	{
		// if client is free - connect
		if (!client || !client.connected()){
			if(client) client.stop();
			client = mbServer.accept(); //.available();
			
		// client is not free - reject
		} 
		else 
		{
			WiFiClient serverClient = mbServer.accept(); //.available();
			serverClient.stop();
		}
	}

  	//-------------------- Read from socket --------------------
  	if (client && client.connected() && client.available())
  	{
		delay(1);
		uint16_t bytesReady;
		while((bytesReady = client.available()) && (messageLength < MB_BUFFER_SIZE))
		{
			messageLength += client.readBytes(&mbByteArray[messageLength], bytesReady);
		}
		if (messageLength > 8)  mb_func = mbByteArray[7];  //Byte 7 of request is FC
		mbByteArray[4] = 0;
		mb_debug("TCP RX:", 0, messageLength);
	}
	else 
	{
		return;
	}	
        //-------------------- Read Registers (3 & 4) --------------------
    if(mb_func == MB_FC_READ_REGISTERS) {
		start = word(mbByteArray[8],mbByteArray[9]);
		wordDataLength = word(mbByteArray[10],mbByteArray[11]);
    	byteDataLength = wordDataLength * 2;
    	mbByteArray[5] = byteDataLength + 3; 
    	mbByteArray[8] = byteDataLength;   
        for(int i = 0; i < wordDataLength; i++)
        {
            mbByteArray[9 + i * 2] = highByte(mbData[start + i]);
            mbByteArray[10 + i * 2] =  lowByte(mbData[start + i]);
        }
        messageLength = byteDataLength + 9;
		if (client && client.connected()) 
		{
			client.write((uint8_t*)mbByteArray, messageLength);
			client.flush();
			delay(1);
			mb_debug("TCP TX:", 0, messageLength);
		}
    }
        //-------------------- Write Register (6) --------------------
    else if(mb_func == MB_FC_WRITE_REGISTER) 
	{
		start = word(mbByteArray[8],mbByteArray[9]);
		mbData[start] = word(mbByteArray[10],mbByteArray[11]);
		mbByteArray[5] = 6; 
		messageLength = 12;
		if (client && client.connected()) 
		{
			client.write((uint8_t*)mbByteArray, messageLength);
			client.flush();
			delay(1);
			mb_debug("TCP TX:", 0, messageLength);
		}
	}
		//-------------------- Write Multiple Registers (16) --------------------
	else if(mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) 
	{
		start = word(mbByteArray[8],mbByteArray[9]);
		wordDataLength = word(mbByteArray[10],mbByteArray[11]);
		byteDataLength = wordDataLength * 2;
		mbByteArray[5] = 6;
		for(int i = 0; i < wordDataLength; i++)
		{
			mbData[start + i] =  word(mbByteArray[ 13 + i * 2],mbByteArray[14 + i * 2]);
		}
		messageLength = 12;
		if (client && client.connected()) 
		{
			client.write((uint8_t*)mbByteArray, messageLength);
			client.flush();
			mb_debug("TCP TX:", 0, messageLength);
		}
	}
}

/*=================================================================================================*/
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED off

  Serial.begin(115200);
  WiFi.begin("ssidname","password");  
   while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  mbServer.begin();
  mbServer.setNoDelay(true);
  digitalWrite(LED_BUILTIN, LOW); // LED 0n
}

/*=================================================================================================*/
void loop()
{
   	modbusTcpServerTask();

	static uint32_t timeCounter = 0;
   	static uint32_t lastTime = 0;
	if (gData.ONOFF) // Output on
	{
		if (gData.UOUT < gData.USET) gData.UOUT += gData.USET / 20;
		if (gData.IOUT < gData.ISET / 2) gData.IOUT += gData.ISET / 20;
		uint32_t curTime = millis();
		if ( (curTime - lastTime) >= 1000) // one second timer
		{
			timeCounter++;
			lastTime = curTime;
			gData.TIME_L = timeCounter & 0xFFFF;
			gData.TIME_H = timeCounter >> 16;	
		}
	}
	else
	{
		if (gData.UOUT > 0) gData.UOUT--;
		if (gData.IOUT > 0) gData.IOUT--;
	}
#ifdef MB_DEBUG
	delay(10);
#endif
}
/*=================================================================================================*/

