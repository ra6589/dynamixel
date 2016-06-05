/*
 * edited by REZA AHMADI
 * 2016.06.05
 */
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include <stddef.h>
#include <vector>
#include "serialport.h"
typedef  unsigned char byte;
typedef  unsigned short word;
typedef  unsigned int dword;
#define MAXNUM_TXPACKET     		(65535)
#define MAXNUM_RXPACKET     		(65535)
#define BROADCAST_ID						(0xFE)
#define BAUDRATE								(57142)

#define LATENCY_TIME						(10)		// ms (USB2Serial Latency timer)

#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

/*----------------------------EEPROM-----------------------------*/
#define DPRO_MODELNUMBER_LB									(0)
#define DPRO_MODELINFORMATION_LW						(2)
#define DPRO_VERSIONOFFIRMWARE							(6)
#define DPRO_ID															(7)
#define DPRO_BAUDRATE												(8)
#define DPRO_RETURNDELAYTIME								(9)
#define DPRO_OPERATINGMODE									(11)
#define DPRO_HOMINGOFFSET_LW						  	(13)
#define DPRO_MOVINGTHRESHOLD_LW							(17)
#define DPRO_TEMPERATURELIMIT               (21)
#define DPRO_MAXVOLTAGELIMIT_LB        	    (22)
#define DPRO_MINVOLTAGELIMIT_LB							(24)
#define DPRO_ACCELERATIONLIMIT_LW						(26)
#define DPRO_TORQUELIMIT_LB									(30)
#define DPRO_VELOCITYLIMIT_LW								(32)
#define DPRO_MAXPOSITIONLIMIT_LW						(36)
#define DPRO_MINPOSITIONLIMIT_LW						(40)
#define DPRO_EXTERNALPORTMODE(i)						({44,45,46,47})
#define DPRO_SHUTDOWN												(48)
#define DPRO_INDIRECTADDRESS(i)							(47+2*i)
/*----------------------------RAM-------------------------------*/
#define DPRO_TORQUEENABLE										(562)
#define DPRO_LEDRED													(563)
#define DPRO_LEDGREEN												(564)
#define DPRO_LEDBLUE												(565)
#define DPRO_VELOCITYIGAIN_LB								(586)
#define DPRO_VELOCITYPGAIN_LB								(588)
#define DPRO_POSITIONPGAIN_LB								(594)
#define DPRO_GOALPOSITION_LW								(596)
#define DPRO_GOALVELOCITY_LW								(600)
#define DPRO_GOALTORQUE_LB									(604)
#define DPRO_GOALACCELERATION_LW  					(606)
#define DPRO_MOVING													(610)
#define DPRO_PRESENTPOSITION_LW							(611)
#define DPRO_PRESENTVELOCITY_LW							(615)
#define DPRO_PRESENTCURRENT_LB							(621)
#define DPRO_PRESENTINPUTVOLTAGE_LB					(623)
#define DPRO_PRESENTTEMPERATURE							(625)
#define DPRO_EXTERNALPORTDATA_LB(i)					(625+i)
#define DPRO_INDIRECTDATA(i)								(633+i)
#define DPRO_REGISTEREDINSTRUCTION 					(890)
#define DPRO_STATUSRETURNLEVEL							(891)
#define DPRO_HARDWAREERRORSTATUS_LB					(892)


	enum {
		PKT_HEADER0=0,
		PKT_HEADER1,
		PKT_HEADER2,
		PKT_RESERVED,
		PKT_ID,
		PKT_LENGTH_L,
		PKT_LENGTH_H,
		PKT_INSTRUCTION,
		PKT_PARAMETER
	};
	
	enum{
		READ,
		WRITE};
		
	enum {
		COMM_TXSUCCESS,
		COMM_RXSUCCESS,
		COMM_TXFAIL,
		COMM_RXFAIL,
		COMM_TXERROR,
		COMM_RXWAITING,
		COMM_RXTIMEOUT,
		COMM_RXCORRUPT
	};

	enum {
		INST_PING			= 1,
		INST_READ			= 2,
		INST_WRITE			= 3,
		INST_REG_WRITE		= 4,
		INST_ACTION			= 5,
		INST_FACTORY_RESET	= 6,
		INST_REBOOT			= 8,
		INST_SYSTEM_WRITE	= 13,	// 0x0D
		INST_STATUS			= 85,	// 0x55
		INST_SYNC_READ		= 130,	// 0x82
		INST_SYNC_WRITE		= 131,	// 0x83
		INST_BULK_READ		= 146,	// 0x92
		INST_BULK_WRITE		= 147	// 0x93
	};

	enum {
		ERRBIT_VOLTAGE		= 1,
		ERRBIT_ANGLE		= 2,
		ERRBIT_OVERHEAT		= 4,
		ERRBIT_RANGE		= 8,
		ERRBIT_CHECKSUM		= 16,
		ERRBIT_OVERLOAD		= 32,
		ERRBIT_INSTRUCTION	= 64
	};

	class PingInfo
	{
	public:
		int ID;
		int ModelNumber;
		int FirmwareVersion;

		PingInfo() : ID(-1), ModelNumber(-1), FirmwareVersion(-1) { };
	};

	class BulkReadData
	{
	public:
	    int             iID;
	    int             iStartAddr;
	    int             iLength;
	    int             iError;
	    unsigned char*  pucTable;

	    BulkReadData()
	    {
			iID = 0; iStartAddr = 0; iLength = 0; iError = 0;
			pucTable = 0;

	    }
		~BulkReadData()
		{
//			if(pucTable != 0)
//				delete[] pucTable;
		}
	};


	class Dynamixel
	{
		public:
			SerialPort *ComPort;

			Dynamixel(unsigned char port);
			~Dynamixel();

			void SetPacketTimeout(int packet_len);
			void SetPacketTimeout(double msec);
			bool IsPacketTimeout();
			double GetPacketTime();
			int Ping(byte id, byte *error);
			int Ping(byte id, PingInfo *info, byte *error);
			int BroadcastPing(std::vector<PingInfo>& vec_info);
			int Reboot(byte id, byte *error);
			int FactoryReset(byte id, int option, byte *error);

			int Read(byte id, int address, int length, unsigned char* data, byte *error);
			int ReadByte(byte id, int address, byte *value, byte *error);
			int ReadWord(byte id, int address, word *value, byte *error);
			int ReadDWord(byte id, int address, dword *value, byte *error);

			int Write(byte id, int address, int length, unsigned char* data, byte *error);
			int WriteByte(byte id, int address, byte value, byte *error);
			int WriteWord(byte id, int address, word value, byte *error);
			int WriteDWord(byte id, int address, long value, byte *error);

			int SyncWrite(int start_addr, int data_length, unsigned char* param, int param_length);
			int BulkRead(std::vector<BulkReadData>& data);
			//high level func
			word getModelNumber(byte id, byte *error);
			byte getVersion(byte id, byte *error);
			void ID_RW(byte id,byte &ID,byte rw_mode, byte *error);
			void BaudRate(byte id,byte &baudrate, byte rw_mode, byte *error);
			void ReturnDelayTime(byte id,byte& value,byte rw_mode , byte *error);
			void LimitTemperature(byte id,byte &limit, byte rw_mode, byte *error);
			void DownLimitVoltage(byte id,word &limit, byte rw_mode, byte *error);
			void UpLimitVoltage(byte id,word &limit, byte rw_mode, byte *error);
			void MaxTorque(byte id,word &limit, byte rw_mode, byte *error);
			void StatusReturnLevel(byte id,byte &value, byte rw_mode, byte *error);
			void AlarmLed(byte id,byte &value, byte rw_mode, byte *error);
			void AlarmShutdown(byte id,byte& value, byte rw_mode, byte *error);
        //ram area
			void TorqueEnable(byte id,byte &value, byte rw_mode, byte *error);
			void LedBlue(byte id,byte &value, byte rw_mode, byte *error);
			void LedGreen(byte id,byte &value, byte rw_mode, byte *error);
			void LedRed(byte id,byte &value, byte rw_mode, byte *error);
			void GoalPosition(byte id,dword &value, byte rw_mode, byte *error);
			void GoalSpeed(byte id,dword &value, byte rw_mode, byte *error);
			void TorqueLimit(byte id,word &limit, byte rw_mode, byte *error);
			dword getPresentPosition(byte id, byte *error);
			dword getPresentSpeed(byte id, byte *error);
			word getPresentCurrent(byte id, byte *error);
			byte getPresentVoltage(byte id, byte *error);
			byte getPresentTemperature(byte id, byte *error);
			void RegisteredInstruction(byte id,byte &value, byte rw_mode, byte *error);
			byte getMoving(byte id, byte *error);
		private:
			unsigned short UpdateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
			void AddStuffing(unsigned char *packet);
			void RemoveStuffing(unsigned char *packet);
			
			int TxPacket(unsigned char *txpacket);
			int RxPacket(unsigned char *rxpacket, word packetlength);
			double ByteTransferTime;
			double PacketStartTime;
			double PacketWaitTime;
			int TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, byte *error);
	};



#endif /* DYNAMIXEL_H_ */
