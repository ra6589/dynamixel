/*
 * edited by REZA AHMADI
 * 2016.06.05
 */

//#include <stdio.h>	//TODO: TEST CODE
#include <cstdlib>
#include <float.h>
#include <lpc17xx.h>
#include <string>
#include "dynamixel.h"
#include "serialport.h"
//using namespace DXL_PRO;
//_init_alloc(base, , top);

/*must Setup and enable the SysTick timer for 100ms in this source code or in main*/
///*_____________________________microinit_______________________*/
//static void microinit () {
//  /* Add System initialisation code here */


//  /* Setup and enable the SysTick timer for 100ms. */
//  SysTick->LOAD = (SystemCoreClock / 10) - 1;
//  SysTick->CTRL = 0x05;
//}
/*_____________________________Dynamixel_______________________*/
Dynamixel::Dynamixel(unsigned char port)
{
//	microinit ();
	ComPort = new SerialPort(port);
}
/*_____________________________~Dynamixel_______________________*/
Dynamixel::~Dynamixel()
{
	
}
/*_______________________________SetPacketTimeout_____________________*/
void Dynamixel::SetPacketTimeout(int packet_len)
{
	PacketStartTime =SysTick->CTRL/65531.0*100;
	ByteTransferTime = (1000.0 / (double)BAUDRATE) * 10.0;
	PacketWaitTime = (ByteTransferTime * (double)packet_len) + (2.0 * (double)LATENCY_TIME) + 2.0;
}
/*______________________________SetPacketTimeout______________________*/

void Dynamixel::SetPacketTimeout(double msec)
{
	PacketStartTime = SysTick->CTRL/65531.0*100;
	PacketWaitTime = msec;
}
/*______________________________IsPacketTimeout______________________*/
bool Dynamixel::IsPacketTimeout()
{
	if(GetPacketTime() > PacketWaitTime)
		return true;
	return false;
}
/*_____________________________GetPacketTime_______________________*/
double Dynamixel::GetPacketTime()
{
	double time;

	time = SysTick->CTRL/65531.0*100 - PacketStartTime;
	if(time < 0.0)
		PacketStartTime = SysTick->CTRL/65531.0*100;

	return time;
}
/*_____________________________UpdateCRC_______________________*/
unsigned short Dynamixel::UpdateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {0x0000,
	0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
	0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
	0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
	0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
	0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
	0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
	0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
	0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
	0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
	0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
	0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
	0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
	0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
	0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
	0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
	0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
	0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
	0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
	0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
	0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
	0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
	0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
	0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
	0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
	0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
	0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
	0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
	0x820D, 0x8207, 0x0202 };

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

/*___________________________TxPacket_________________________*/
int Dynamixel::TxPacket(unsigned char *txpacket)
{
	int packet_tx_len, real_tx_len;
	int length;
	unsigned short crc = 0;

	//TODO: check port using

	// Byte stuffing
	////AddStuffing(txpacket);
	length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]);

	// Check Max packet length
	if(length > MAXNUM_TXPACKET)
	{
		//TODO: port free?
		return COMM_TXERROR;
	}

	// Make Packet Header
	txpacket[PKT_HEADER0]	= 0xFF;
	txpacket[PKT_HEADER1]	= 0xFF;
	txpacket[PKT_HEADER2]	= 0xFD;
	txpacket[PKT_RESERVED]	= 0x00;	// reserved

	// Add CRC16
	crc = UpdateCRC(0, txpacket, length+PKT_LENGTH_H+1-2);	// -2 : CRC16
	txpacket[length+PKT_LENGTH_H-1] = DXL_LOBYTE(crc);		// last - 1
	txpacket[length+PKT_LENGTH_H-0] = DXL_HIBYTE(crc);		// last - 0

	// Tx Packet
	
	packet_tx_len = length + PKT_LENGTH_H + 1;
	//printf("txfail1\n");
	real_tx_len = ComPort->WritePort(txpacket, packet_tx_len);
	//printf("txfail2\n");
	if(packet_tx_len != real_tx_len)
	{
		//TODO: port free?
		return COMM_TXFAIL;
	}

	return COMM_TXSUCCESS;
}
/*___________________________RxPacket_________________________*/
int Dynamixel::RxPacket(unsigned char *rxpacket, word packetlength)
{
	int rx_length = 0, wait_length = PKT_LENGTH_H + 4 + 1+packetlength;	// 4 : INST ERROR CHKSUM_L CHKSUM_H
	int  result = COMM_RXFAIL;
	unsigned short crc = 0;
  ComPort->ReadPort(&rxpacket[rx_length], wait_length - rx_length);
	if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF && rxpacket[2] == 0xFD)
			crc = DXL_MAKEWORD(rxpacket[wait_length-2], rxpacket[wait_length-1]);
	else 
		return COMM_RXCORRUPT;
	if(UpdateCRC(0, rxpacket, wait_length-2) == crc) // -2 : except CRC16
					result = COMM_RXSUCCESS;
	else
					result = COMM_RXCORRUPT;
	
	return	result;
}
/*__________________________TxRxPacket__________________________*/
int Dynamixel::TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, byte *error)
{
	 word packetlength=0;
	int result = COMM_TXFAIL;

	//TODO: check bus idle?

	result = TxPacket(txpacket);

	// Check Tx packet result
	if(result != COMM_TXSUCCESS)
		return result;

	// Set Rx timeout
	if(txpacket[PKT_INSTRUCTION] == INST_READ)
	{
		SetPacketTimeout(DXL_MAKEWORD(txpacket[PKT_PARAMETER+2], txpacket[PKT_PARAMETER+3]) + 11);
		packetlength=DXL_MAKEWORD(txpacket[PKT_PARAMETER+2], txpacket[PKT_PARAMETER+3]);		
	}
	else
	{
		SetPacketTimeout(PKT_LENGTH_H + 4 + 1);	// 4 : INST ERROR CHKSUM_L CHKSUM_H
		packetlength=0;
	}

	// Broadcast ID && !BulkRead == no need to wait for a rxpacket
	if(txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ)
	{
		//TODO: bus free?
		return COMM_RXSUCCESS;
	}
   
	result = RxPacket(rxpacket, packetlength);
	if((result == COMM_RXSUCCESS) && (txpacket[PKT_ID] != BROADCAST_ID) && (txpacket[PKT_ID] != rxpacket[PKT_ID]))
		result = RxPacket(rxpacket,packetlength);

	if(result == COMM_RXSUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
	{
		if(error != 0)
			*error = (int)rxpacket[PKT_PARAMETER];
	}

	return result;
}
/*_____________________________BroadcastPing_______________________*/
int Dynamixel::BroadcastPing(std::vector<PingInfo>& vec_info)
{
	const int PING_STATUS_LENGTH = 14, MAX_ID = 252;
	int result = COMM_TXFAIL;
	int rx_length = 0, wait_length = 0,i=0;
	unsigned char txpacket[10]	= {0};
	unsigned char rxpacket[PING_STATUS_LENGTH * MAX_ID]	= {0};

	txpacket[PKT_ID]			= (unsigned char)BROADCAST_ID;
	txpacket[PKT_LENGTH_L]		= 0x03;
	txpacket[PKT_LENGTH_H]		= 0x00;
	txpacket[PKT_INSTRUCTION]	= INST_PING;

	result = TxPacket(txpacket);
	if(result != COMM_TXSUCCESS)
	{
		// TODO: Bus free?
		return result;
	}

	wait_length = PING_STATUS_LENGTH * MAX_ID;

	// Set Rx Timeout
	SetPacketTimeout( (double)((ByteTransferTime * wait_length) + (3 * MAX_ID) + 2 * LATENCY_TIME) );

	while(1)
	{
		int _cnt = ComPort->ReadPort(&rxpacket[rx_length], wait_length - rx_length);
		if(_cnt > 0)
		{
			rx_length += _cnt;
			i++;
			if(i==10000000)
				break;
				
//			fprintf(stderr, "cnt: %d, Interval: %f / Wait time: %f \n", _cnt, GetPacketTime(), PacketWaitTime);
		}
		if((IsPacketTimeout() == 1) || (rx_length >= wait_length))
			break;
	}
	// TODO: Bus free?

	if(rx_length == 0)
		return COMM_RXTIMEOUT;

	while(1)
	{
		if(rx_length < PING_STATUS_LENGTH)
			return COMM_RXCORRUPT;

		// find packet header
		byte idx = 0;
		while(idx < (rx_length - 2))
		{
			if(rxpacket[idx] == 0xFF && rxpacket[idx + 1] == 0xFF && rxpacket[idx + 2] == 0xFD)
				break;
			else
				idx++;
		}

		if(idx == 0)
		{
			// check CRC16
			int crc = DXL_MAKEWORD(rxpacket[PING_STATUS_LENGTH - 2], rxpacket[PING_STATUS_LENGTH - 1]);
			if(UpdateCRC(0, rxpacket, PING_STATUS_LENGTH - 2) == crc) // -2 : except CRC16
			{
				vec_info.push_back(PingInfo());
				vec_info.at(vec_info.size()-1).ID = rxpacket[PKT_ID];
				vec_info.at(vec_info.size()-1).ModelNumber = DXL_MAKEWORD(rxpacket[PKT_PARAMETER+1], rxpacket[PKT_PARAMETER+2]);
				vec_info.at(vec_info.size()-1).FirmwareVersion = rxpacket[PKT_PARAMETER+3];

				memcpy(&rxpacket[0], &rxpacket[PING_STATUS_LENGTH], rx_length - PING_STATUS_LENGTH);
				rx_length -= PING_STATUS_LENGTH;
			}
			else
			{
				result = COMM_RXCORRUPT;

				// remove header (0xFF 0xFF 0xFD)
				memcpy(&rxpacket[0], &rxpacket[3], rx_length - 3);
				rx_length -= 3;
			}

			if(rx_length < PING_STATUS_LENGTH)
				break;
		}
		else
		{
			// remove unnecessary packets
			memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
			rx_length -= idx;
		}
	}

	return result;
}
/*_______________________________Ping_____________________*/
int Dynamixel::Ping(byte id, byte *error)
{
	return Ping(id, 0, error);
}
/*______________________________Ping______________________*/
int Dynamixel::Ping(byte id, PingInfo *info, byte *error)
{
	int result = COMM_TXFAIL;
	unsigned char txpacket[10] 	= {0};
	unsigned char rxpacket[14] 	= {0};

	txpacket[PKT_ID]			= (unsigned char)id;
	txpacket[PKT_LENGTH_L]		= 0x03;
	txpacket[PKT_LENGTH_H]		= 0x00;
	txpacket[PKT_INSTRUCTION]	= INST_PING;

	result = TxRxPacket(txpacket, rxpacket, error);
	if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
	{
		if(info != 0)
		{
			info->ID = rxpacket[PKT_ID];
			info->ModelNumber = DXL_MAKEWORD(rxpacket[PKT_PARAMETER+1], rxpacket[PKT_PARAMETER+2]);
			info->FirmwareVersion = rxpacket[PKT_PARAMETER+3];
		}
	}

	return result;
}
/*___________________________Reboot_________________________*/
int Dynamixel::Reboot(byte id, byte *error)
{
	unsigned char txpacket[10]	= {0};
	unsigned char rxpacket[11]	= {0};

	txpacket[PKT_ID]            = (unsigned char)id;
	txpacket[PKT_LENGTH_L]      = 0x03;
	txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_REBOOT;

	return TxRxPacket(txpacket, rxpacket, error);
}
/*___________________________FactoryReset_________________________*/
int Dynamixel::FactoryReset(byte id, int option, byte *error)
{
	unsigned char txpacket[11]	= {0};
	unsigned char rxpacket[11]	= {0};

	txpacket[PKT_ID]            = (unsigned char)id;
	txpacket[PKT_LENGTH_L]      = 0x04;
	txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET;
	txpacket[PKT_PARAMETER]		= (unsigned char)option;

	return TxRxPacket(txpacket, rxpacket, error);
}
/*__________________________Read__________________________*/
int Dynamixel::Read(byte id, int address, int length, unsigned char* data, byte *error)
{
	int result = COMM_TXFAIL;
	unsigned char txpacket[14]	= {0};
	unsigned char* rxpacket=new	unsigned char[length+11];	//= (unsigned char*)calloc(length+11, sizeof(unsigned char));

	txpacket[PKT_ID]			= (unsigned char)id;
	txpacket[PKT_LENGTH_L]		= 0x07;
	txpacket[PKT_LENGTH_H]		= 0x00;
	txpacket[PKT_INSTRUCTION]	= INST_READ;
	txpacket[PKT_PARAMETER+0]	= (unsigned char)DXL_LOBYTE(address);
	txpacket[PKT_PARAMETER+1]	= (unsigned char)DXL_HIBYTE(address);
	txpacket[PKT_PARAMETER+2]	= (unsigned char)DXL_LOBYTE(length);
	txpacket[PKT_PARAMETER+3]	= (unsigned char)DXL_HIBYTE(length);

	result = TxRxPacket(txpacket, rxpacket, error);
	if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
		std::memcpy(data, &rxpacket[PKT_PARAMETER+1], length);
	delete []rxpacket;
	//free(rxpacket);
	return result;
}
/*__________________________ReadByte__________________________*/
int Dynamixel::ReadByte(byte id, int address, byte *value, byte *error)
{
	int result = COMM_TXFAIL;
	unsigned char data[1] = {0};

	result = Read(id, address, 1, data, error);
	if(result == COMM_RXSUCCESS)
		*value = data[0];

	return result;
}
/*________________________ReadWord____________________________*/
int Dynamixel::ReadWord(byte id, int address, word *value, byte *error)
{
	int result = COMM_TXFAIL;
	unsigned char data[2] = {0};

	result = Read(id, address, 2, data, error);
	if(result == COMM_RXSUCCESS)
		*value = DXL_MAKEWORD(data[0], data[1]);

	return result;
}
/*________________________ReadDWord____________________________*/
int Dynamixel::ReadDWord(byte id, int address, dword *value, byte *error)
{
	int result = COMM_TXFAIL;
	unsigned char data[4] = {0};

	result = Read(id, address, 4, data, error);
	if(result == COMM_RXSUCCESS)
		*value = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]), DXL_MAKEWORD(data[2], data[3]));

	return result;
}
/*_________________________Write___________________________*/
int Dynamixel::Write(byte id, int address, int length, unsigned char* data, byte *error)
{
	unsigned char* txpacket = new unsigned char[length+12];//(unsigned char*)std::calloc(length+12, sizeof(unsigned char));
	unsigned char rxpacket[12] 	= {0,0,0,0,0,0,0,0,0,0,0,0};
	int result = COMM_TXFAIL;
	txpacket[PKT_ID]			= (unsigned char)id;
	txpacket[PKT_LENGTH_L]		= DXL_LOBYTE(length+5);
	txpacket[PKT_LENGTH_H]		= DXL_HIBYTE(length+5);
	txpacket[PKT_INSTRUCTION]	= INST_WRITE;
	txpacket[PKT_PARAMETER+0]	= (unsigned char)DXL_LOBYTE(address);
	txpacket[PKT_PARAMETER+1]	= (unsigned char)DXL_HIBYTE(address);

	std::memcpy(&txpacket[PKT_PARAMETER+2], data, length);

	result = TxRxPacket(txpacket, rxpacket, error);

	//free(txpacket);
  delete []txpacket;
	return result;
}
/*_________________________WriteByte___________________________*/
int Dynamixel::WriteByte(byte id, int address, byte value, byte *error)
{
	unsigned char data[1] = {value};
	return Write(id, address, 1, data, error);
}
/*__________________________WriteWord__________________________*/
int Dynamixel::WriteWord(byte id, int address, word value, byte *error)
{
	unsigned char data[2] = {DXL_LOBYTE(value), DXL_HIBYTE(value)};
	return Write(id, address, 2, data, error);
}
/*_________________________WriteDWord___________________________*/
int Dynamixel::WriteDWord(byte id, int address, long value, byte *error)
{
    unsigned char data[4] = { DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
	                          DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value)) };
	return Write(id, address, 4, data, error);
}
/*____________________________SyncWrite________________________*/
int Dynamixel::SyncWrite(int start_addr, int data_length, unsigned char* param, int param_length)
{
	int pkt_length = param_length + 7;	// 7 : INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CHKSUM_L CHKSUM_H
	int resualt;
	unsigned char* txpacket 	=new unsigned char[pkt_length+7];//= (unsigned char*)calloc(pkt_length+7, sizeof(unsigned char));
	unsigned char rxpacket[12] 	= {0};

	txpacket[PKT_ID]			= BROADCAST_ID;
	txpacket[PKT_LENGTH_L]		= DXL_LOBYTE(pkt_length);
	txpacket[PKT_LENGTH_H]		= DXL_HIBYTE(pkt_length);
	txpacket[PKT_INSTRUCTION]	= INST_SYNC_WRITE;
	txpacket[PKT_PARAMETER+0]	= DXL_LOBYTE(start_addr);
	txpacket[PKT_PARAMETER+1]	= DXL_HIBYTE(start_addr);
	txpacket[PKT_PARAMETER+2]	= DXL_LOBYTE(data_length);
	txpacket[PKT_PARAMETER+3]	= DXL_HIBYTE(data_length);
	memcpy(&txpacket[PKT_PARAMETER+4], param, param_length);

	//free(txpacket);
	resualt=TxRxPacket(txpacket, rxpacket, 0);
  delete []txpacket;
	return  resualt;
}
/*________________________BulkRead____________________________*/
int Dynamixel::BulkRead(std::vector<BulkReadData>& data)
{
	int param_length = data.size()*5;
	byte packetlength=0;
	if(param_length == 0)
		return COMM_TXFAIL;
    int result = COMM_TXFAIL, n, wait_length = 0;
    int num = data.size(); // each length : 5 (ID ADDR_L ADDR_H LEN_L LEN_H)
    int pkt_length = param_length + 3;  // 3 : INST CHKSUM_L CHKSUM_H
    unsigned char txpacket[MAXNUM_TXPACKET] = {0};
    unsigned char rxpacket[MAXNUM_RXPACKET] = {0};

    txpacket[PKT_ID]            = (unsigned char)BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = (unsigned char)DXL_LOBYTE(pkt_length);
    txpacket[PKT_LENGTH_H]      = (unsigned char)DXL_HIBYTE(pkt_length);
    txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;
    for(unsigned int i = 0; i < data.size(); i++)
    {
    	txpacket[PKT_PARAMETER+5*i+0] = data[i].iID;
    	txpacket[PKT_PARAMETER+5*i+1] = DXL_LOBYTE(data[i].iStartAddr);
    	txpacket[PKT_PARAMETER+5*i+2] = DXL_HIBYTE(data[i].iStartAddr);
    	txpacket[PKT_PARAMETER+5*i+3] = DXL_LOBYTE(data[i].iLength);
    	txpacket[PKT_PARAMETER+5*i+4] = DXL_HIBYTE(data[i].iLength);
    }


    //memcpy(&txpacket[PKT_PARAMETER], param, param_length);

    for(n = 0; n < num; n++)
    {
        wait_length += data[n].iLength + 11;
    }

    /************ TxRxPacket *************/
    // Wait for Bus Idle
//    while(comm->iBusUsing == 1)
//    {
//        //Sleep(0);
//    }
    //usleep(1000);
    //printf("aaaaa\n");

     result = TxPacket(txpacket);


    // Check Tx packet result
    if( result != COMM_TXSUCCESS )
    {
        return result;
    }

    // Set Rx Timeout (BULK_READ)
    SetPacketTimeout(wait_length);


    for(n = 0; n < num; n++)
    {
    	result = RxPacket(rxpacket,packetlength);

    	if(result == COMM_RXSUCCESS)
    	{
    		data[n].iError = rxpacket[PKT_PARAMETER];
    	}
    	else
    	{
    		//printf("result %d", result);
    		return result;
    	}

    	memcpy(data[n].pucTable, &rxpacket[PKT_PARAMETER+1], data[n].iLength);
    }

    return result;
}

/*________________________________GET_MODEL_NUMBER___________________________*/
word Dynamixel::getModelNumber(byte id, byte *error)
{
    word  modelnumber;
    ReadWord(id,DPRO_MODELNUMBER_LB,&modelnumber,error);
    return  modelnumber;
}
/*__________________________________GET_VERSION_____________________________*/
byte Dynamixel::getVersion(byte id, byte *error)
{
    byte  version;
    ReadByte(id,DPRO_VERSIONOFFIRMWARE,&version,error);
    return  version;

}

/*_____________________________________ID_RW________________________________*/
void Dynamixel::ID_RW(byte id,byte &ID,byte rw_mode, byte *error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_ID, ID,error);
    else
        ReadByte(id,DPRO_ID,&ID,error);
}

/*___________________________________BAUD_RATE_______________________________*/
void Dynamixel::BaudRate(byte id,byte &baudrate, byte rw_mode, byte *error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_BAUDRATE, baudrate,error);
    else
        ReadByte(id,DPRO_BAUDRATE,&baudrate,error);
}

/*_______________________________Return_Delay_Time___________________________*/
void Dynamixel::ReturnDelayTime(byte id,byte& value,byte rw_mode , byte *error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_RETURNDELAYTIME, value,error);
    else
        ReadByte(id,DPRO_RETURNDELAYTIME,&value,error);
}

/*_______________________________Limit_Temperature___________________________*/
void Dynamixel::LimitTemperature(byte id,byte&  limit,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_TEMPERATURELIMIT, limit,error);
    else
        ReadByte(id,DPRO_TEMPERATURELIMIT,&limit,error);
}

/*_______________________________Down_Limit_Voltage__________________________*/
void Dynamixel::DownLimitVoltage(byte id,word& limit,byte rw_mode,byte *error)
{
    if(rw_mode==WRITE)
        WriteWord(id,DPRO_MINVOLTAGELIMIT_LB, limit,error);
    else
        ReadWord(id,DPRO_MINVOLTAGELIMIT_LB,&limit,error);
}

/*________________________________Up_Limit_Voltage___________________________*/
void Dynamixel::UpLimitVoltage(byte id,word& limit,byte rw_mode,byte *error)
{
    if(rw_mode==WRITE)
        WriteWord(id,DPRO_MAXVOLTAGELIMIT_LB, limit,error);
    else
        ReadWord(id,DPRO_MAXVOLTAGELIMIT_LB,&limit,error);
}

/*______________________________Status_Return_Level__________________________*/
void Dynamixel::StatusReturnLevel(byte id,byte& value,byte rw_mode,byte *error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_STATUSRETURNLEVEL,value,error);
    else
        ReadByte(id,DPRO_STATUSRETURNLEVEL,&value,error);
}

    //////\\\\\\\////\\\/////\\\\\_RAM_AREA_/////\\\\\////\\\\\////\\\/////
/*_________________________________Torque_Enable_____________________________*/
void Dynamixel::TorqueEnable(byte id,byte &value, byte rw_mode, byte *error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_TORQUEENABLE,value,error);
    else
        ReadByte(id,DPRO_TORQUEENABLE,&value,error);
}

/*_____________________________________Led___________________________________*/
void Dynamixel::LedBlue(byte id,byte& value,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_LEDBLUE,value,error);
    else
        ReadByte(id,DPRO_LEDBLUE,&value,error);
}
void Dynamixel::LedGreen(byte id,byte& value,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_LEDGREEN,value,error);
    else
        ReadByte(id,DPRO_LEDGREEN,&value,error);
}
void Dynamixel::LedRed(byte id,byte& value,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_LEDRED,value,error);
    else
        ReadByte(id,DPRO_LEDRED,&value,error);
}
/*_________________________________Goal_Position_____________________________*/
void Dynamixel::GoalPosition(byte id,dword &value,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteDWord(id,DPRO_GOALPOSITION_LW ,value,error);
    else
        ReadDWord(id, DPRO_GOALPOSITION_LW ,&value,error);
}

/*__________________________________Goal_Speed_______________________________*/
void Dynamixel::GoalSpeed(byte id,dword& value,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteDWord(id, DPRO_GOALVELOCITY_LW ,value,error);
    else
        ReadDWord(id, DPRO_GOALVELOCITY_LW,&value,error);
}

/*_________________________________Torque_Limit______________________________*/
void Dynamixel::TorqueLimit(byte id,word& limit,byte rw_mode,byte *error)
{
    if(rw_mode==WRITE)
        WriteWord(id, DPRO_TORQUELIMIT_LB,limit,error);
    else
        ReadWord(id,DPRO_TORQUELIMIT_LB,&limit,error);
  }

/*______________________________Get_Present_Position_________________________*/
dword Dynamixel::getPresentPosition(byte id, byte *error)
{
    dword position;
    ReadDWord(id, DPRO_PRESENTPOSITION_LW,&position,error);
    return  position;
}

/*________________________________GETPRESENTSPEED___________________________*/
dword Dynamixel::getPresentSpeed(byte id, byte *error)
{
    dword speed;
    ReadDWord(id, DPRO_PRESENTVELOCITY_LW,&speed,error);
    return  speed;
}

/*________________________________Get_Present_Current___________________________*/
word Dynamixel::getPresentCurrent(byte id, byte *error)
{
    word current;
    ReadWord(id, DPRO_PRESENTCURRENT_LB,&current,error);
    return  current;
}

/*_______________________________Get_Present_Voltage_________________________*/
byte Dynamixel::getPresentVoltage(byte id, byte *error)
{
    byte volt;
    ReadByte(id, DPRO_PRESENTINPUTVOLTAGE_LB,&volt,error);
    return volt;
}

/*_____________________________Get_Present_Temperature_______________________*/
byte Dynamixel::getPresentTemperature(byte id, byte *error)
{
    byte temp;
    ReadByte( id,DPRO_PRESENTTEMPERATURE,&temp,error);
    return  temp;

}
/*______________________________Registered_Instruction_______________________*/
void Dynamixel::RegisteredInstruction(byte id,byte& value,byte rw_mode,byte* error)
{
    if(rw_mode==WRITE)
        WriteByte(id,DPRO_REGISTEREDINSTRUCTION,value,error);
    else
        ReadByte(id,DPRO_REGISTEREDINSTRUCTION,&value,error);
}

/*___________________________________Get_Moving______________________________*/
byte Dynamixel::getMoving(byte id, byte *error)
{
    byte move;
    ReadByte( id,DPRO_MOVING,&move,error);
    return  move;
}


