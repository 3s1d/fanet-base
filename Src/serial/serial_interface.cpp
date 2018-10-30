/*
 * serial.cpp
 *
 *  Created on: 1 Oct 2016
 *      Author: sid
 */

#include <stdio.h>
#include <string>
#include <time.h>
#include <cmath>

#include "common.h"
#include "jump.h"
#include "serial.h"
#include "usart.h"

#include "../fanet/fanet.h"
#include "../fanet/fmac.h"
#include "../fanet/sx1272.h"
#include "serial_interface.h"


/*
 * C
 */
void wire_task(void const * argument)
{
	/* initialize serial */
	serialInt.begin(serial::init(&hlpuart1));

	while(1)
	{
		/* Get the message from the queue */
		osMessageGet(serialInt.myserial->queueID, 1000);		//-> queueID not nullptr by definition
		serialInt.handle_rx();
	}
}

/*
 * C++
 */

/* Fanet Commands */

/* Address: #FNA manufacturer(hex),id(hex) */
void Serial_Interface::fanet_cmd_addr(char *ch_str)
{
#if defined(DEBUG) && (SERIAL_debug_mode > 0)
	printf("### Addr %s\n", ch_str);
#endif
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		/* report addr */
		char buf[64];
		snprintf(buf, sizeof(buf), "%s%c %02X,%04X\n", FANET_CMD_START, CMD_ADDR, fmac.addr.manufacturer, fmac.addr.id);
		print(buf);
		return;
	}

	if(strstr(ch_str, "ERASE!") != NULL)
	{
		/* erase config */
		//must never be used by the end user
		if(fmac.eraseAddr())
			print_line(FN_REPLY_OK);
		else
			print_line(FN_REPLYE_FN_UNKNOWN_CMD);
		return;
	}

	/* address */
	char *p = (char *)ch_str;
	int manufacturer = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	int id = strtol(p, NULL, 16);

	if(manufacturer<=0 || manufacturer >= 0xFE || id <= 0 || id >= 0xFFFF)
	{
		print_line(FN_REPLYE_INVALID_ADDR);
	}
	else if(fmac.writeAddr(FanetMacAddr(manufacturer, id)))
		print_line(FN_REPLY_OK);
	else
		print_line(FN_REPLYE_ADDR_GIVEN);
}

/* Transmit: #FNT type,dest_manufacturer,dest_id,forward,ack_required,length,length*2hex[,signature] */
//note: all in HEX
void Serial_Interface::fanet_cmd_transmit(char *ch_str)
{
#if SERIAL_debug_mode > 0
	printf("### Packet %s\n", ch_str);
#endif

	/* w/o an address we can not tx */
	if(fmac.addr == FanetMacAddr())
	{
		print_line(FN_REPLYE_NO_SRC_ADDR);
		return;
	}

	/* no need to generate a package. tx queue is full */
	if(!fmac.txQueueHasFreeSlots())
	{
		print_line(FN_REPLYE_TX_BUFF_FULL);
		return;
	}

	FanetFrame *frm = new FanetFrame(fmac.addr);
	if(frm == nullptr)
	{
		print_line(DN_REPLYE_OUTOFMEMORY);
		return;
	}

	/* header */
	char *p = (char *)ch_str;
	frm->setType(static_cast<FanetFrame::FrameType_t>(strtol(p, NULL, 16)));
	p = strchr(p, SEPARATOR)+1;
	frm->dest.manufacturer = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->dest.id = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->forward = !!strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	/* ACK required */
	if(strtol(p, NULL, 16))
	{
		frm->ackRequested = frm->forward?FRM_ACK_TWOHOP:FRM_ACK_SINGLEHOP;
		frm->numTx = MAC_TX_RETRANSMISSION_RETRYS;
	}
	else
	{
		frm->ackRequested = FRM_NOACK;
		frm->numTx = 0;
	}

	/* payload */
	p = strchr(p, SEPARATOR)+1;
	frm->payloadLength = strtol(p, NULL, 16);
	frm->payload = new uint8_t[frm->payloadLength];
	if(frm->payload == nullptr)
	{
		delete frm;
		print_line(DN_REPLYE_OUTOFMEMORY);
		return;
	}

	p = strchr(p, SEPARATOR)+1;
	for(int i=0; i<frm->payloadLength; i++)
	{
		char sstr[3] = {p[i*2], p[i*2+1], '\0'};
		if(strlen(sstr) != 2)
		{
			print_line(FN_REPLYE_CMD_TOO_SHORT);
			delete frm;
			return;
		}
		frm->payload[i] = strtol(sstr,  NULL,  16);
	}

	/* signature */
	if((p = strchr(p, SEPARATOR)) != NULL)
		frm->signature = ((uint32_t)strtoll(++p, NULL, 16));

	/* pass to mac */
	if(fmac.transmit(frm) == 0)
	{
		if(!sx1272_isArmed())
			print_line(FN_REPLYM_PWRDOWN);
		else
			print_line(FN_REPLY_OK);
	}
	else
	{
		delete frm;
		print_line(FN_REPLYE_TX_BUFF_FULL);
	}
}

void Serial_Interface::fanet_cmd_neighbor(char *ch_str)
{
	std::list<FanetNeighbor*> neighbors = fanet.getNeighbors_locked();
	for(auto neighbor : neighbors)
	{
		char buf[48];
		snprintf(buf, sizeof(buf), "%02X,%04X", neighbor->addr.manufacturer, neighbor->addr.id);
		print(buf);
		if(strlen(neighbor->name))
		{
			print(",");
			print(neighbor->name);
		}
		print("\n");
	}
	fanet.releaseNeighbors();
	print_line(FN_REPLY_OK);
}

void Serial_Interface::fanet_cmd_promiscuous(char *ch_str)
{
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		/* report armed state */
		char buf[64];
		snprintf(buf, sizeof(buf), "%s%c %X\n", FANET_CMD_START, CMD_PROMISCUOUS, fanet.promiscuous);
		print(buf);
		return;
	}

	/* set status */
	fanet.promiscuous = !!atoi(ch_str);
	print_line(FN_REPLY_OK);
}

/* mux string */
void Serial_Interface::fanet_cmd_eval(char *str)
{
	switch(str[strlen(FANET_CMD_START)])
	{
	case CMD_TRANSMIT:
		fanet_cmd_transmit(&str[strlen(FANET_CMD_START) + 1]);
		break;
	case CMD_ADDR:
		fanet_cmd_addr(&str[strlen(FANET_CMD_START) + 1]);
		break;
	case CMD_NEIGHBOR:
		fanet_cmd_neighbor(&str[strlen(FANET_CMD_START) + 1]);
		break;
	case CMD_PROMISCUOUS:
		fanet_cmd_promiscuous(&str[strlen(FANET_CMD_START) + 1]);
		break;
	default:
		print_line(FN_REPLYE_FN_UNKNOWN_CMD);
	}
}

/*
 * Remote
 */

void Serial_Interface::fanet_remote_key(char *ch_str)
{
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		if(strlen(fanet.key))
		{
			/* report armed state */
			char buf[64];
			snprintf(buf, sizeof(buf), "%s%c %s\n", REMOTE_CMD_START, CMD_REMOTEKEY, fanet.key);
			print(buf);
		}
		else
		{
			print_line(FR_REPLYE_KEYNOTSET);
		}
		return;
	}

	/* set status */
	fanet.writeKey(ch_str);
	print_line(FN_REPLY_OK);
}

void Serial_Interface::fanet_remote_location(char *ch_str)
{
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		if(fanet.position != Coordinate3D())
		{
			/* report location */
			char buf[64];
			snprintf(buf, sizeof(buf), "%s%c %.5f,%.5f,%.f,%.1f\n", REMOTE_CMD_START, CMD_REMOTELOCATION,
					fanet.position.latitude, fanet.position.longitude, fanet.position.altitude, fanet.heading);
			print(buf);
		}
		else
		{
			print_line(FR_REPLYE_LOCATIONNOTSET);
		}
		return;
	}

	/* set position / heading */
	char *p = (char *)ch_str;
	const float lat = atof(p);
	p = strchr(p, SEPARATOR);
	if(p == nullptr)
	{
		print_line(FR_REPLYE_CMDTOOSHORT);
		return;
	}
	const float lon = atof(++p);
	p = strchr(p, SEPARATOR);
	if(p == nullptr)
	{
		print_line(FR_REPLYE_CMDTOOSHORT);
		return;
	}
	const float alt = atof(++p);
	p = strchr(p, SEPARATOR);
	if(p == nullptr)
	{
		print_line(FR_REPLYE_CMDTOOSHORT);
		return;
	}
	const float heading = atof(++p);
	fanet.writePosition(Coordinate3D(lat, lon, alt), heading);
	print_line(FN_REPLY_OK);
}

void Serial_Interface::fanet_remote_replay(char *ch_str)
{
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		print_line(FR_REPLYE_CMDTOOSHORT);
		return;
	}

	/* get mandatory feature number */
	char *p = (char *)ch_str;
	const uint16_t num = strtol(p,  NULL,  16);
	if(num >= NELEM(fanet.replayFeature))
	{
		print_line(FR_REPLYE_OUTOFBOUND);
		return;
	}

	p = strchr(p, SEPARATOR);
	if(p == nullptr)
	{
		/* print empty */
		if(fanet.replayFeature[num].type == 0xFF)
		{
			print_line(FR_REPLYM_EMPTY);
			return;
		}

		/* print feature */
		char buf[4];
		snprintf(buf, sizeof(buf), "%02X", fanet.replayFeature[num].type);
		print(buf);
		for(uint16_t i=0; i<fanet.replayFeature[num].payloadLength; i++)
		{
			snprintf(buf, sizeof(buf), "%02X", fanet.replayFeature[num].payload[i]);
			print(buf);
		}
		print("\n");
		print_line(FR_REPLY_OK);
		return;
	}
	p++;

	/* create new feature */
	uint8_t buf[sizeof(rpf_t) + 1];
	for(uint16_t i=0; i<std::min(strlen(p)/2, sizeof(buf)); i++)
	{
		char sstr[3] = { p[i*2], p[i*2+1], '\0' };
		if(strlen(sstr) != 2)
		{
			print_line(FR_REPLYE_ALIGN);
			return;
		}
		buf[i] = strtol(sstr,  NULL,  16);
	}
	fanet.writeReplayFeature(num, buf, strlen(p)/2);
	print_line(FR_REPLY_OK);
}

/* mux string */
void Serial_Interface::fanet_remote_eval(char *str)
{
	switch(str[strlen(FANET_CMD_START)])
	{
	case CMD_REMOTEKEY:
		fanet_remote_key(&str[strlen(REMOTE_CMD_START) + 1]);
		break;
	case CMD_REMOTELOCATION:
		fanet_remote_location(&str[strlen(REMOTE_CMD_START) + 1]);
		break;
	case CMD_REMOTEREPLAY:
		fanet_remote_replay(&str[strlen(REMOTE_CMD_START) + 1]);
		break;
	default:
		print_line(FN_REPLYE_FN_UNKNOWN_CMD);
	}
}

/*
 * Dongle Stuff
 */

void Serial_Interface::dongle_cmd_version(char *ch_str)
{
	if(myserial == NULL)
		return;

	char buf[64];
	snprintf(buf, sizeof(buf), "%s%c %s\n", DONGLE_CMD_START, CMD_VERSION, BUILD);
	print(buf);
}

void Serial_Interface::dongle_cmd_jump(char *ch_str)
{
	if(!strncmp(ch_str, " BLstm", 6))
		jump_mcu_bootloader();

	print_line(DN_REPLYE_JUMP);
}

void Serial_Interface::dongle_cmd_power(char *ch_str)
{
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		/* report armed state */
		char buf[64];
		snprintf(buf, sizeof(buf), "%s%c %X\n", DONGLE_CMD_START, CMD_POWER, sx1272_isArmed());
		print(buf);
		return;
	}

	/* set status */
	if(sx1272_setArmed(!!atoi(ch_str)))
		print_line(DN_REPLY_OK);
	else
		print_line(DN_REPLYE_POWER);
}

void Serial_Interface::dongle_cmd_region(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### Region "));
	SerialDEBUG.print(ch_str);
#endif

	/* eval parameter */
	char *p = (char *)ch_str;
	if(p == NULL)
	{
		print_line(DN_REPLYE_TOOLESSPARAMETER);
		return;
	}
	int freq = strtol(p, NULL, 10);
	p = strchr(p, SEPARATOR)+1;
	if(p == NULL)
	{
		print_line(DN_REPLYE_TOOLESSPARAMETER);
		return;
	}
	sx_region_t region;
	region.channel = 0;
	region.dBm = strtol(p, NULL, 10);

	switch(freq)
	{
	case 868:
		region.channel = CH_868_200;
		break;
	}

	/* configure hardware */
	if(region.channel && sx1272_setRegion(region))
		print_line(DN_REPLY_OK);
	else
		print_line(DN_REPLYE_UNKNOWNPARAMETER);
}

/* mux string */
void Serial_Interface::dongle_eval(char *str)
{
	switch(str[strlen(DONGLE_CMD_START)])
	{
	case CMD_VERSION:
		dongle_cmd_version(&str[strlen(DONGLE_CMD_START) + 1]);
		break;
	case CMD_POWER:
		dongle_cmd_power(&str[strlen(DONGLE_CMD_START) + 1]);
		break;
	case CMD_REGION:
		dongle_cmd_region(&str[strlen(DONGLE_CMD_START) + 1]);
		break;
	case CMD_BOOTLOADER:
		dongle_cmd_jump(&str[strlen(DONGLE_CMD_START) + 1]);
		break;
	default:
		print_line(DN_REPLYE_DONGLE_UNKNOWN_CMD);
	}
}

/* collect string */
void Serial_Interface::handle_rx(void)
{
	if(myserial == NULL)
		return;

	char line[256];
	bool cmd_rxd = serial::poll(myserial, line, sizeof(line));
	if (cmd_rxd == false)
		return;

	/* Process message */
#if defined(DEBUG) && (SERIAL_debug_mode > 1)
	printf("### rx:'%s'\n", line);
#endif
	if(!strncmp(line, FANET_CMD_START, 3))
		fanet_cmd_eval(line);
	else if(!strncmp(line, REMOTE_CMD_START, 3))
		fanet_remote_eval(line);
	else if(!strncmp(line, DONGLE_CMD_START, 3))
		dongle_eval(line);
	else
		print_line(FN_REPLYE_UNKNOWN_CMD);

	last_activity = HAL_GetTick();
}

void Serial_Interface::begin(serial_t *serial)
{
	myserial = serial;
}

/*
 * Handle redirected App Stuff
 */

void Serial_Interface::handle_acked(bool ack, FanetMacAddr &addr)
{
	if(myserial == NULL)
		return;

	char buf[64];
	snprintf(buf, sizeof(buf), "%s,%X,%X\n", ack?FANET_CMD_ACK:FANET_CMD_NACK, addr.manufacturer, addr.id);
	print(buf);
}

void Serial_Interface::handle_frame(FanetFrame *frm)
{
	if(myserial == NULL || frm == NULL)
		return;

	/* simply print frame */
	/* src_manufacturer,src_id,broadcast,signature,type,payloadlength,payload */

	char buf[128];
	snprintf(buf, sizeof(buf), "%s %X,%X,%X,%X,%X,%X,",
			FANET_CMD_START CMD_RX_FRAME, frm->src.manufacturer, frm->src.id, frm->dest==FanetMacAddr(), (unsigned int)frm->signature,
			frm->type, frm->payloadLength);
	print(buf);

	for(int i=0; i<frm->payloadLength; i++)
	{
		snprintf(buf, sizeof(buf), "%02X", frm->payload[i]);
		print(buf);
	}
	print((char*) "\n");
}

void Serial_Interface::print_line(const char *type, int key, const char *msg)
{
	if(myserial == NULL || type == NULL)
		return;

	/* general answer */
	print(type);

	/* key */
	char buf[64];
	if(key > 0)
	{
		snprintf(buf, sizeof(buf), ",%d", key);
		print(buf);

		/* human readable message */
		if(msg != nullptr && strlen(msg) > 0)
		{
			snprintf(buf, sizeof(buf), ",%s", msg);
			print(buf);
		}
	}

	print("\n");
}

Serial_Interface serialInt = Serial_Interface();

