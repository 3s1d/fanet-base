/*
 * serial.h
 *
 *  Created on: 1 Oct 2016
 *      Author: sid
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif


void wire_task(void const * argument);

#ifdef __cplusplus
}

#include <string.h>
#include "serial.h"

#include "config.h"
#include "../../fanet/fanet.h"
#include "../../fanet/fmac.h"


#define FANET_CMD_START			"#FN"
#define REMOTE_CMD_START		"#FR"
#define DONGLE_CMD_START		"#DG"
#define SEPARATOR			','


/* Fanet Replies */
#define FANET_CMD_OK			FANET_CMD_START "R OK"
#define FANET_CMD_ERROR			FANET_CMD_START "R ERR"
#define FANET_CMD_MSG			FANET_CMD_START "R MSG"
#define FANET_CMD_ACK			FANET_CMD_START "R ACK"
#define FANET_CMD_NACK			FANET_CMD_START "R NACK"

/* Remote Replies */
#define REMOTE_CMD_OK			REMOTE_CMD_START "R OK"
#define REMOTE_CMD_ERROR		REMOTE_CMD_START "R ERR"
#define REMOTE_CMD_MSG			REMOTE_CMD_START "R MSG"

/* Dongle Replies */
#define DONGLE_CMD_OK			DONGLE_CMD_START "R OK"
#define DONGLE_CMD_MSG			DONGLE_CMD_START "R MSG"
#define DONGLE_CMD_WARN			DONGLE_CMD_START "R WRN"
#define DONGLE_CMD_ERROR		DONGLE_CMD_START "R ERR"

/* FLARM Replies */
#ifdef FLARM
#define FLARM_CMD_OK			FLARM_CMD_START "R OK"
#define FLARM_CMD_ERROR			FLARM_CMD_START "R ERR"
#endif

/* Commands */
/* FANET */
#define CMD_STATE			'S'
#define CMD_TRANSMIT			'T'
#define CMD_ADDR			'A'
#define CMD_CONFIG			'C'
#define CMD_MODE			'M'
#define CMD_NEIGHBOR			'N'
#define CMD_PRINT2CONSOLE		'P'
#define CMD_WEATHER			'W'
#define CMD_DUMP			'D'

#define CMD_RX_FRAME			"F"

/* Remote */
#define CMD_REMOTEKEY			'K'
#define CMD_COORDINATE			'C'
#define CMD_REMOTEREPLAY		'R'
#define CMD_REMOTEGEOFENCE		'G'

/* Dongle */
#define CMD_VERSION			'V'
#define CMD_POWER			'P'
#define CMD_REGION			'L'
#define CMD_BOOTLOADER			'J'

#define SERIAL_debug_mode		0

/* values <= 0 removes code and message */
#define FN_REPLY_OK			FANET_CMD_OK, 	 0,  ""
#define FN_REPLYM_INITIALIZED		FANET_CMD_MSG,   1,  "initialized"
#define FN_REPLYE_RADIO_FAILED		FANET_CMD_ERROR, 2,  "radio failed"
#define FN_REPLYE_UNKNOWN_CMD		FANET_CMD_ERROR, 5,  "unknown command"
#define FN_REPLYE_FN_UNKNOWN_CMD	FANET_CMD_ERROR, 6,  "unknown FN command"
#define FN_REPLYE_NO_SRC_ADDR		FANET_CMD_ERROR, 10, "no source address"
#define FN_REPLYE_INVALID_ADDR		FANET_CMD_ERROR, 11, "invalid address"
#define FN_REPLYE_INCOMPATIBLE_TYPE	FANET_CMD_ERROR, 12, "incompatible type"
#define FN_REPLYM_PWRDOWN		FANET_CMD_MSG,	 13, "power down"
#define FN_REPLYE_TX_BUFF_FULL		FANET_CMD_ERROR, 14, "tx buffer full"
#define FN_REPLYE_ADDR_GIVEN		FANET_CMD_ERROR, 15, "address already set"
#define FN_REPLYE_CMD_TOO_SHORT		FANET_CMD_ERROR, 30, "too short"
#define FN_REPLYE_NOPOSITION		FANET_CMD_ERROR, 31, "no location"
#define FR_REPLY_OK			REMOTE_CMD_OK, 	 0,  ""
#define FR_REPLYE_KEYNOTSET		REMOTE_CMD_ERROR,35, "key not set"
#define FR_REPLYE_CMDTOOSHORT		REMOTE_CMD_ERROR,37, "too short"
#define FR_REPLYE_OUTOFBOUND		REMOTE_CMD_ERROR,38, "out of bound"
#define FR_REPLYE_ALIGN			REMOTE_CMD_ERROR,39, "align"
#define FR_REPLYM_EMPTY			REMOTE_CMD_MSG,  40, "empty"
#define FR_REPLYE_WRITEFAILED		REMOTE_CMD_ERROR,41, "write failed"
#define FR_REPLYE_CMD_TOO_SHORT		FANET_CMD_ERROR, 32, "too short"
#define DN_REPLY_OK			DONGLE_CMD_OK, 	 0,  ""
#define DN_REPLYE_DONGLE_UNKNOWN_CMD	DONGLE_CMD_ERROR,60, "unknown DG command"
#define DN_REPLYE_JUMP			DONGLE_CMD_ERROR,61, "unknown jump point"
#define DN_REPLYE_POWER			DONGLE_CMD_ERROR,70, "power switch failed"
#define DN_REPLYE_TOOLESSPARAMETER	DONGLE_CMD_ERROR,80, "too less parameter"
#define DN_REPLYE_UNKNOWNPARAMETER	DONGLE_CMD_ERROR,81, "unknown parameter"
#define DN_REPLYE_OUTOFMEMORY		DONGLE_CMD_ERROR,85, "out of memory"
#define DN_REPLYE_LOCATIONNOTSET	DONGLE_CMD_ERROR,90, "location not set"
#define DN_REPLYE_WRITEFAILED		DONGLE_CMD_ERROR,91, "write failed"

#ifdef FLARM
#define FA_REPLY_OK			FLARM_CMD_OK, 	 0,  ""
#define FA_REPLYE_UNKNOWN_CMD		FLARM_CMD_ERROR, 90, "unknown FLARM command"
#define FA_REPLYE_EXPIRED		FLARM_CMD_ERROR, 91, "FLARM expired"
#endif
/*
 * Normal Commands
 *
 * Direct Weather:	#FNW inet(0..1),[temperature(float)],						note: despite of inet everything is
 * 				[wind direction(degree,float)],							optional. just put correct amount
 * 				[wind speed(kmph,float)],[wind gust(kmph,float)],				of commas.
 * 				[humidity(percent,float)],[pressure(hPa,float)]
 * Dump:		#FND										note: one neighbor per line
 * Transmit: 		#FNT type,dest_manufacturer,dest_id,forward,ack_required,length,length*2hex[,signature]	note: all values in hex
 *
 * Address: 		#FNA manufacturer(hex),id(hex)							note: w/o address is returned
 * Neighbors:		#FNN										note: one neighbor per line
 * Print frame:		#FNP toConsole (0..2)								note: 0=off(default), 1=on, 2=everything
 * Config:		#FNC Inet(0..1, for type 4)							note: Inet=1 sets toConsole to >0 as well
 *
 * Remote Key:		#FRK key
 * Replay Feature:	#FRR num(hex)[,type(hex),windsector(hex, always=FF),forwarding(0..1),payload]	note: only until num gives an reply,
 * 													note: #FRR num,0 -> clean
 * GeoFrence		#FRG num(0..3)[,numVertices(hex)[,floor(hex m),ceiling(hex m),[,lat(float deg),lon(float deg)] 'num' times]]
 * 													note: #FRG num,0 -> clean
 *
 * Receive a Frame:	#FNF src_manufacturer,src_id,broadcast,signature,type,payloadlength,payload
 *
 * Maintenance/Dongle
 * Version:		#DGV
 * Power:		#DGP powermode(0..2)								note: 0=off, 1=on(compare, default), 2=psu
 * Region:		#DGL freq(868,915),power(2..20 (dBm))						note: 10dBm for Skytraxx stock antenna
 * Coordinate:		#DGC latitude,longitude,altitude,heading					note: in degree
 *
 * Jump to DFU:		#DGJ BLstm									note: stm32 default bootloader
 * 													recommended: use 2 GPIOs connected
 * 															to RESEST and BOOT0
 */

class Serial_Interface
{
private:
	uint32_t last_activity = 0;

	uint16_t hash(uint8_t *buf, uint16_t len);			//len must be mod 2

	/* Normal Commands */
	void fanet_cmd_eval(char *str);
	void fanet_cmd_addr(char *ch_str);
	void fanet_cmd_transmit(char *ch_str);
	void fanet_cmd_neighbor(char *ch_str);
	void fanet_cmd_promiscuous(char *ch_str);
	void fanet_cmd_weather(char *ch_str);
	void fanet_cmd_dump(char *ch_str);
	void fanet_cmd_config(char *ch_str);

	void fanet_remote_eval(char *str);
	void fanet_remote_key(char *ch_str);
	void fanet_remote_replay(char *ch_str);
	void fanet_remote_geofence(char *ch_str);

	/* Dongle Commands */
	void dongle_eval(char *str);
	void dongle_cmd_version(char *ch_str);
	void dongle_cmd_power(char *ch_str);
	void dongle_cmd_region(char *ch_str);
	void dongle_cmd_jump(char *ch_str);
	void dongle_cmd_location(char *ch_str);

public:
	serial_t *myserial = nullptr;

	Serial_Interface() { }
	void begin(serial_t *serial);

	/* redirected from app */
	void handleFrame(FanetFrame *frm);
	void handle_acked(bool ack, FanetMacAddr &addr);

	void handle_rx(void);
	void ack(bool value, int manufacturer, int id);

	uint32_t get_lastactivity(void) { return last_activity; };
	bool any_actitity(void) { return (last_activity!=0); };

	void print_raw(const uint8_t *data, uint16_t len);
	void print_line(const char *type, int key, const char *msg);
	void print(const char *str) { serial::print(str, myserial); }
};

extern Serial_Interface serialInt;


#endif


#endif /* SERIAL_H_ */
