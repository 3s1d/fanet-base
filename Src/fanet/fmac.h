/*
 * fmac.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FMAC_H_
#define VARIO_HAL_FANET_FMAC_H_

#include <stdint.h>

#include "stm32l4xx.h"
#include "cmsis_os.h"

/*
 * Hard coded tx time assumption:
 * -SR7
 * -BW250
 * -CR8 (worst case)
 *
 * payload (byte) -> airtime (ms) -> airtime per byte payload (ms)
 * 0		9.43		0
 * 1		13.44		4.1
 * 2-5		17.54		4-1.63
 * 10		25.73		1.63
 * 64		87.17		1.2
 * 201		246.91		1.18
 * (number according to LoRa calculator)
 *
 * -> tx time assumption:
 * 15ms + 2*payload(bytes)
 * MAC_TX_MINHEADERTIME_MS + (blength * MAC_TX_TIMEPERBYTE_MS)
 */

/*
 * Timing defines
 * ONLY change if you know what you are doing. Can destroy the hole nearby network!
 */

#define MAC_SLOT_MS				20

#define MAC_TX_MINPREAMBLEHEADERTIME_MS		15
#define MAC_TX_TIMEPERBYTE_MS			2
#define MAC_TX_ACKTIMEOUT			1000
#define MAC_TX_RETRANSMISSION_TIME		1000
#define MAC_TX_RETRANSMISSION_RETRYS		3
#define MAC_TX_BACKOFF_EXP_MIN			7
#define MAC_TX_BACKOFF_EXP_MAX			12

#define MAC_FORWARD_MAX_RSSI_DBM		-90		//todo test
#define MAC_FORWARD_MIN_DB_BOOST		20
#define MAC_FORWARD_DELAY_MIN			100
#define MAC_FORWARD_DELAY_MAX			300

#define MAC_SYNCWORD				0xF1

/*
 * Number defines
 */
#define MAC_MAXNEIGHBORS_4_TRACKING_2HOP	5
#define MAC_CODING48_THRESHOLD			8

#define MAC_FRAME_LENGTH			254

/* address in flash */
#define MAC_FLASH_PAGESIZE			2048
#define MAC_ADDR_PAGE				((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/MAC_FLASH_PAGESIZE - 1)
#define MAC_ADDR_BASE				(FLASH_BASE + MAC_ADDR_PAGE*MAC_FLASH_PAGESIZE)
#define MAC_ADDR_MAGIC				0x1337000000000000ULL
#define MAC_ADDR_MAGIC_MASK			0xFFFF000000000000ULL

/* Debug */
#define MAC_debug_mode				0
#if !defined(DEBUG) && MAC_debug_mode > 0
	#undef MAC_debug_mode
	#define MAC_debug_mode 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}

#include <list>

#include "fmacaddr.h"
#include "fframe.h"
#include "fmacfifo.h"

class FanetMac;

class Fapp
{

public:
	Fapp() { }
	virtual ~Fapp() { }

	/* device -> air */
	virtual bool isBroadcastReady(void) = 0;
	virtual void broadcastSuccessful(FanetFrame::FrameType_t type) = 0;
	virtual FanetFrame *getFrame() = 0;

	/* air -> device */
	virtual void handleAcked(bool ack, FanetMacAddr &addr) = 0;
	virtual void handleFrame(FanetFrame *frm) = 0;

	/* neighbors */
	virtual std::list<FanetNeighbor*> &getNeighbors_locked(void) = 0;
	virtual void releaseNeighbors(void) = 0;
	virtual void seenNeighbor(FanetMacAddr &addr) = 0;
	virtual bool isNeighbor(FanetMacAddr & addr) = 0;
	virtual uint16_t numNeighbors(void) = 0;

	friend class FanetMac;
};

class FanetMac
{
private:
	FanetMacFifo txFifo;
	FanetMacFifo rxFifo;
	Fapp *myApp = NULL;
	FanetMacAddr _addr;
	bool _power = false;

	unsigned long csmaNextTx = 0;
	int csmaBackoffExp = MAC_TX_BACKOFF_EXP_MIN;

	/* used for interrupt handler */
	uint8_t rxFrame[MAC_FRAME_LENGTH];
	uint16_t num_received = 0;

	static void frameRxWrapper(int length);
	void receivedBuffer2Frame(int length);

	void ack(FanetFrame* frm);

	void handleTx();
	void handleRx();

public:
	bool doforward = true;
	const FanetMacAddr &addr;
	const bool &power;

	FanetMac() : addr(_addr), power(_power) { }
	~FanetMac(){};						//todo clear lists
	void handle() { handleRx(); handleTx(); };

	bool init(Fapp &app);

	void setPower(bool pwr);
	bool txQueueDepleted(void) { return (txFifo.size() == 0); };
	bool txQueueHasFreeSlots(void){ return (txFifo.size() < FANETMACFIFO_SIZE); };
	int transmit(FanetFrame *frm) { return txFifo.add(frm); };

	/* Addr */
	bool writeAddr(FanetMacAddr addr);
	bool eraseAddr(void);
	FanetMacAddr readAddr();
	bool isAddrStored(void) { return *(__IO uint64_t*)MAC_ADDR_BASE != UINT64_MAX; }
};

extern FanetMac fmac;

#endif

#endif /* VARIO_HAL_FANET_FMAC_H_ */
