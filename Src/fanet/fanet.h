/*
 * fanet.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FANET_H_
#define VARIO_HAL_FANET_FANET_H_

#ifdef __cplusplus
extern "C" {
#endif

/* C */

/* Queue */
#define FANET_QUEUE_SIZE			4
#define FANET_SX1272_IRQ			1

void fanet_rtos(void);
void fanet_irq(void);
void fanet_task(void const * argument);


#ifdef __cplusplus
}
/* C++ */

#include <list>

#define FANET_NEIGHBOR_SIZE			64

#define FANET_TYPE1OR7_AIRTIME_MS		40		//more like 20-30ms
#define	FANET_TYPE1OR7_MINTAU_MS		250
#define	FANET_TYPE1OR7_TAU_MS			5000
#define FANET_TYPE2_TAU_MS			240000		//4min


#define FLASH_PAGESIZE				2048
#define FANET_KEYADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 2)
#define FANET_KEYADDR_BASE			(FLASH_BASE + FANET_KEYADDR_PAGE*FLASH_PAGESIZE)

#include "fmac.h"


class Fanet : public Fapp
{
public:
	enum FanetAckRes_t : int16_t
	{
		NACK = -1,
		WAIT = 0,
		ACK = 1
	};
private:
	/* neighbors */
	std::list<FanetNeighbor*> neighbors;
	osMutexDef(neighborMutex);
	osMutexId neighborMutex = osMutexCreate(osMutex(neighborMutex));

	/* broadcasts */
	uint32_t lastTrkBroadcast = 0;
	uint32_t nextTrkBroadcast = 0;
	uint32_t nextNameBroadcast = 0;

	/* ACK buffer */
	FanetMacAddr ackAddr;
	FanetAckRes_t ackRes = WAIT;
	char _key[16] = { '\0' };

public:
	bool promiscuous = false;
	const char *key = _key;

	Fanet();

	/* device -> air */
	bool isBroadcastReady(void);
	void broadcastSuccessful(int type) { lastTrkBroadcast = osKernelSysTick(); }
	FanetFrame *getFrame();

	/* air -> device */
	void handleAcked(bool ack, FanetMacAddr &addr);
	void handleFrame(FanetFrame *frm);
	void handle(void);				//general stuff

	/* neighbors */
	std::list<FanetNeighbor*> &getNeighbors_locked(void);
	FanetNeighbor *getNeighbor_locked(const FanetMacAddr &addr);
	void releaseNeighbors(void);
	void seenNeighbor(FanetMacAddr &addr);
	void cleanNeighbors(void);
	bool isNeighbor(FanetMacAddr & addr);
	uint16_t numNeighbors(void);

	/* ACK */
	void ackReset(void) {ackAddr = FanetMacAddr(); ackRes = WAIT; }
	bool ackResult(const FanetMacAddr &addr, FanetAckRes_t &result) {result = ackRes; return addr == ackAddr; }

	/* remote config */
	void writeKey(char *newKey);
	void loadKey(void);
};

extern Fanet fanet;

#endif

#endif /* VARIO_HAL_FANET_FANET_H_ */
