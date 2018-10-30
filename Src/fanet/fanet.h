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

#define FANET_RC_ACK				0
#define FANET_RC_POSITION			2
#define FANET_RC_REPLAY_LOWER			3
#define FANET_RC_REPLAY_UPPER			15

#define FLASH_PAGESIZE				2048
#define FANET_KEYADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 2)
#define FANET_KEYADDR_BASE			(FLASH_BASE + FANET_KEYADDR_PAGE*FLASH_PAGESIZE)
#define FANET_POSADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 3)
#define FANET_POSADDR_BASE			(FLASH_BASE + FANET_POSADDR_PAGE*FLASH_PAGESIZE)
#define FANET_RPADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 4)
#define FANET_RPADDR_BASE			(FLASH_BASE + FANET_RPADDR_PAGE*FLASH_PAGESIZE)

#include "fmac.h"

typedef struct
{
	uint8_t type;
	uint8_t payloadLength;
	uint8_t payload[150];
} rpf_t;					//note: size has to divide'able by 8 AND sizeof(rpf_t) <= FLASH_PAGESIZE/13


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

	/* position, all in degree */
	Coordinate3D _position = Coordinate3D();
	float _heading = 0.0f;

	/* remote */
	void loadKey(void);
	void loadPosition(void);
	void loadReplayFeatures(void);
	void writeReplayFeatures(void);

	bool rcPosition(uint8_t *payload, uint16_t payload_length);
	bool decodeRemoteConfig(FanetFrame *frm);

public:
	bool promiscuous = false;
	const char *key = _key;
	const Coordinate3D &position;
	const float &heading;
	rpf_t replayFeature[13];				//will be initialized upon constructor

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
	bool writeKey(char *newKey);
	void writePosition(Coordinate3D newPos, float newHeading);
	bool writeReplayFeature(uint16_t num, uint8_t *payload, uint16_t len);
};

extern Fanet fanet;

#endif

#endif /* VARIO_HAL_FANET_FANET_H_ */
