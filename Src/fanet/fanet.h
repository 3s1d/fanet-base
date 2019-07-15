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


/* service */
#define	FANET_TYPE4_TAU_MS			20000

/* remote configuration, replayed data */
#define	FANET_TYPE6_TAU_MS			30000
#define FANET_TYPE6_PAUSE_MS			180000

#define FLASH_PAGESIZE				2048
#define FANET_KEYADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 2)
#define FANET_KEYADDR_BASE			(FLASH_BASE + FANET_KEYADDR_PAGE*FLASH_PAGESIZE)
#define FANET_POSADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 3)
#define FANET_POSADDR_BASE			(FLASH_BASE + FANET_POSADDR_PAGE*FLASH_PAGESIZE)
#define FANET_RPADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 4)
#define FANET_RPADDR_BASE			(FLASH_BASE + FANET_RPADDR_PAGE*FLASH_PAGESIZE)

#define FANET_KEY_SIZE				16

#include "replay.h"
#include "fmac.h"

/*
typedef struct
{
	uint8_t type;
	uint8_t payloadLength;
	uint8_t windSector;
	uint8_t padding;
	uint8_t payload[132];
} __attribute__((packed)) rpf_t;			//note: size has to divide'able by 8 AND sizeof(rpf_t) <= FLASH_PAGESIZE/12
*/

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
	uint32_t nextRfBroadcast = 1000;
	int16_t nextRfIdx = -1;
	uint32_t nextServiceBroadcast = 500;

	/* ACK buffer */
	FanetMacAddr ackAddr;
	FanetAckRes_t ackRes = WAIT;
	char _key[FANET_KEY_SIZE] = { '\0' };

	/* position, all in degree */
	Coordinate3D _position = Coordinate3D();
	float _heading = 0.0f;

	/* manual / serial */
	uint32_t noAutoServiceBefore = 0;
	int16_t _frameToConsole = 0;									//0: false, 1: true, 2: promiscuous

	/* remote */
	void loadKey(void);
	void loadPosition(void);
	void loadReplayFeatures(void);

public:
	const char *key = _key;
	const Coordinate3D &position;
	const float &heading;
	Replay replayFeature[12];									//will be initialized upon constructor

	const int16_t &frameToConsole;

	Fanet() : Fapp(), position(_position), heading(_heading), frameToConsole(_frameToConsole) { }
	void init(FanetMac *fmac);

	/* device -> air */
	FanetFrame *broadcastIntended(void);
	void broadcastSuccessful(FanetFrame::FrameType_t type);

	/* air -> device */
	void handleAcked(bool ack, FanetMacAddr &addr);
	void handleFrame(FanetFrame *frm);
	void handle(void);											//general stuff

	/* neighbors */
	std::list<FanetNeighbor*> &getNeighbors_locked(void);
	FanetNeighbor *getNeighbor_locked(const FanetMacAddr &addr);
	void releaseNeighbors(void);
	void seenNeighbor(FanetMacAddr &addr);
	void cleanNeighbors(void);
	bool isNeighbor(FanetMacAddr & addr);
	uint16_t numNeighbors(void);

	/* manual / serial */
	void manualServiceSent(void) { noAutoServiceBefore = osKernelSysTick() + 180000; }			//disable for 3min
	void setFrameToConsole(int16_t what) { fmac.promiscuous = (what == 2); /* || geofoarwrding */ _frameToConsole = what;}

	/* ACK */
	void ackReset(void) {ackAddr = FanetMacAddr(); ackRes = WAIT; }
	bool ackResult(const FanetMacAddr &addr, FanetAckRes_t &result) {result = ackRes; return addr == ackAddr; }

	/* remote config */
	bool writeKey(char *newKey);
	bool writePosition(Coordinate3D newPos, float newHeading);
	bool writeReplayFeatures(void);
};

extern Fanet fanet;

#endif

#endif /* VARIO_HAL_FANET_FANET_H_ */
