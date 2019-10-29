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

#define FANET_RADIO_UPTIME			750

/* service */
#define	FANET_TYPE4_TAU_MS			20000

/* remote configuration, replayed data */
#define	FANET_TYPE6_TAU_MS			30000
#define FANET_TYPE6_PAUSE_MS			180000

/* HW info */
#define	FANET_TYPE8_TAU_MS			600000

#define FLASH_PAGESIZE				2048
#define FANET_KEYADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 2)
#define FANET_KEYADDR_BASE			(FLASH_BASE + FANET_KEYADDR_PAGE*FLASH_PAGESIZE)
#define FANET_POSADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 3)
#define FANET_POSADDR_BASE			(FLASH_BASE + FANET_POSADDR_PAGE*FLASH_PAGESIZE)
#define FANET_RPADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 4)
#define FANET_RPADDR_BASE			(FLASH_BASE + FANET_RPADDR_PAGE*FLASH_PAGESIZE)
#define FANET_GEOFENCEADDR_PAGE			((((uint16_t)(READ_REG(*((uint32_t *)FLASHSIZE_BASE)))) * 1024)/FLASH_PAGESIZE - 5)
#define FANET_GEOFENCEADDR_BASE			(FLASH_BASE + FANET_GEOFENCEADDR_PAGE*FLASH_PAGESIZE)
#define FANET_KEY_SIZE				16

#include "replay.h"
#include "fmac.h"
#include "geofence.h"

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

	/* forwarding */
	osMutexDef(geoFenceMutex);
	osMutexId geoFenceMutex = osMutexCreate(osMutex(geoFenceMutex));
	GeoFence geoFence[4];

	/* replay features */
	static const uint16_t numReplayFeatureNoneVolatile = 12;
	static const uint16_t numReplayFeatureVolatile = 12;
	osMutexDef(replayFeatureMutex);
	osMutexId replayFeatureMutex = osMutexCreate(osMutex(replayFeatureMutex));
	Replay replayFeature[numReplayFeatureNoneVolatile+numReplayFeatureVolatile];

	/* broadcasts */
	uint32_t nextRfBroadcast = 1000;
	int16_t nextRfIdx = -1;
	uint32_t nextServiceBroadcast = 500;
	uint32_t txQueueLastUsed = 0;
	uint32_t nextHwInfoBroadcast = 2000;

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
	void loadGeoFences(void);
	bool writeReplayFeatures(uint16_t num);

public:
	const char *key = _key;
	const Coordinate3D &position;
	const float &heading;

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

	/* forwarding */
	bool isGeoForwarding(void);
	uint16_t numGeoFences(void) { return NELEM(geoFence); }
	GeoFence &getGeoFence_locked(uint16_t num);
	void releaseGeoFence(void);

	/* replay feature */
	uint16_t numReplayFeatures(void) { return NELEM(replayFeature); }
	Replay &getReplayFeature_locked(uint16_t num);
	bool releaseReplayFeature(int16_t changedNum = -1);

	/* manual / serial */
	void manualServiceSent(void) { noAutoServiceBefore = osKernelSysTick() + 180000; }			//disable for 3min
	void setFrameToConsole(int16_t what) { fmac.promiscuous = (what == 2) || isGeoForwarding(); _frameToConsole = what; }
	bool sendHwInfo(void);

	/* ACK */
	void ackReset(void) {ackAddr = FanetMacAddr(); ackRes = WAIT; }
	bool ackResult(const FanetMacAddr &addr, FanetAckRes_t &result) {result = ackRes; return addr == ackAddr; }

	/* remote config */
	bool writeKey(char *newKey);
	bool writePosition(Coordinate3D newPos, float newHeading);
	bool writeGeoFences(void);
};

extern Fanet fanet;

#endif

#endif /* VARIO_HAL_FANET_FANET_H_ */
