/*
 * fanet.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#include "cmsis_os.h"

#include "common.h"
#include "minmax.h"
#include "print.h"

#include "../hal/power.h"
#include "../hal/serial/serial_interface.h"
#include "../hal/sht2x/sht2x.h"
#include "../hal/wind.h"
#include "../misc/rnd.h"
#include "../misc/sha1.h"
#include "sx1272.h"
#include "fanet.h"
#include "frame/fname.h"
#include "frame/fgndtracking.h"
#include "frame/fhwinfo.h"
#include "frame/fservice.h"
#include "frame/ftracking.h"

osMessageQId fanet_QueueID;

void fanet_rtos(void)
{
	/* Queue */
	osMessageQDef(Fanet_Queue, FANET_QUEUE_SIZE, uint16_t);
	fanet_QueueID = osMessageCreate (osMessageQ(Fanet_Queue), NULL);
}

void fanet_irq(void)
{
	if(osKernelRunning() == 0)
		return;

	osMessagePut(fanet_QueueID, FANET_SX1272_IRQ, osWaitForever);
}

void fanet_task(void const * argument)
{
	/* seed rnd */
	/* start random machine */
	rnd::seed(HAL_GetUIDw0() + HAL_GetUIDw1() + HAL_GetUIDw2());

	/* configure mac */
	while(fmac.init(fanet) == false)
	{
		serialInt.print_line(FN_REPLYE_RADIO_FAILED);
		osDelay(1000);
	}
	//fmac.writeAddr(FanetMacAddr(1, 2));

	osThreadYield();
	serialInt.print_line(FN_REPLYM_INITIALIZED);
	debug_printf("Addr %02X:%04X\n", fmac.addr.manufacturer, fmac.addr.id);

	/* turn on FANET */
	fmac.setPower(true);

	/* fanet loop */
	while(1)
	{
		/* Get the message from the queue */
		osEvent event = osMessageGet(fanet_QueueID, MAC_SLOT_MS);
		if(event.status == osEventMessage)
		{
			if(event.value.v == FANET_SX1272_IRQ)
				sx1272_irq();
		}

		/* handle rx/tx stuff */
		fmac.handle();

		/* handle regular fanet stuff */
		fanet.handle();
	}
}

/*
 * C++
 */

/*
 * Memory management
 */
void *operator new(size_t size)
{
	void *mem = pvPortMalloc(size);
		return mem;
}

void operator delete(void *p)
{
	vPortFree(p);
}


void Fanet::handle(void)
{
	const uint32_t current = osKernelSysTick();

	/* time for next replay broadcast? */
	if(nextRfBroadcast < current)
	{
		osMutexWait(replayFeatureMutex, osWaitForever);

		/* just in case, nothing is found delay next eval by 15sec */
		nextRfBroadcast = current + FANET_TYPE6_TAU_MS/2;

		/* find next index */
		bool found = false;
		for(uint16_t i=0; i<NELEM(replayFeature); i++)
		{
			/* overflow -> pause */
			if(++nextRfIdx >= (int16_t)NELEM(replayFeature))
			{
				nextRfIdx = -1;
				nextRfBroadcast = current + FANET_TYPE6_PAUSE_MS;
				break;
			}

			if(nextRfIdx >= 0 && replayFeature[nextRfIdx].isVaild())
			{
				found = true;
				break;
			}
		}

		/* send frame */
		if(found == true)
		{
			/* in case of a busy channel, ensure that frames from the tx fifo gets also a change */
			nextRfBroadcast = current + FANET_TYPE6_TAU_MS * (!!power::isSufficiant() + 1);

			/* broadcast replay */
			FanetFrame *frm = replayFeature[nextRfIdx].toFrame();
			if(fmac.transmit(frm) != 0)
			{
				/* failed, try again latter */
				delete frm;
				nextRfIdx--;
				nextRfBroadcast = current + FANET_TYPE6_TAU_MS;
			}
		}

		osMutexRelease(replayFeatureMutex);
	}

	/* power management */
	if(power::isSufficiant() == false)
	{
		/* not enough power for forwarding of any kind */
		fmac.promiscuous = false;
		fmac.doForward = false;

		/* control rf chip */
		if(fmac.isTxQueueEmpty() == false || nextServiceBroadcast <= current)
		{
			/* enabling radio chip */
			sx1272_setArmed(true);
			txQueueLastUsed = current;
		}
		else if(txQueueLastUsed + FANET_RADIO_UPTIME < current)		//750ms delay to ensure remote config can reach us
		{
			/* disabling radio chip */
			sx1272_setArmed(false);
		}
	}
	else
	{
		if(nextHwInfoBroadcast < current)
		{
			/* in case of a busy channel, ensure that frames from the tx fifo gets also a change */
			nextHwInfoBroadcast = current + FANET_TYPE8_TAU_MS;

			/* count replay features */
			osMutexWait(replayFeatureMutex, osWaitForever);
			uint16_t usedRf = 0;
			for(uint16_t i = numReplayFeatureNoneVolatile; i<NELEM(replayFeature); i++)
				if(replayFeature[i].used())
					usedRf++;
			osMutexRelease(replayFeatureMutex);

			/* broadcast HW info */
			debug_printf("HwInfo Tx\n");
			FanetFrame *frm = new FanetFrameHwInfo(usedRf);
			if(fmac.transmit(frm) != 0)
			{
				/* failed, try again latter */
				delete frm;
				nextHwInfoBroadcast = current + 10000;			//10sec;
			}
		}

		/* ensure continuous operation */
		sx1272_setArmed(true);

		/* Geo-based forwarding */
		osMutexWait(geoFenceMutex, osWaitForever);
		bool doGeoForward = false;
		for(uint16_t i=0; i<NELEM(geoFence) && !doGeoForward; i++)
			doGeoForward |= geoFence[i].isActive();
		osMutexRelease(geoFenceMutex);
		fmac.promiscuous = doGeoForward || !!frameToConsole;
		fmac.doForward = !doGeoForward || (numNeighbors() < MAC_MAXNEIGHBORS_FOR_2HOP);

	}

	/* remove unavailable nodes */
	fanet.cleanNeighbors();
}

bool Fanet::sendHwInfo(void)
{
	/* already sent a few seconds (30s) ago */
	if(nextHwInfoBroadcast > osKernelSysTick()+FANET_TYPE8_TAU_MS-30000 || power::isSufficiant() == false)
		return false;

	nextHwInfoBroadcast = 0;
	return true;

}

std::list<FanetNeighbor*> &Fanet::getNeighbors_locked(void)
{
	osMutexWait(neighborMutex, osWaitForever);
	return neighbors;
}

FanetNeighbor *Fanet::getNeighbor_locked(const FanetMacAddr &addr)
{
	osMutexWait(neighborMutex, osWaitForever);

	/* find neighbor of interest */
	FanetNeighbor *retNeighbor = nullptr;
	for(auto *n : neighbors)
	{
		if(n->addr == addr)
		{
			retNeighbor = n;
			break;
		}
	}

	/* does not exist -> release */
	if(retNeighbor == nullptr)
		osMutexRelease(neighborMutex);

	return retNeighbor;
}

void Fanet::releaseNeighbors(void)
{
	osMutexRelease(neighborMutex);
}

void Fanet::cleanNeighbors(void)
{
	osMutexWait(neighborMutex, osWaitForever);
	neighbors.remove_if([](FanetNeighbor *fn){ if(fn->isAround()) return false; delete fn; return true; });
	osMutexRelease(neighborMutex);
}

void Fanet::seenNeighbor(FanetMacAddr &addr)
{
	osMutexWait(neighborMutex, osWaitForever);

	/* update neighbors list */
	bool neighbor_known = false;
	for(auto *neighbor : neighbors)
	{
		if(neighbor->addr == addr)
		{
			neighbor->seen();
			neighbor_known = true;
			break;
		}
	}

	/* neighbor unknown until now, add to list */
	if (neighbor_known == false)
	{
		/* too many neighbors, delete oldest member (front) */
		if (neighbors.size() > FANET_NEIGHBOR_SIZE)
			neighbors.pop_front();

		neighbors.push_back(new FanetNeighbor(addr));
	}

	osMutexRelease(neighborMutex);
}

bool Fanet::isNeighbor(FanetMacAddr & addr)
{
	osMutexWait(neighborMutex, osWaitForever);
	bool found = false;

	for(auto *neighbor : neighbors)
	{
		if(neighbor->addr == addr)
		{
			found = true;
			break;
		}
	}

	osMutexRelease(neighborMutex);
	return found;
}

uint16_t Fanet::numNeighbors(void)
{
	osMutexWait(neighborMutex, osWaitForever);
	uint16_t nn = neighbors.size();
	osMutexRelease(neighborMutex);

	return nn;
}

FanetFrame *Fanet::broadcastIntended(void)
{
	const uint32_t current = osKernelSysTick();

	/* time for next service broadcast? */
	if(nextServiceBroadcast > current)
		return nullptr;

	FanetFrameService *sfrm = new FanetFrameService(fanet.hasInet, strlen(fanet.key)>0);
	debug_printf("Service Tx\n");

	/* wind */
	if(wind.sensorPresent)
		sfrm->setWind(wind.getDir_2min_avg(), wind.getSpeed_2min_avg(), wind.getSpeed_max());

	/* air */
	float temp, rh;
	sht2x.get(&temp, &rh);
	if(std::isnan(temp) == false)
		sfrm->setTemperature(temp);
	if(std::isnan(rh) == false)
		sfrm->setHumidity(rh);

	/* power */
	if(power::psu == false)
		sfrm->setSoc(power::isSufficiant() ? 100.0f : 30.0f);

	/* in case of a busy channel, ensure that frames from the tx fifo gets also a change */
	nextServiceBroadcast = current + 1000;
	return sfrm;
}

void Fanet::broadcastSuccessful(FanetFrame::FrameType_t type)
{
	const uint32_t current = osKernelSysTick();

	/* service frame */
	nextServiceBroadcast = current + (numNeighbors()/20+1) * FANET_TYPE4_TAU_MS * (!power::isSufficiant() + 1);
}

void Fanet::handleAcked(bool ack, FanetMacAddr &addr)
{
	if(frameToConsole)
		serialInt.handle_acked(ack, addr);

	ackRes = ack ? ACK : NACK;
	ackAddr = addr;							//note: address has to be set as second to prevent race conditions!
}

void Fanet::handleFrame(FanetFrame *frm)
{
	if(frameToConsole)
		serialInt.handleFrame(frm);

	/* decode */
	if(frm->dest == fmac.addr || frm->dest == FanetMacAddr())
	{
		osMutexWait(neighborMutex, osWaitForever);

		/* find neighbor */
		//note: prev called seenNeighbor() which ensures neighbor is already known
		FanetNeighbor *fn = nullptr;
		for(auto *neighbor : neighbors)
		{
			if(neighbor->addr == frm->src)
			{
				fn = neighbor;
				break;
			}
		}

		/* decode information */
		frm->decodePayload(fn);

		osMutexRelease(neighborMutex);
	}

	/* forward? */
	//note: do not forward already forwarded frames
	if(fmac.forwardAble(frm) || frm->dest == fmac.addr || frm->geoForward || sx1272_get_airlimit() > 0.8f ||
			fmac.txQueueHasFreeSlots() == false || power::isSufficiant() == false)
		return;

//	debug_printf("%02X:%04X->%02X:%04X %X: ", frm->src.manufacturer, frm->src.id, frm->dest.manufacturer, frm->dest.id, frm->type);

	/* get involved positions and time */
	Coordinate3D srcPos = Coordinate3D();
	Coordinate3D destPos = Coordinate3D();
	FanetNeighbor *neighbor = getNeighbor_locked(frm->src);
	if(neighbor == nullptr)						//should not happen, just in case...
		return;
	srcPos = neighbor->pos;
	const uint32_t lastFw = (frm->dest != FanetMacAddr()) ? neighbor->lastUniCastForwarded : neighbor->lastBrdCastForwarded;
	releaseNeighbors();

	const uint32_t current = osKernelSysTick();
	const uint32_t tau = ((numNeighbors()/10)+1)*30000;
	if(current > lastFw + 4000 && lastFw + tau > current)
	{
#ifdef DEBUG
//		printf("dropped, too often\n");
#endif
		return;
	}

	if(frm->dest != FanetMacAddr() && (neighbor = getNeighbor_locked(frm->dest)) != nullptr)
	{
		destPos = neighbor->pos;
		releaseNeighbors();
	}

	/* not directly frame specifically for us. geo-forward? */
	osMutexWait(geoFenceMutex, osWaitForever);

	bool doForward = false;
	for(uint16_t i=0; i<NELEM(geoFence) && doForward == false; i++)
	{
		if(geoFence[i].isActive() == false)
			continue;

		bool srcInside = geoFence[i].inside(srcPos);
		bool destInside = geoFence[i].inside(destPos);

		doForward |= (srcInside != destInside);
	}

	osMutexRelease(geoFenceMutex);

	if(doForward == false)
	{
#ifdef DEBUG
//		printf("dropped, not within fence\n");
#endif
		return;
	}

	/* prepare for retransmission */
	FanetFrame *fw = new FanetFrame(frm->dest);
	if(fw == nullptr)
		return;
	fw->src = frm->src;
	if(frm->ackRequested)
	{
		fw->ackRequested = FRM_ACK_SINGLEHOP;
		fw->numTx = 1;						//note: only 1 retransmission as we are transparent and do not handle ACKs
	}
	fw->geoForward = true;
	fw->signature = frm->signature;
	fw->setType(frm->type);
//	fw->nextTx = current + 3000;					//only for development
	if(frm->payloadLength > 0 && frm->payload != nullptr)
	{
		fw->payloadLength = frm->payloadLength;
		fw->payload = new uint8_t[fw->payloadLength];
		memcpy(fw->payload, frm->payload, fw->payloadLength * sizeof(uint8_t));
	}

	/* send if filled */
	if(fmac.transmit(fw) != 0)
	{
#ifdef DEBUG
		printf("problem while sending\n");
#endif
		delete fw;
	}
#ifdef DEBUG
	else
	{
		printf("forwarded\n");
	}
#endif

	/* update tx time at src */
	neighbor = getNeighbor_locked(frm->src);
	if(neighbor != nullptr)
	{
		if(frm->dest != FanetMacAddr())
			neighbor->lastUniCastForwarded = current;
		else
			neighbor->lastBrdCastForwarded = current;
		releaseNeighbors();
	}
}

/*
 * remote configuration
 */

bool Fanet::writeKey(char *newKey)
{
	if(newKey == nullptr || strlen(newKey) > sizeof(_key))
		return false;

	/* copy key */
	memset(_key, 0, sizeof(_key));
	snprintf(_key, sizeof(_key), "%s", newKey);

	/* determine page */
	FLASH_EraseInitTypeDef eraseInit = {0};
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.Page = FANET_KEYADDR_PAGE;
	eraseInit.NbPages = 1;

	/* erase */
	uint32_t sectorError = 0;
	if(HAL_FLASH_Unlock() != HAL_OK)
		return false;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	if(HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	/* write */
	for(unsigned int i=0; i<(sizeof(_key)+7)/8; i++)
	{
		/* build config */
		uint64_t addr_container = _key[i*8] | _key[i*8+1]<<8 | _key[i*8+2]<<16  | _key[i*8+3]<<24 |
				((uint64_t)_key[i*8+4])<<32 | ((uint64_t)_key[i*8+5])<<40  |
				((uint64_t)_key[i*8+6])<<48 | ((uint64_t)_key[i*8+7])<<56;

		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FANET_KEYADDR_BASE + i*8, addr_container) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return false;
		}
	}

	HAL_FLASH_Lock();
	return true;
}

void Fanet::loadKey(void)
{
	if(*(__IO uint64_t*)FANET_KEYADDR_BASE == UINT64_MAX)
		return;

	snprintf(_key, sizeof(_key), "%s", (char *)(__IO uint64_t*)FANET_KEYADDR_BASE);

	debug_printf("Key: '%s'\n", key);
}

bool Fanet::writePosition(Coordinate3D newPos, float newHeading)
{
	/* copy position */
	_position = newPos;
	_heading = newHeading;

	/* determine page */
	FLASH_EraseInitTypeDef eraseInit = {0};
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.Page = FANET_POSADDR_PAGE;
	eraseInit.NbPages = 1;

	/* erase */
	uint32_t sectorError = 0;
	if(HAL_FLASH_Unlock() != HAL_OK)
		return false;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	if(HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	/* write */

	uint64_t addr_container = ((uint64_t) ((uint8_t *)&position.latitude)[0]) << 0 | ((uint64_t) ((uint8_t *)&position.latitude)[1]) << 8 |
				((uint64_t) ((uint8_t *)&position.latitude)[2]) << 16 |	((uint64_t) ((uint8_t *)&position.latitude)[3]) << 24;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FANET_POSADDR_BASE + 0, addr_container) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	addr_container = ((uint64_t) ((uint8_t *)&position.longitude)[0]) << 0 | ((uint64_t) ((uint8_t *)&position.longitude)[1]) << 8 |
				((uint64_t) ((uint8_t *)&position.longitude)[2]) << 16 | ((uint64_t) ((uint8_t *)&position.longitude)[3]) << 24;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FANET_POSADDR_BASE + 8, addr_container) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	addr_container = ((uint64_t) ((uint8_t *)&position.altitude)[0]) << 0 |	((uint64_t) ((uint8_t *)&position.altitude)[1]) << 8 |
				((uint64_t) ((uint8_t *)&position.altitude)[2]) << 16 |	((uint64_t) ((uint8_t *)&position.altitude)[3]) << 24;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FANET_POSADDR_BASE + 16, addr_container) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	addr_container = ((uint64_t) ((uint8_t *)&heading)[0]) << 0 | ((uint64_t) ((uint8_t *)&heading)[1]) << 8 |
				((uint64_t) ((uint8_t *)&heading)[2]) << 16 | ((uint64_t) ((uint8_t *)&heading)[3]) << 24;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FANET_POSADDR_BASE + 24, addr_container) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	HAL_FLASH_Lock();
	return true;
}

void Fanet::loadPosition(void)
{
	if(*(__IO uint64_t*)FANET_POSADDR_BASE == UINT64_MAX)
		return;

	memcpy(&_position.latitude, (void *)(__IO uint64_t*) (FANET_POSADDR_BASE+0), sizeof(float));
	memcpy(&_position.longitude, (void *)(__IO uint64_t*) (FANET_POSADDR_BASE+8), sizeof(float));
	memcpy(&_position.altitude, (void *)(__IO uint64_t*) (FANET_POSADDR_BASE+16), sizeof(float));
	memcpy(&_heading, (void *)(__IO uint64_t*) (FANET_POSADDR_BASE+24), sizeof(float));

	debug_printf("Loc %.4f,%.4f,%.fm,%.fdeg\n", rad2deg(position.latitude), rad2deg(position.longitude), position.altitude, heading);
}

bool Fanet::writeReplayFeatures(uint16_t num)
{
	/* volatile feature -> no need to rewrite */
	if(num >= numReplayFeatureNoneVolatile)
		return true;

	/* determine page */
	FLASH_EraseInitTypeDef eraseInit = {0};
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.Page = FANET_RPADDR_PAGE;
	eraseInit.NbPages = 1;

	/* erase */
	uint32_t sectorError = 0;
	if(HAL_FLASH_Unlock() != HAL_OK)
		return false;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	if(HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	/* write */
	bool error = false;
	osMutexWait(replayFeatureMutex, osWaitForever);
	for(uint16_t i=0; i<numReplayFeatureNoneVolatile; i++)
		error |= !replayFeature[i].write(FANET_RPADDR_BASE + i*FLASH_PAGESIZE/numReplayFeatureNoneVolatile/8*8);
	osMutexRelease(replayFeatureMutex);

	HAL_FLASH_Lock();
	return !error;
}

void Fanet::loadReplayFeatures(void)
{
	osMutexWait(replayFeatureMutex, osWaitForever);
	for(uint16_t i=0; i<numReplayFeatureNoneVolatile; i++)
		replayFeature[i].load(FANET_RPADDR_BASE + i*FLASH_PAGESIZE/numReplayFeatureNoneVolatile/8*8);
	osMutexRelease(replayFeatureMutex);
}

bool Fanet::writeGeoFences(void)
{
	/* determine page */
	FLASH_EraseInitTypeDef eraseInit = {0};
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.Page = FANET_GEOFENCEADDR_PAGE;
	eraseInit.NbPages = 1;

	/* erase */
	uint32_t sectorError = 0;
	if(HAL_FLASH_Unlock() != HAL_OK)
		return false;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	if(HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}

	/* write */
	bool error = false;
	osMutexWait(geoFenceMutex, osWaitForever);
	for(uint16_t i=0; i<NELEM(geoFence); i++)
		error |= !geoFence[i].write(FANET_GEOFENCEADDR_BASE + i*FLASH_PAGESIZE/NELEM(geoFence)/8*8);
	osMutexRelease(geoFenceMutex);

	HAL_FLASH_Lock();
	return !error;
}

void Fanet::loadGeoFences(void)
{
	osMutexWait(geoFenceMutex, osWaitForever);
	for(uint16_t i=0; i<NELEM(geoFence); i++)
		geoFence[i].load(FANET_GEOFENCEADDR_BASE + i*FLASH_PAGESIZE/NELEM(geoFence)/8*8);
	osMutexRelease(geoFenceMutex);
}

void Fanet::init(FanetMac *fmac)
{
	/* read configuration */
	loadKey();
	loadPosition();
	loadReplayFeatures();
	loadGeoFences();
}

bool Fanet::isGeoForwarding(void)
{
	osMutexWait(geoFenceMutex, osWaitForever);
	bool doGeoForward = false;
	for(uint16_t i=0; i<NELEM(geoFence) && !doGeoForward; i++)
		doGeoForward |= geoFence[i].isActive();
	osMutexRelease(geoFenceMutex);

	return doGeoForward;
}

GeoFence &Fanet::getGeoFence_locked(uint16_t num)
{
	osMutexWait(geoFenceMutex, osWaitForever);
	return geoFence[num % NELEM(geoFence)];

}

void Fanet::releaseGeoFence(void)
{
	osMutexRelease(geoFenceMutex);
}

Replay &Fanet::getReplayFeature_locked(uint16_t num)
{
	osMutexWait(replayFeatureMutex, osWaitForever);
	return replayFeature[num % NELEM(replayFeature)];
}

bool Fanet::releaseReplayFeature(int16_t changedNum)
{
	osMutexRelease(replayFeatureMutex);

	if(changedNum < 0 || changedNum >= static_cast<int16_t>(NELEM(replayFeature)))
		return true;

	return writeReplayFeatures(changedNum);
}

Fanet fanet = Fanet();

