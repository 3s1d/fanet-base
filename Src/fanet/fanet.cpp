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
#include "../hal/wind.h"
#include "../misc/rnd.h"
#include "../misc/sha1.h"
#include "../serial/serial_interface.h"
#include "sx1272.h"
#include "fanet.h"
#include "frame/fname.h"
#include "frame/fgndtracking.h"
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
//	if(mem != nullptr)
		return mem;
//	else
//		throw std::bad_alloc();
}

//void *operator new(std::size_t size, const std::nothrow_t &)
//{
//	return pvPortMalloc(size);
//}

void operator delete(void *p)
{
	vPortFree(p);
}


void Fanet::handle(void)
{
#if 0
	pwr management

	/* turn on rf chip */
	if(is_broadcast_ready(fmac.numNeighbors()))
	{
		/* enabling radio chip */
		if(!sx1272_isArmed())
		{
			last_framecount = framecount;
#if defined(DEBUG) || defined(DEBUG_SEMIHOSTING)
			printf("%u en (%d)\n", (unsigned int)HAL_GetTick(), pwr_suf);
#endif
		}
		sx1272_setArmed(true);
		last_ready_time = HAL_GetTick();
	}
	else if((last_ready_time + WSAPP_RADIO_UPTIME < HAL_GetTick() || !power_sufficiant()) &&
			fmac.tx_queue_depleted() && framecount != last_framecount)
	{
		/* disabling radio chip */
#if defined(DEBUG) || defined(DEBUG_SEMIHOSTING)
		if(sx1272_isArmed())
			printf("%u dis\n", (unsigned int)HAL_GetTick());
#endif
		sx1272_setArmed(false);
	}

	required???
	/* sleep for 10ms */
	//note: tick will wake us up every 1ms
	for(int i=0; i<10; i++)
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

#endif


	/* broadcast name */
//	if(settings.broadcastName && nextNameBroadcast < osKernelSysTick() && strlen(pilot.name))
//	{
//		FanetFrameName *ffn = new FanetFrameName();
//		if(fmac.transmit(ffn) != 0)
//			delete ffn;

//		nextNameBroadcast = osKernelSysTick() + FANET_TYPE2_TAU_MS;
//	}



	/* remove unavailable nodes */
	fanet.cleanNeighbors();
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
	if(nextServiceBroadcast <= current)
	{
		FanetFrameService *sfrm = new FanetFrameService(false, strlen(fanet.key)>0);		//no Inet but remoteCfg if key present
		debug_printf("tx service\n");
		if(wind.sensorPresent)
			sfrm->setWind(wind.getDir_2min_avg(), wind.getSpeed_2min_avg(), wind.getSpeed_max());
		sfrm->setSoc(power::isSufficiant() ? 100.0f : 30.0f);

		/* in case of a busy channel, ensure that frames from the tx fifo gets also a change */
		nextServiceBroadcast = current + FANET_TYPE4_MINTAU_MS;

		return sfrm;
	}

	/* time for next replay broadcast? */
	if(nextRfBroadcast > current)
		return nullptr;

	/* not valid replay feature -> find next */
	if(replayFeature[nextRfIdx].type == 0xFF)
	{
		/* find next index, max iterate once over array */
		for(uint16_t i=0; i<NELEM(replayFeature); i++)
		{
			/* overflow */
			nextRfIdx = (nextRfIdx+1) % NELEM(replayFeature);

			/* suitable content */
			if(replayFeature[nextRfIdx].type != 0xFF)
				break;
		}

		/* still not valid */
		if(replayFeature[nextRfIdx].type == 0xFF)
		{
			/* just in case, delay eval by 10sec */
			nextRfBroadcast = current + 10000;
			return nullptr;
		}
	}

	/* in case of a busy channel, ensure that frames from the tx fifo gets also a change */
	nextRfBroadcast = current + FANET_TYPE6_MINTAU_MS;

	/* broadcast replay */
	FanetFrame *frm = new FanetFrame();
	debug_printf("tx replay type:%x (idx:%d)\n", replayFeature[nextRfIdx].type, nextRfIdx);

	/* allow for replay/alternative information */
	//note: we'll might send crap if somebody altered the content since isBroadcastReady(). Thou' can't stop here...
	frm->setType(static_cast<FanetFrame::FrameType_t>(replayFeature[nextRfIdx].type));
	frm->payloadLength = replayFeature[nextRfIdx].payloadLength;
	frm->payload = new uint8_t[frm->payloadLength];
	memcpy(frm->payload, replayFeature[nextRfIdx].payload, frm->payloadLength);

	return frm;
}

void Fanet::broadcastSuccessful(FanetFrame::FrameType_t type)
{
	const uint32_t current = osKernelSysTick();

	/* service frame */
	if(type == FanetFrame::TYPE_SERVICE && nextServiceBroadcast-current < FANET_TYPE4_MINTAU_MS) 	//shortly released before
	{
		nextServiceBroadcast = current + (numNeighbors()/20+1) * FANET_TYPE4_TAU_MS * (!!power::isSufficiant() + 1);
		return;
	}

	//actually replayFeature[nextRfIdx].type == type... could be tested.

	/* Replay Frame */
	nextRfBroadcast = current + FANET_TYPE6_TAU_MS * (!!power::isSufficiant() + 1);

	/* find next index */
	for(uint16_t i=0; i<NELEM(replayFeature); i++)
	{
		/* overflow */
		if(++nextRfIdx >= NELEM(replayFeature))
		{
			nextRfIdx = 0;
			nextRfBroadcast += FANET_TYPE6_PAUSE_MS - FANET_TYPE6_TAU_MS;
		}

		if(replayFeature[nextRfIdx].type != 0xFF)
			break;
	}
}

bool Fanet::rcPosition(uint8_t *payload, uint16_t payload_length)
{
	/* remove our position */
	if(payload_length == 0)
		return writePosition(Coordinate3D(), 0.0f);

	/* to little information */
	if(payload_length < 9)
		return false;

	/* update position */
	Coordinate2D nPos;
	FanetFrame::payload2coord_absolute(payload, nPos);
	int16_t alt = ((uint16_t*)payload)[3]&0x7FF;
	if(((uint16_t*)payload)[3] & 1<<11)
		alt*=4;
	float head = (((float)payload[8])/256.0f) * 360.0f;
	return writePosition(Coordinate3D(nPos.latitude, nPos.longitude, alt), head);
}

//note: decoded here as it heavily interacts w/ class Fanet
bool Fanet::decodeRemoteConfig(FanetFrame *frm)
{
	if(frm == nullptr || frm->type != FanetFrame::TYPE_REMOTECONFIG || strlen(key) == 0)
		return false;

	/* check signature */
	SHA1_CTX ctx;
	sha1_init(&ctx);
	BYTE hash[SHA1_BLOCK_SIZE];

	/* pseudo header */
	uint8_t phdr[4];
	phdr[0] = frm->type & 0x3F;
	phdr[1] = frm->src.manufacturer;
	phdr[2] = frm->src.id & 0xFF;
	phdr[3] = frm->src.id >> 8;
	sha1_update(&ctx, phdr, sizeof(phdr));

	/* payload */
	sha1_update(&ctx, frm->payload, frm->payloadLength);

	/* pre shared key */
	sha1_update(&ctx, (uint8_t*) _key, std::min(strlen(_key), sizeof(_key)));

	/* sign */
	sha1_final(&ctx, hash);
	uint32_t signature;
	memcpy(&signature, hash, sizeof(uint32_t));

	/* ignore incorrectly signed frames */
	if(signature != frm->signature)
		return false;

	//tbr
	debug_printf("rc: %d", frm->payload[0]);

	/*
	 * Evaluate payload
	 */
	bool success = false;
	if(frm->payloadLength > 0 && frm->payload[0] == FANET_RC_POSITION)
	{
		success = rcPosition(&frm->payload[1], frm->payloadLength - 1);
	}
	else if(frm->payloadLength > 0 && frm->payload[0] >= FANET_RC_REPLAY_LOWER && frm->payload[0] <= FANET_RC_REPLAY_UPPER)
	{
		success = writeReplayFeature(frm->payload[0] - FANET_RC_REPLAY_LOWER, &frm->payload[1], frm->payloadLength - 1);

		/* broadcast latest feature */
		if(frm->payload[1] != 0xFF)		//type
		{
			nextRfIdx = frm->payload[0] - FANET_RC_REPLAY_LOWER;
			nextRfBroadcast = osKernelSysTick() + 500;
		}
	}

	/* generate RC ACK */
	if(success)
	{
		FanetFrame *rfrm = new FanetFrame(fmac.addr);
		if(rfrm == nullptr)
			return true;

		/* manually construct frame */
		rfrm->setType(FanetFrame::TYPE_REMOTECONFIG);
		rfrm->dest = frm->src;
		rfrm->forward = false;
		rfrm->ackRequested = false;
		rfrm->payloadLength = 2;
		rfrm->payload = new uint8_t[rfrm->payloadLength];
		rfrm->payload[0] = FANET_RC_ACK;
		rfrm->payload[1] = frm->payload[0];

		if(fmac.transmit(rfrm) != 0)
			delete frm;
	}

	return true;
}

void Fanet::handleAcked(bool ack, FanetMacAddr &addr)
{
	if(promiscuous)
		serialInt.handle_acked(ack, addr);

	ackRes = ack ? ACK : NACK;
	ackAddr = addr;							//note: address has to be set as second to prevent race conditions!
}


void Fanet::handleFrame(FanetFrame *frm)
{
	if(promiscuous)
		serialInt.handle_frame(frm);

	/* remote configuration frame */
	if(decodeRemoteConfig(frm) == true)
		return;

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
}

bool Fanet::writeReplayFeatures(void)
{
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
	for(uint16_t i=0; i<sizeof(replayFeature)/8; i++)
	{
		/* build config */
		uint64_t addr_container = ((uint64_t) ((uint8_t *)replayFeature)[i*8]) << 0 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 1]) << 8 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 2]) << 16 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 3]) << 24 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 4]) << 32 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 5]) << 40 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 6]) << 48 |
				((uint64_t) ((uint8_t *)replayFeature)[i*8 + 7]) << 56;

		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FANET_RPADDR_BASE + i*8, addr_container) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return false;
		}
	}
	HAL_FLASH_Lock();
	return true;
}

bool Fanet::writeReplayFeature(uint16_t num, uint8_t *payload, uint16_t len)
{
	if(num >= NELEM(replayFeature))
		return false;

	if(len < 1)
	{
		/* remove feature */
		replayFeature[num].type = 0xFF;		//empty
		replayFeature[num].payloadLength = 0;

		return writeReplayFeatures();
	}

	/* copy feature */
	replayFeature[num].type = payload[0];
	replayFeature[num].payloadLength = len-1;
	memcpy(replayFeature[num].payload, &payload[1],
			std::min((uint16_t) replayFeature[num].payloadLength, (uint16_t) sizeof(replayFeature[num].payload)));

	return writeReplayFeatures();
}

void Fanet::loadReplayFeatures(void)
{
	memcpy(replayFeature, (void *)(__IO uint64_t*)FANET_RPADDR_BASE, sizeof(replayFeature));
}

void Fanet::manualServiceSent(void)
{
	noAutoServiceBefore = osKernelSysTick() + 180000;						//disable for 3min
}


Fanet::Fanet() : Fapp(), position(_position), heading(_heading)
{
	/* read configuration */
	loadKey();
	loadPosition();
	loadReplayFeatures();
}

Fanet fanet = Fanet();
