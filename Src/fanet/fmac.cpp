/*
 * fmac.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#include <math.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "spi.h"
#include "clamp.h"
#include "print.h"

#include "../misc/rnd.h"
#include "sx1272.h"
#include "fmac.h"
#include "frame/fack.h"

/*
 * FanetMac
 */

bool FanetMac::init(Fapp &app)
{
	myApp = &app;

	/* configure phy radio */
	if (sx1272_init(&hspi3) == false)
		return false;
	sx1272_setBandwidth(BW_250);
	sx1272_setSpreadingFactor(SF_7);
	sx1272_setCodingRate(CR_5);
	sx1272_setExplicitHeader(true);
	sx1272_setSyncWord(MAC_SYNCWORD);
	sx1272_setPayloadCrc(true);
	sx1272_setLnaGain(LNAGAIN_G1_MAX, true);
	sx1272_setIrqReceiver(frameRxWrapper);

	/* region specific. default is EU */
	sx_region_t region;
	region.channel = CH_868_200;
	region.dBm = 12;			//+2dB antenna gain (skytraxx/lynx) -> max allowed output (14dBm)
	sx1272_setRegion(region);

	/* enter sleep mode */
	sx1272_setArmed(false);

	/* address */
	//eraseAddr();
	_addr = readAddr();

	return true;
}

FanetMacAddr FanetMac::readAddr(void)
{
	uint64_t addr_container = *(__IO uint64_t*)MAC_ADDR_BASE;

	/* identify container */
	if((addr_container & MAC_ADDR_MAGIC_MASK) != MAC_ADDR_MAGIC)
	{
		debug_printf("WRN: No Addr set!\n");
		return FanetMacAddr();
	}

	return FanetMacAddr((addr_container>>16) & 0xFF, addr_container & 0xFFFF);
}

bool FanetMac::writeAddr(FanetMacAddr addr)
{
	/* test for clean storage */
	if(*(__IO uint64_t*)MAC_ADDR_BASE != UINT64_MAX)
		return false;

	/* build config */
	uint64_t addr_container = MAC_ADDR_MAGIC | (addr.manufacturer&0xFF)<<16 | (addr.id&0xFFFF);

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	HAL_StatusTypeDef flash_ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, MAC_ADDR_BASE, addr_container);
	HAL_FLASH_Lock();

	if(flash_ret == HAL_OK)
		_addr = addr;

	return (flash_ret == HAL_OK);
}

bool FanetMac::eraseAddr(void)
{
	/* determine page */
	FLASH_EraseInitTypeDef eraseInit = {0};
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.Page = MAC_ADDR_PAGE;
	eraseInit.NbPages = 1;

	uint32_t sectorError = 0;
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef ret = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
	HAL_FLASH_Lock();

	return (ret == HAL_OK && sectorError == UINT32_MAX);
}

/* this is executed in a non-linear fashion */
void FanetMac::receivedBuffer2Frame(int length)
{
	/* quickly read registers */
	num_received = sx1272_getFrame(rxFrame, sizeof(rxFrame));
	int rssi = sx1272_getRssi();

#if MAC_debug_mode > 1
	debug_printf("rx[%d@%d]=", num_received, rssi);
#ifdef DEBUG
	for(int i=0; i<num_received; i++)
		printf("%02x", rxFrame[i]);
	printf("\n");
#endif
#endif

	/* build frame from stream */
	FanetFrame *frm = new FanetFrame(num_received, rxFrame);
	frm->rssi = rssi;

	/* add to fifo */
	if (rxFifo.add(frm) < 0)
		delete frm;
}

/* wrapper to fit callback into c++ */
void FanetMac::frameRxWrapper(int length)
{
	fmac.receivedBuffer2Frame(length);
}

void FanetMac::setPower(bool pwr)
{
	if(sx1272_setArmed(pwr))
		_power = pwr;
	else
		_power = false;
}

/*
 * Generates ACK frame
 */
void FanetMac::ack(FanetFrame* frm)
{
#if defined(DEBUG) && MAC_debug_mode > 1
	debug_printf("gen ACK\n");
#endif

	/* generate reply */
	FanetFrame *ack = new FanetFrameAck(frm->src);

	/* only do a 2 hop ACK in case it was requested and we received it via a two hop link (= forward bit is not set anymore) */
	if (frm->ackRequested == FRM_ACK_TWOHOP && !frm->forward)
		ack->forward = true;

	/* add to front of fifo */
	//note: this will not fail by define
	if (txFifo.add(ack) != 0)
		delete ack;
}

/*
 * Processes stuff from rxFifo
 */
void FanetMac::handleRx()
{
	/* get first frame */
	FanetFrame *frm = rxFifo.front();
	if(frm == nullptr)
		return;

	/* update neighbor list */
	if(myApp != nullptr)
		myApp->seenNeighbor(frm->src);

	/* is the frame a forwarded one and is it still in our tx queue? */
	if(txFifo.update(frm))
	{
		delete frm;
		return;
	}

	/* broadcast or directed to us */
	if ((frm->dest == FanetMacAddr() || frm->dest == addr) && frm->src != addr)
	{
		/* a relevant frame */
		if(frm->type == FanetFrame::TYPE_ACK)
		{
			if (txFifo.removeDeleteAckedFrame(frm->src) && myApp != nullptr)
				myApp->handleAcked(true, frm->src);
		}
		else
		{
			/* generate ACK */
			if (frm->ackRequested)
				ack(frm);

			/* forward frame to app */
			if (myApp != nullptr)
				myApp->handleFrame(frm);
		}
	}
	else if(promiscuous && myApp != nullptr)
	{
		/* not a frame that was intended for us! */
		myApp->handleFrame(frm);
	}

	/* Forward frame */
	if (doForward && frm->forward && txFifo.size() < FANETMACFIFO_SIZE - 3 && frm->rssi <= MAC_FORWARD_MAX_RSSI_DBM
			&& (frm->dest == FanetMacAddr() || (myApp!=nullptr&& myApp->isNeighbor(frm->dest))) && sx1272_get_airlimit() < 0.5f)
	{
#if defined(DEBUG) && MAC_debug_mode > 1
		debug_printf("forward frame\n");
#endif
		/* prevent from re-forwarding */
		frm->forward = false;

		/* generate new tx time */
		frm->nextTx = osKernelSysTick() + rnd::get(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);

		/* add to list */
		txFifo.add(frm);
		return;		//note: do not delete frame as it's been added to txFifo
	}

	/* discard frame */
	delete frm;
}

/*
 * get a from from tx_fifo (or the app layer) and transmit it
 */
void FanetMac::handleTx()
{
	/* still in backoff or chip turned off*/
	if (osKernelSysTick() < csmaNextTx  || !sx1272_isArmed())
		return;

	/* find next send-able packet */
	/* this breaks the layering. however, this approach is much more efficient as the app layer now has a much higher priority */
	FanetFrame* frm;
	bool appTx = false;
	if (myApp != nullptr && (frm = myApp->broadcastIntended()) != nullptr)
	{
		/* app wants to broadcast */
		/* set forward bit in case of low neighbor count */
		frm->forward = myApp->numNeighbors() <= MAC_MAXNEIGHBORS_FOR_2HOP;

		appTx = true;
	}
	else if(sx1272_get_airlimit() < 0.9f && (frm = txFifo.getNextTx_lock()) != nullptr)
	{
		/* get a frame from the fifo */

		/* frame w/o a received ack and no more re-transmissions left */
		if (frm->ackRequested && frm->numTx <= 0)
		{
#if MAC_debug_mode > 0
			debug_printf("Frm%02X NACK!\n", frm->type);
#endif
			if (myApp != NULL)
				myApp->handleAcked(false, frm->dest);

			txFifo.removeDelete_nolock(frm);
			txFifo.release();
			return;
		}

		/* unicast frame w/o forwarding and it is not a direct neighbor */
		if (frm->forward == false && frm->dest != FanetMacAddr() && myApp != nullptr && myApp->isNeighbor(frm->dest) == false)
			frm->forward = true;

		appTx = false;
	}
	else
	{
		/* no frame found / no air time left */
		return;
	}

	/* serialize frame */
	uint8_t* buffer = nullptr;
	int blength = frm->serialize(buffer);
	if (blength < 0 || buffer == nullptr)
	{
#if MAC_debug_mode > 0
		debug_printf("problem serialize type %02X\n", frm->type);
#endif
		/* problem while assembling the frame */
		if (appTx)
		{
			delete frm;
		}
		else
		{
			txFifo.removeDelete_nolock(frm);
			txFifo.release();
		}
		return;
	}

#if MAC_debug_mode > 1
	debug_printf("tx%02X\n", frm->type);
#endif

#if MAC_debug_mode >= 4
	/* print hole packet */
	printf("pkt");
	for(int i=0; i<blength; i++)
	{
		printf("%02X", buffer[i]);
		if(i<blength-1)
			printf(":");
	}
	printf("\n");
#endif

	/* channel free and transmit? */
	//note: for only a few nodes around, increase the coding rate to ensure a more robust transmission
	int txRet = sx1272_sendFrame(buffer, blength, (myApp != nullptr && myApp->numNeighbors() < MAC_CODING48_THRESHOLD) ? CR_8 : CR_5);
	delete[] buffer;

	if (txRet == TX_OK)
	{
		/* transmission successful */

		if (appTx)
		{
			/* app tx */
			if(myApp != nullptr)
				myApp->broadcastSuccessful(frm->type);		//notify success
		}
		else
		{
			/* fifo tx */
			if (!frm->ackRequested)
			{
				/* remove frame from FIFO only if no ACK is expected*/
				txFifo.removeDelete_nolock(frm);
			}
			else
			{
				/* update next transmission */
				if (--frm->numTx > 0)
					frm->nextTx = osKernelSysTick() +
							(MAC_TX_RETRANSMISSION_TIME * (MAC_TX_RETRANSMISSION_RETRYS - frm->numTx));
				else
					frm->nextTx = osKernelSysTick() + MAC_TX_ACKTIMEOUT;		//will be removed next time
			}
		}

		/* prepare for a new transmission in */
		csmaBackoffExp = MAC_TX_BACKOFF_EXP_MIN;
		csmaNextTx = HAL_GetTick() + MAC_TX_MINPREAMBLEHEADERTIME_MS + (blength * MAC_TX_TIMEPERBYTE_MS);
	}
	else if (txRet == TX_RX_ONGOING || txRet == TX_TX_ONGOING)
	{
		/* unable to transmit, revert everything */
#if MAC_debug_mode > 0
		if(txRet == TX_RX_ONGOING)
			debug_printf("rx, abort.\n");
		else
			debug_printf("tx not done yet, abort.\n");
#endif

		/* channel busy, increment backoff exp */
		if(csmaBackoffExp < MAC_TX_BACKOFF_EXP_MAX)
			csmaBackoffExp++;

		/* next tx try */
		csmaNextTx = osKernelSysTick() + rnd::get(1 << (MAC_TX_BACKOFF_EXP_MIN - 1), 1 << csmaBackoffExp);

#if MAC_debug_mode > 1
		debug_printf("backoff %lums\n", csmaNextTx - osKernelSysTick());
#endif
	}
	else
	{
		/* ignoring TX_TX_ONGOING */
#if MAC_debug_mode > 0
		debug_printf("WAT: %d\n", txRet);
#endif
	}

	/* cleanup or release lock */
	if(appTx)
		delete frm;
	else
		txFifo.release();
}

FanetMac fmac = FanetMac();
