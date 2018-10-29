/*
 * fmacfifo.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#include <stdint.h>

#include "print.h"

#include "fmac.h"
#include "fmacfifo.h"
#include "../misc/rnd.h"


/*
 * MacFifo
 */

/* add frame to fifo */
int FanetMacFifo::add(FanetFrame *frm)
{
	osMutexWait(mutex, osWaitForever);

	/* buffer full */
	/* note: ACKs will always fit */
	if (fifo.size() >= FANETMACFIFO_SIZE && frm->type != FanetFrame::TYPE_ACK)
	{
		osMutexRelease(mutex);
		return -1;
	}

	/* only one ack_requested from us to a specific address at a time is allowed in the queue */
	//in order not to screw with the awaiting of ACK
	if (frm->ackRequested)
	{
		for(auto *ffrm : fifo)
		{
			if (frm->ackRequested && ffrm->src == fmac.addr && ffrm->dest == frm->dest)
			{
				osMutexRelease(mutex);
				return -2;
			}
		}
	}

	if (frm->type == FanetFrame::TYPE_ACK)
		/* add to front */
		fifo.push_front(frm);
	else
		/* add to tail */
		fifo.push_back(frm);

	osMutexRelease(mutex);
	return 0;
}


uint16_t FanetMacFifo::size(void)
{
	osMutexWait(mutex, osWaitForever);
	uint16_t size = fifo.size();
	osMutexRelease(mutex);

	return size;
}

FanetFrame* FanetMacFifo::front()
{
	osMutexWait(mutex, osWaitForever);

	/* empty list must be checked, otherwise behavior is undefined on empty lists */
	if(fifo.size() == 0)
	{
		osMutexRelease(mutex);
		return nullptr;
	}

	/* get first entry and delete it form list */
	FanetFrame *frm = fifo.front();
	fifo.pop_front();

	osMutexRelease(mutex);

	return frm;
}

/* used for received frames to update tx fifo */
bool FanetMacFifo::update(FanetFrame *frm)
{
	osMutexWait(mutex, osWaitForever);

	/* find suitable frame */
	FanetFrame *frmfifo = nullptr;
	for(auto *frmlist : fifo)
	{
		if(*frmlist != *frm)
			continue;

		frmfifo = frmlist;
	}

	/* found frame -> update */
	if(frmfifo != nullptr)
	{
		/* frame already in tx queue */
		if (frm->rssi > frmfifo->rssi + MAC_FORWARD_MIN_DB_BOOST)
		{
			/* somebody broadcasted it already towards our direction */
#if defined(DEBUG) && MAC_debug_mode > 0
			debug_printf("rx frame better than org. dropping both.\n");
#endif
			/* received frame is at least 20dB better than the original -> no need to rebroadcast */
			fifo.remove(frmfifo);
			delete frmfifo;			//note: leave frmfifo != nullptr for return
		}
		else
		{
#if defined(DEBUG) && MAC_debug_mode > 0
			debug_printf("adjusting tx time\n");
#endif
			/* adjusting new departure time */
			frmfifo->nextTx = osKernelSysTick() + rnd::get(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
		}
	}

	osMutexRelease(mutex);
	return frmfifo != nullptr;
}

/* used for removing any pending frame of the txFifo that waits on an ACK from a host (avoid any further resents) */
bool FanetMacFifo::removeDeleteAckedFrame(FanetMacAddr &dest)
{
	bool found = false;
	osMutexWait(mutex, osWaitForever);

	for(auto *frm : fifo)
	{
		if(frm->ackRequested && frm->dest == dest)
		{
			fifo.remove(frm);
			delete frm;
			found = true;
		}
	}

	osMutexRelease(mutex);
	return found;
}

/* delete frame from list */
bool FanetMacFifo::removeDelete_nolock(FanetFrame *frm)
{
	bool found = false;

	for(auto *lfrm : fifo)
	{
		if (frm == lfrm)
		{
			fifo.remove(lfrm);
			delete lfrm;
			found = true;
			break;
		}
	}

	return found;
}
bool FanetMacFifo::removeDelete(FanetFrame *frm)
{
	osMutexWait(mutex, osWaitForever);

	bool found = removeDelete_nolock(frm);

	osMutexRelease(mutex);
	return found;
}

FanetFrame* FanetMacFifo::getNextTx_lock()
{
	osMutexWait(mutex, osWaitForever);

	FanetFrame *nextfrm = nullptr;
	for (auto *lfrm : fifo)
	{
		if(lfrm->nextTx < osKernelSysTick())
		{
			nextfrm = lfrm;
			break;
		}
	}

	/* release in case of no frame found */
	if(nextfrm == nullptr)
		osMutexRelease(mutex);

	return nextfrm;
}
