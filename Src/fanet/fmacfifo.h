/*
 * fmacfifo.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FMACFIFO_H_
#define VARIO_HAL_FANET_FMACFIFO_H_

#define FANETMACFIFO_SIZE				8

#include <list>

#include "cmsis_os.h"

#include "fmac.h"
#include "fframe.h"

class FanetMacFifo
{
private:
	std::list<FanetFrame*> fifo;
	osMutexDef(mutex);
	osMutexId mutex = osMutexCreate(osMutex(mutex));

public:
	FanetFrame* front();
	int add(FanetFrame *frm);
	uint16_t size(void);

	/* used for received frames to update tx fifo */
	bool update(FanetFrame *frm);

	/* used for removing any pending frame of the txFifo that waits on an ACK from a host (avoid any further resents) */
	bool removeDeleteAckedFrame(FanetMacAddr &dest);

	/* delete frame from list */
	bool removeDelete(FanetFrame *frm);
	bool removeDelete_nolock(FanetFrame *frm);

	/* get next frame which can be sent out */
	FanetFrame* getNextTx_lock();
	void release(void) { osMutexRelease(mutex); }

};



#endif /* VARIO_HAL_FANET_FMACFIFO_H_ */
