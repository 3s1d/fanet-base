/*
 * fneighbor.cpp
 *
 *  Created on: Sep 20, 2018
 *      Author: sid
 */

#include <algorithm>

#include "cmsis_os.h"
#include "common.h"

#include "fneighbor.h"

FanetNeighbor::FanetNeighbor(const FanetMacAddr &addr) : addr(addr), pos(_pos), aircraft(_aircraft), status(_status)
{
#if defined(DEBUG) && defined(FNEIGHBOR_DEBUG)
	printf("fn%02x:%04x\n", addr.manufacturer, addr.id);
#endif

	/* new -> seen */
	lastSeen = osKernelSysTick();
}

void FanetNeighbor::setName(const char *newName, uint16_t len)
{
	snprintf(name, sizeof(name), "%.*s", len, newName);
}

void FanetNeighbor::setMessage(const char *text, uint16_t len)
{
	if(text == nullptr || len == 0)
		return;

	/* text already exists? is equal? */
	bool textEqual = false;
	if(msg != nullptr)
	{
		/* different text */
		if(strncmp(text, msg, std::max(len, (uint16_t)strlen(msg))))
			delete [] msg;
		else
			textEqual = true;
	}

	/* copy text */
	if(textEqual == false)
	{
		msg = new char[len+1];
		memcpy(msg, text, len);
		msg[len] = '\0';
	}
}

void FanetNeighbor::setTrackingType(FanetDef::status_t stat)
{
	_status = stat;
}
