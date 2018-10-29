/*
 * fmacaddr.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FMACADDR_H_
#define VARIO_HAL_FANET_FMACADDR_H_

#include <stdint.h>
#include <stdio.h>


/*
 * 0, 0 == Broadcast
 */

class FanetMacAddr
{
public:
	uint8_t manufacturer;
	uint16_t id;

	FanetMacAddr(uint8_t manufacturer_addr, uint16_t id_addr): manufacturer(manufacturer_addr), id(id_addr) {};
	FanetMacAddr() : manufacturer(0), id(0) {};									//broadcast address
	FanetMacAddr(const FanetMacAddr &ma) : manufacturer(ma.manufacturer), id(ma.id) {};

	inline bool operator == (const FanetMacAddr& rhs) const { return ((id == rhs.id) && (manufacturer == rhs.manufacturer));};
	inline bool operator != (const FanetMacAddr& rhs) const { return ((id != rhs.id) || (manufacturer != rhs.manufacturer));};
	void toString(char *str, uint16_t len) const { snprintf(str, len, "%02X:%04X", manufacturer, id); }
};



#endif /* VARIO_HAL_FANET_FMACADDR_H_ */
