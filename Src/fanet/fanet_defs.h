/*
 * fanet_defs.h
 *
 *  Created on: Sep 19, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FANET_DEFS_H_
#define VARIO_HAL_FANET_FANET_DEFS_H_

namespace FanetDef
{
	enum aircraft_t : uint16_t
	{
		otherAircraft = 0,
		paraglider = 1,
		hangglider = 2,
		balloon = 3,
		glider = 4,
		poweredAircraft = 5,
		helicopter = 6,
		uav = 7,

		/* internal states */
		NONE = UINT16_MAX
	};

	enum status_t : uint16_t
	{
		otherStatus = 0,
		hiking = 1,
		vehilce = 2,
		bike = 3,
		boot = 4,
		needAride = 8,

		needTechnicalAssistance = 12,
		needMedicalHelp = 13,
		distressCall = 14,
		distressCallAuto = 15,				//max number

		/* internal states */
		AUTO = (hiking|0x8000),				//auto fallback to hiking
		SERVICE = UINT16_MAX-1,				//for fanet neighbors
		AIRBORNE = UINT16_MAX,				//for fanet neighbors
	};
}



#endif /* VARIO_HAL_FANET_FANET_DEFS_H_ */
