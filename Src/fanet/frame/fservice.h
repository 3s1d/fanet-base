/*
 * fservice.h
 *
 *  Created on: Oct 31, 2018
 *      Author: sid
 */

#ifndef FANET_FRAME_FSERVICE_H_
#define FANET_FRAME_FSERVICE_H_

#define FANETFRAMESERVICE_INET			7
#define FANETFRAMESERVICE_TEMP			6
#define FANETFRAMESERVICE_WIND			5
#define FANETFRAMESERVICE_HUMIDITY		4
#define FANETFRAMESERVICE_PRESSURE		3
#define FANETFRAMESERVICE_REMOTECFGSUPPORT	2
#define FANETFRAMESERVICE_SOC			1

#include "../fframe.h"

class FanetFrameService : public FanetFrame
{
private:
	uint8_t header = 0;

	/* state */
	float temperature = 0.0f;
	float windDir_deg = 0.0f;
	float windSpeed_kmph = 0.0f;
	float windGust_kmph = 0.0f;
	float relHumidity = 0.0f;
	float preasure_hPa = 0.0f;
	float soc_percent = 0.0f;
public:
	/* state */
	bool hasInet = false;
	bool remoteCfgSupported = false;

	FanetFrameService(bool hasInet = false, bool remoteCfgSupported = false) :
		FanetFrame(), hasInet(hasInet), remoteCfgSupported(remoteCfgSupported) { _type = FanetFrame::TYPE_SERVICE; }

	/* define state */
	void setTemperature(float temp);
	void setWind(float dir, float speed, float gust);
	void setHumidity(float relHum);
	void setPressure(float hPa);
	void setSoc(float percent);

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor);
};



#endif /* FANET_FRAME_FSERVICE_H_ */
