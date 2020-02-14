/*
 * main.cpp
 *
 * @author Fran Pinto Santos
 * @author Gabriel Martín Blázquez
 *
 */

#include <iostream>
#include <dpy/gnssApi.h>
#include <dpyutils/logging.h>
#include <dpy/smsApi.h>
#include <dpy/geofencingApi.h>
#include <dpy/mcuApi.h>
#include <dpy/logconfigApi.h>
#include <dpy/smsApi.h>


#define OUR_F_SOGID "16"
#define OUR_F_ID 16


void leds_handler(boost::system::error_code ec, dpyMcu::LedsConfig & leds_struct) {
	std::string color;

	DPYLOG_INFO("LEDS HANDLER ENTERED!");

	if (ec.value() != 0) {
		DPYLOG_ERR("LED HANDLER get status Error = %v, %v", ec.value(), ec.message().c_str());
	} else {
        switch (leds_struct.color) {
            case dpyMcu::LedsColor::BLUE:
                color = "Blue";
            break;
            case dpyMcu::LedsColor::RED:
                color = "Red";
                break;
            case dpyMcu::LedsColor::GREEN:
                color = "Green";
                break;
            case dpyMcu::LedsColor::MAGENTA:
                color = "Magenta";
                break;
            case dpyMcu::LedsColor::YELLOW:
                color = "Yellow";
                break;
            case dpyMcu::LedsColor::CYAN:
                color = "Cyan";
                break;
            case dpyMcu::LedsColor::UNDEFINED:
                color = "Undefined";
                break;
        }

		DPYLOG_INFO("Semaphore color has changed to %s", color);
	}
}

void geofence_add_polygon_handler(boost::system::error_code ec) {
	if (ec.value() != 0) {
	    	DPYLOG_ERR("GEOFENCE ADD POLYGON get status Error = %v, %v", ec.value(), ec.message().c_str());
	} else {
		DPYLOG_INFO("Polygon has been correctly added to the geofencing");
	}
}

void periodic_position_handler(boost::system::error_code error_code, GnssPosition next_position)
{
    if (error_code.value() != 0) {
    	DPYLOG_ERR("GNSS get status Error = %v, %v", error_code.value(), error_code.message().c_str());
    } else {


        std::map<dpyGnssPosition::QualityIndicator, std::string> fix_to_string_map;
        fix_to_string_map[dpyGnssPosition::NO_FIX] = "NO FIX";
        fix_to_string_map[dpyGnssPosition::AUTONOMOUS_FIX] = "AUTONOMOUS FIX";
        fix_to_string_map[dpyGnssPosition::DIFFERENTIAL_FIX] = "DIFFERENTIAL FIX";
        fix_to_string_map[dpyGnssPosition::RTK_FIXED] = "RTK FIXED";
        fix_to_string_map[dpyGnssPosition::RTK_FLOAT] = "RTK FLOAT";
        fix_to_string_map[dpyGnssPosition::ESTIMATED_DEAD_RECKONING_FIX] = "ESTIMATED DEAD RECKONING FIX";

        DPYLOG_INFO("FIX status is %v", fix_to_string_map[next_position.fix_quality]);
        DPYLOG_INFO("Satellites used [%v]", next_position.sky.satinuse);
        DPYLOG_INFO("Date [%v:%v:%v]", next_position.date.tm_mday, next_position.date.tm_mon, next_position.date.tm_year + 1900);
        DPYLOG_INFO("time [%v:%v:%v]", next_position.date.tm_hour, next_position.date.tm_min, next_position.date.tm_sec);


        if (next_position.fix_quality != dpyGnssPosition::NO_FIX) {
        	DPYLOG_WARN("Position coordinates: [%v,%v]", next_position.lat_deg, next_position.lon_deg);
        	DPYLOG_WARN("Altitude above/below mean sea level is: %v meters", next_position.alt_m);
        	DPYLOG_WARN("Horizontal Dilution Of Precision is: %v", next_position.HDOP);
        	DPYLOG_WARN("Speed over the ground is: %v kilometres per hour", next_position.speed_kmh);
        	DPYLOG_WARN("Course over the ground (track angle) is: %v degrees", next_position.dir_deg);
        }
    }
}

void sms_confirmation_handler(boost::system::error_code error_code) {

}

class MyGeoObserver : public IgeofencingObserver {
	virtual void NewEvent(
			dpyGeofencing::GeofenceEvent type,
			std::string sogid,
			int id,
			dpyGeofencing::Geofence geofence,
			double angle) {

		// This class is to control the leds
		Mcu * mcu = new Mcu();
		Sms * smsManager = new Sms();

		std::uint32_t period = 0;
		std::uint32_t duty_cycle = 1;

		if (0 == sogid.compare(OUR_F_SOGID)) {
			DPYLOG_INFO("A new event has been received!");
			if (type == dpyGeofencing::GeofenceEvent::ENTER) {
				// Put the RED light
				DPYLOG_INFO("Semaphore state changing to RED");
				mcu->asyncSetLedsConfig(leds_handler, dpyMcu::RED, period, duty_cycle);

				// Sens SMS that has entered the zone
				smsManager->sendSms("The bus has entered into the zone", "655036904", sms_confirmation_handler);
			} else if(type == dpyGeofencing::GeofenceEvent::EXIT) {
				// Put the GREEN light
				DPYLOG_INFO("Semaphore state changing to GREEN");
				mcu->asyncSetLedsConfig(leds_handler, dpyMcu::GREEN, period, duty_cycle);

				// Sens SMS that has entered the zone
				smsManager->sendSms("The bus has exited the zone", "655036904", sms_confirmation_handler);
			}
		}
	}
};

void set_level_handler(boost::system::error_code error_code){

}


int main(int argc, char **argv)
{
	// Logger
	LogConfig logConfig;
	Logging log("app16");
	std::string appName = "app16";
	std::string level = "Debug";
	logConfig.asyncSetLevel(appName, level, set_level_handler);

	DPYLOG_INFO("Starting App");
	Gnss dpyGnss;
	dpyGnss.getPeriodicPosition_S(periodic_position_handler);

	// Instanciate geofencing
	Geofencing geofencing;

	// Observer
	MyGeoObserver * myGeoObserver = new MyGeoObserver();

	// Geofence vector
	std::vector<std::pair<double, double>> positionList;

	// Push the coordinates into the vector
	positionList.push_back(std::pair<double, double>(35.1714917, 33.3503306));
	positionList.push_back(std::pair<double, double>(35.1659056, 33.3503222));
	positionList.push_back(std::pair<double, double>(35.1692000, 33.3576694));
	positionList.push_back(std::pair<double, double>(35.1659056, 33.3581444));

	// Add square to Geofence
	geofencing.asyncAddPolygon(geofence_add_polygon_handler, OUR_F_SOGID, OUR_F_ID, "geofence", true, positionList);

	// Subscribe to Geofencing events
	geofencing.subscribePeriodicPosition(myGeoObserver);

	while(1){
	   sleep(1);
	}
}
