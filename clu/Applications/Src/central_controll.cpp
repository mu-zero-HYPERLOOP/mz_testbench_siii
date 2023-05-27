/*
 * central_controll.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#include "central_controll.hpp"
#include "mdb_remote.hpp"

namespace central_controll {

/*
 * executed when the clu starts up. (once).
 */
void init(){

}

/*
 * executed around every 5ms.
 */
void update(){
	// mdb::getAirGap(0); get air gap of mdb 1
	// mdb::setTargetAirGap(0, 0.0); set target airgap of mdb 1 to 0.0

	// there is a good chance that controll signals might have to be send to the mdbs. (something like a launch signal).
}

}
