/*
 * mdb_controll.hpp
 *
 *  Created on: May 18, 2023
 *      Author: OfficeLaptop
 */

#include "mdb_controll.hpp"
#include "canzero.hpp"


namespace mdb {


void mdbTemperatureReceiver(RxMessage& raw){
	uint8_t* buf = raw.rxBuf;
	float temperature = *reinterpret_cast<float*>(buf);
	uint8_t id = buf[4];
	switch(id){
	case 0:
		OD_Magnet_1_Temperature_set(temperature);
	case 1:
		OD_Magnet_2_Temperature_set(temperature);
	case 2:
		OD_Magnet_3_Temperature_set(temperature);
	case 3:
		OD_Magnet_4_Temperature_set(temperature);
	case 4:
		OD_Magnet_5_Temperature_set(temperature);
	case 5:
		OD_Magnet_6_Temperature_set(temperature);
	default:
		printf("ERROR: received invalid MDB_TX_Temperature Frame (invalid id)\n");
		ERR_OtherError_set();
	}
}

uint8_t mdbStates[6];

void mdbStateReceiver(RxMessage& raw){
	can::Message<can::messages::MDB_TX_State> msg{raw};
	uint8_t id = msg.get<can::signals::MDB_Id_State>();
	if(id >= 6){
		printf("ERROR: received invalid MDB_TX_State Frame (invalid id)\n");
		ERR_OtherError_set();
	}else{
		mdbStates[id] = msg.get<can::signals::MDB_State>();
	}
}

void init(){
	can::registerMessageReceiver<can::messages::MDB_TX_Temperature>(mdbTemperatureReceiver);
	can::registerMessageReceiver<can::messages::MDB_TX_State>(mdbStateReceiver);
}

void update(){
	// not 100% memory save but if od is inconsistant it will be corrected in the next iteration.
	for(uint8_t i=1;i<6;i++){
		if(mdbStates[i] != mdbStates[0]){
			OD_MdbState_set(MDB_STATE_INCONSISTANT);
			return;
		}
	}
	OD_MdbState_set(mdbStates[0]);
}


}
