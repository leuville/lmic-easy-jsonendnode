/*
 * Common initializations for projects
 */

#pragma once

#include <Arduino.h>
#include <LMICWrapper.h>

namespace leuville {
namespace lora {

/*
 * ---------------------------------------------------------------------------------------
 * PIN mappings
 *
 * https://github.com/bborncr/FeatherM0-TTN-Getting-Started
 * https://learn.sparkfun.com/tutorials/sparkfun-samd21-pro-rf-hookup-guide/programming-the-samd21-pro-rf
 * 
 * ---------------------------------------------------------------------------------------
 */

enum BOARD { 
	ADAFRUIT_FEATHER_M0, 
	SPARKFUN_PRORF,
	_BOARD_COUNT 
};

const lmic_pinmap lmic_pinmaps[] = {
	/* ADAFRUIT_FEATHER_M0 	*/		
	{	
		.nss = 8, 
		.rxtx = LMIC_UNUSED_PIN, 
		.rst = 4, 
		.dio = {3,  6, LMIC_UNUSED_PIN}, 
		.rxtx_rx_active = 0, 
		.rssi_cal = 8, 			// 10 ?
		.spi_freq = 8000000 
	},

	/* SPARKFUN_PRORF */	
	{ 
		.nss = 12, 
		.rxtx = LMIC_UNUSED_PIN, 
		.rst = 7, 
		.dio = {6, 10, 11}
	}				
};

/*
 * ---------------------------------------------------------------------------------------
 * LoRaWAN configurations
 * ---------------------------------------------------------------------------------------
 */

enum ADR : bool { ON = true, OFF = false };

enum Network: u4_t { EXP = 0x000000, EXP_ADR = 0x000001, TTN = 0x000013, ORANGE = 0x00000F };

enum Config { 
	DEVICE1, DEVICE2
};

OTAAId id[] = {
	// APPEUI				// DEVEUI			// APPKEY
	{ "FF00000000000001", "FF00000000000001", "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF01" }, // DEVICE1 
	{ "FF00000000000002", "FF00000000000002", "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF02" } 	// DEVICE2
};
/*
 * ---------------------------------------------------------------------------------------
 * Configure channels according to network
 * 
 * https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
 * ---------------------------------------------------------------------------------------
 */
void configureNetwork(Network netid, bool adr = true) {
	switch (netid) {
		case Network::TTN:
			LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band 
			break;
		case Network::ORANGE:		
			if (!adr) {
				LMIC_setDrTxpow(EU868_DR_SF12, 20);
				LMIC_setupChannel(8, 	865500000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_MILLI); 	// LC9
				LMIC_setupChannel(9, 	865700000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_MILLI); 	// LC10
				LMIC_setupChannel(10, 	865900000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_MILLI); 	// LC11
				LMIC_setupChannel(11, 	866100000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_CENTI); 	// LC12
				LMIC_setupChannel(12, 	866300000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_CENTI); 	// LC13
				LMIC_setupChannel(13, 	866500000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_CENTI); 	// LC14
				LMIC_setupChannel(14, 	866900000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_CENTI); 	// LC15
				LMIC_setupChannel(15, 	867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), 	BAND_CENTI); 	// LC16
			}
			break;
	}
}

/*
 * ---------------------------------------------------------------------------------------
 * Configure LMIC features after joined specific network
 * ---------------------------------------------------------------------------------------
 */
void postJoinSetup(u4_t netid) {
	switch (netid) {
		case Network::EXP:
		case Network::EXP_ADR:
			LMIC_setLinkCheckMode(1);	
			break; 
		case Network::TTN:
			LMIC_setLinkCheckMode(0);	
			LMIC.dn2Dr = SF9;
			break; 
		default:
			LMIC_setLinkCheckMode(1);
	}
}

}
}