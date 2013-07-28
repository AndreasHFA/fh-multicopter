/*
 * xBee_lib.h
 *
 *  Created on: Jun 20, 2013
 *      Author: Andreas
 */


#ifndef XBEE_LIB_H_
#define XBEE_LIB_H_

#include <stm32f30x.h>
#include <helperFunctions.h>

#define XBEE_USART 			USART3

/* Networking */
#define SSID				HexaWifi
#define Destination_IP 		'192.168.0.1'
#define Modul_IP			'192.168.0.2'
#define SUBNETMAST			'255.255.255.0'
#define UDP					0
#define TCP					1
#define IBSS_JOINER			0
#define IBSS_CREATOR		1
#define INFRASTRUCTURE		2

/* Serial Interface */
#define BR_9600				3
#define BR_115200			7
#define BR_230400			8


int setup_xBeeS6(void);


#endif /* xBee_LIB_H_ */
