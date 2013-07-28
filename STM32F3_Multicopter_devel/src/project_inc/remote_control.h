/*
 * remote_control.h
 *
 *  Created on: Apr 28, 2013
 *      Author: franz
 */

#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

#include <stm32f30x.h>

void CalcRXValues(unsigned char *msgPointer, int16_t adjValue);

#endif /* REMOTE_CONTROL_H_ */
