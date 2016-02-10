/*
 * Ramp.h
 *
 *  Created on: Feb 9, 2016
 *      Author: Erik
 */

#ifndef INCLUDE_RAMP_H_
#define INCLUDE_RAMP_H_

#include "main.h"

struct Ramp{

	int upperPort;
	int lowerPort;
	int delayTime;
	long deployTime;
	int deployed;

}typedef Ramp;

Ramp initRamp(int upperPort, int lowerPort, int delayTime);
void deployRamp(Ramp *ramp);
void runRamp(Ramp *ramp);

#endif /* INCLUDE_RAMP_H_ */
