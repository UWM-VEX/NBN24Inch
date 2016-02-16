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

	int port;

}typedef Ramp;

Ramp initRamp(int port);
void deployRamp(Ramp ramp);

#endif /* INCLUDE_RAMP_H_ */
