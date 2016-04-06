/*
 * Ramp.h
 *
 *  Created on: Feb 9, 2016
 *      Author: Erik
 */

#ifndef INCLUDE_LIFT_H_
#define INCLUDE_LIFT_H_

#include "main.h"

struct Lift{

	int rampExtensionPort;
	int liftPinPort;
	int liftArmPort;

}typedef Lift;

Lift initLift(int rampExtensionPort, int liftPinPort, int liftArmPort);
void deployRamp(Lift lift);
void liftRobot(Lift lift);



#endif /* INCLUDE_LIFT_H_ */


