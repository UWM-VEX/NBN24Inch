/*
 * Ramp.c
 *
 *  Created on: Feb 9, 2016
 *      Author: Erik
 */

#include "main.h"

Lift initLift(int rampExtensionPort, int liftPinPort, int liftArmPort)
{
	pinMode(rampExtensionPort, OUTPUT);
	pinMode(liftPinPort, OUTPUT);
	pinMode(liftArmPort, OUTPUT);

	digitalWrite(rampExtensionPort, HIGH); //????
	digitalWrite(liftPinPort, LOW);
	digitalWrite(liftArmPort, LOW);

	Lift newRamp = {.rampExtensionPort = rampExtensionPort, .liftPinPort = liftPinPort, .liftArmPort = liftArmPort};
	return newRamp;
}


void deployRamp(Lift lift)
{
	digitalWrite(lift.rampExtensionPort, LOW);
}

void retractRamp(Lift lift)
{
	digitalWrite(lift.rampExtensionPort, HIGH);
}

void liftRobot(Lift lift){
	digitalWrite(lift.liftPinPort, HIGH);
	digitalWrite(lift.liftArmPort, HIGH);
}
