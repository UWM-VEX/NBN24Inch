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

	long *liftTime = malloc(sizeof(long));
	int *lifted = malloc(sizeof(int));

	*liftTime = 0;
	*lifted = 0;

	Lift newRamp = {.rampExtensionPort = rampExtensionPort, .liftPinPort = liftPinPort, .liftArmPort = liftArmPort, .liftTime = liftTime, .lifted = lifted};
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
	*lift.liftTime = millis();
	*lift.lifted = 1;
}

void runLift(Lift lift){
	if(*lift.lifted && millis() - *lift.liftTime > 1000) {
		digitalWrite(lift.liftArmPort, HIGH);
	}
}
