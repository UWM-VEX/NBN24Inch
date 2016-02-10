/*
 * Ramp.c
 *
 *  Created on: Feb 9, 2016
 *      Author: Erik
 */

#include "main.h"

Ramp initRamp(int upperPort, int lowerPort, int delayTime)
{
	pinMode(upperPort, OUTPUT);
	pinMode(lowerPort, INPUT);

	digitalWrite(upperPort, LOW);
	digitalWrite(lowerPort, LOW);

	Ramp newRamp = {upperPort, lowerPort, delayTime, 0, 0};
	return newRamp;
}

void deployRamp(Ramp *ramp)
{
	(*ramp).deployed = 1;
	(*ramp).deployTime = millis();

	digitalWrite((*ramp).upperPort, HIGH);
}

void runRamp(Ramp *ramp)
{
	if((*ramp).deployed && (*ramp).deployTime - millis() > (*ramp).delayTime)
	{
		digitalWrite((*ramp).lowerPort, HIGH);
	}
}
