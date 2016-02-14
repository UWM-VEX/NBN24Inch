/*
 * Ramp.c
 *
 *  Created on: Feb 9, 2016
 *      Author: Erik
 */

#include "main.h"

Ramp initRamp(int port)
{
	pinMode(port, OUTPUT);

	digitalWrite(port, LOW);

	Ramp newRamp = {port};
	return newRamp;
}

void deployRamp(Ramp ramp)
{
	digitalWrite(ramp.port, HIGH);
}
