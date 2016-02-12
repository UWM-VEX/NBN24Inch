/*
 * IncrementalController.c
 *
 *  Created on: Feb 10, 2016
 *      Author: Erik
 */

#include "main.h"

IncrementalController initIncrementalController(RedEncoder *encoder)
{
	IncrementalController newController = {encoder, micros(), 0.0, 0};
	return newController;
}

int runIncrementalController(IncrementalController *controller, double setPoint)
{
	if(micros() - (*controller).lastReadTime > 100000)
	{
		double currentVelocity = updateRedEncoder((*controller).encoder);
		double error = currentVelocity - setPoint;

		//lcdPrint(uart1, 1, "%f", currentVelocity);

		if(absDouble(error) < 2)
		{
			//Controller within deadband, keep at correct speed
		}
		else if(error > 0)
		{
			if(error > 10)
			{
				(*controller).output -= .5;
			}
			else
			{
				(*controller).output -= .2;
			}
		}
		else
		{
			if(error < -10)
			{
				(*controller).output += 0.5;
				lcdSetText(uart1, 2, "Incrementing");
			}
			else
			{
				(*controller).output += .2;
			}
		}

		(*controller).error = error;
		(*controller).output = limitDouble((*controller).output, 127.0, 0.0);
	}

	lcdPrint(uart1, 1, "%f", (*controller).output);

	int output = (int) (*controller).output;

	return output;
}
