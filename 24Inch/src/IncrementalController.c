/*
 * IncrementalController.c
 *
 *  Created on: Feb 10, 2016
 *      Author: Erik
 */

#include "main.h"

IncrementalController initIncrementalController(RedEncoder *encoder)
{
	IncrementalController newController = {encoder, micros(), 0, 0};
	return newController;
}

int runIncrementalController(IncrementalController *controller, double setPoint)
{
	if(micros() - (*controller).lastReadTime > 100000)
	{
		double currentVelocity = updateRedEncoder((*controller).encoder);
		double error = currentVelocity - setPoint;

		if(absDouble(currentVelocity) < 10 || absDouble(currentVelocity) > 1000)
		{
			//Unreasonable value from encoder, do not change output
		}
		else if(absDouble(error) < 2)
		{
			//Controller within deadband, keep at correct speed
		}
		else if(error > 0)
		{
			if(error > 10)
			{
				(*controller).output -= 5;
			}
			else
			{
				(*controller).output -= 2;
			}
		}
		else
		{
			if(error < -10)
			{
				(*controller).output += 5;
			}
			else
			{
				(*controller).output += 2;
			}
		}

		(*controller).error = error;
		(*controller).output = limit((*controller).output, 127, 50);
	}

	return (*controller).output;
}
