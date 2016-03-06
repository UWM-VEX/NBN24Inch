/*
 * PIDController.c
 *
 *  Created on: Jun 15, 2015
 *      Author: Erik
 */
#include "main.h"

PIDController initPIDController(double kP, double kI, double kD, double kF, int setPoint, int errorEpsilon)
{
	PIDController newController = {kP, kI, kD, kF, setPoint, 0, millis(),
			0, errorEpsilon};

	return newController;
}

void PIDsetkP(PIDController *controller, double kP)
{
	(*controller).kP = kP;
}

void PIDsetkI(PIDController *controller, double kI)
{
	(*controller).kI = kI;
}

void PIDsetkD(PIDController *controller, double kD)
{
	(*controller).kD = kD;
}

void PIDsetkF(PIDController *controller, double kF)
{
	(*controller).kF = kF;
}

void PIDsetSetPoint(PIDController *controller, double setPoint)
{
	(*controller).setPoint = setPoint;
}

void PIDsetErrorEpsilon(PIDController *controller, double errorEpsilon)
{
	(*controller).errorEpsilon = errorEpsilon;
}

int PIDgetPContribution(PIDController *controller, int processVariable)
{
	return (int) ((*controller).kP * ((*controller).setPoint - processVariable));
}

int PIDgetIContribution(PIDController *controller, int processVariable)
{
	int error = (*controller).setPoint - processVariable;

	if(abs(error) < (*controller).errorEpsilon)
	{
		(*controller).sumOfError = 0;
		puts("Error cleared.");
		return 0;
	}
	else
	{
		puts("Error accumulated.");
		long timeDiff = millis() - (*controller).lastTime;
		int newError = (int) timeDiff * error;
		(*controller).sumOfError += newError;
		printf("Sum of Error: %d\n", (*controller).sumOfError);
		long numToReturn = (long) (*controller).sumOfError * (*controller).kI;
		numToReturn = limit(numToReturn, 2000000000, -2000000000);
		return (int) numToReturn;
	}
}

int PIDgetDContribution(PIDController *controller, int processVariable)
{
	int error = (*controller).setPoint - processVariable;
	int timeDiff = (int) (millis() - (*controller).lastTime);
	int errorDiff = error - (*controller).lastError;

	double slope = (double) (errorDiff / timeDiff);

	return (int) (slope * (*controller).kD);
}

int PIDgetFContribution(PIDController *controller)
{
	return (int) ((*controller).kF * (*controller).setPoint);
}

int PIDRunController(PIDController *controller, int processVariable)
{
	int pContribution = PIDgetPContribution(controller, processVariable);
	int iContribution = PIDgetIContribution(controller, processVariable);
	int dContribution = PIDgetDContribution(controller, processVariable);
	int fContribution = PIDgetFContribution(controller);

	(*controller).lastError = (*controller).setPoint - processVariable;
	(*controller).lastTime = millis();

	printf("PV: %d\n", processVariable);
	printf("SP: %d\n", (*controller).setPoint);
	printf("I: %d\n", iContribution);

	return pContribution + iContribution + dContribution + fContribution;
}
