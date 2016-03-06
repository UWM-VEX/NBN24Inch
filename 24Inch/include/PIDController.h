/*
 * PIDController.h
 *
 *  Created on: Jun 15, 2015
 *      Author: Erik
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

struct PIDController{

	double kP;
	double kI;
	double kD;
	double kF;
	int setPoint;
	int lastError;
	long lastTime;
	int sumOfError;
	int errorEpsilon;

}typedef PIDController;

PIDController initPIDController(double kP, double kI, double kD, double kF, int setPoint, int errorEpsilon);
void PIDsetkP(PIDController *controller, double kP);
void PIDsetkI(PIDController *controller, double kP);
void PIDsetkD(PIDController *controller, double kP);
void PIDsetkF(PIDController *controller, double kP);
void PIDsetSetPoint(PIDController *controller, double setPoint);
void PIDsetErrorEpsilon(PIDController *controller, double errorEpsilon);
int PIDgetPContribution(PIDController *controller, int processVariable);
int PIDgetIContribution(PIDController *controller, int processVariable);
int PIDgetDContribution(PIDController *controller, int processVariable);
int PIDgetFContribution(PIDController *controller);
int PIDRunController(PIDController *controller, int processVariable);

#endif /* PIDCONTROLLER_H_ */
