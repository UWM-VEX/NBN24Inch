/*
 * DriveToLine.c
 *
 *  Created on: Apr 13, 2016
 *      Author: Erik
 */
#include "main.h"

DriveToLine initDriveToLine(Drive drive, int speed, unsigned long timeout, int threshold)
{
	if(threshold == 0)
		threshold = 2000;

	unsigned long *startTime = malloc(sizeof(unsigned long));
	*startTime = 0;

	int *isFinished = malloc(sizeof(int));
	*isFinished = 0;

	DriveToLine newDrive = {.drive = drive, .speed = speed, .timeout = timeout,
			.threshold = threshold, .startTime = startTime, .isFinished = isFinished};
	return newDrive;
}

void driveToLine(DriveToLine step)
{
	if(*step.startTime == 0)
	{
		*step.startTime = millis();
	}

	int driveLeft;
	int driveRight;

	driveLeft = analogRead(step.drive.leftLineTracker) > step.threshold;
	driveRight = analogRead(step.drive.rightLineTracker) > step.threshold;

	tankDrive(step.drive, driveLeft * step.speed, driveRight * step.speed);

	if(!driveLeft && !driveRight)
	{
		tankDrive(step.drive, 0, 0);
		*step.isFinished = 1;
	}

	if(millis() - *step.startTime > step.timeout)
		*step.isFinished = 1;
}
