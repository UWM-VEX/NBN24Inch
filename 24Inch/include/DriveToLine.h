/*
 * DriveToLine.h
 *
 *  Created on: Apr 13, 2016
 *      Author: Erik
 */

#ifndef INCLUDE_DRIVETOLINE_H_
#define INCLUDE_DRIVETOLINE_H_

struct DriveToLine{

	Drive drive;
	int speed;
	int threshold;
	unsigned long timeout;
	int *isFinished;
	unsigned long *startTime;

}typedef DriveToLine;

#endif /* INCLUDE_DRIVETOLINE_H_ */
