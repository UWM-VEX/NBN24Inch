/*
 * Auto.h
 *
 *  Created on: Feb 25, 2015
 *      Author: Erik
 */

#ifndef AUTO_H_
#define AUTO_H_

/**
 * Reference type for information about the progress of autonomous mode.
 * the only instance of this type that should be used is autonomousInfo.
 */
struct AutonomousInfo{

	int step;
	int lastStep;
	int elapsedTime;
	int isFinished;

}typedef AutonomousInfo;

/**
 * Struct containing information about the autonomous mode.
 */
AutonomousInfo autonomousInfo;

int alliance;

#define RED 0
#define BLUE 1

#define DO_NOTHING 0
#define MODE_1 1
#define JUST_TURN 2
#define JUST_DRIVE 3
#define WORLDS_1 4
#define WORLDS_2 5
#define WORLDS_3 6
#define WORLDS_4 7
#define FIFTEENFEED1 8
#define FIFTEENFEED2 9

int autonomousSelection;

#endif /* AUTO_H_ */
