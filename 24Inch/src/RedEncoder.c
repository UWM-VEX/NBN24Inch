/*
 * RedEncoder.c
 *
 *  Created on: Feb 10, 2016
 *      Author: Erik
 */

#include "main.h"

RedEncoder initRedEncoder(Encoder encoder, long refreshTime)
{
	RedEncoder newEncoder = {encoder, micros(), encoderGet(encoder), 0};
	return newEncoder;
}

int getRedEncoder(RedEncoder *encoder)
{
	return encoderGet((*encoder).encoder);
}

double getRedEncoderVelocity(RedEncoder *encoder)
{
	if(micros() - (*encoder).lastReadTime > 100000)
	{
		if(micros() - (*encoder).lastReadTime < 1000000)
		{
			int currentEncoder = encoderGet((*encoder).encoder);

			double velocity = (double) ((double) (currentEncoder - (*encoder).lastEncoder) /
				(double) (micros() - (*encoder).lastReadTime));

			(*encoder).lastEncoder = encoderGet((*encoder).encoder);

			(*encoder).lastReadTime = micros();

			(*encoder).velocity = velocity * 100000;
			lcdPrint(uart1, 1, "%d", encoderGet((*encoder).encoder));
		}
		else
		{
			(*encoder).lastEncoder = encoderGet((*encoder).encoder);
			(*encoder).lastReadTime = micros();
			(*encoder).velocity = 0;
			//encoderReset((*encoder).encoder);
			lcdSetText(uart1, 1, "Timeout");
		}

	}

	return (*encoder).velocity;
}
