/*
 * RedEncoder.c
 *
 *  Created on: Feb 10, 2016
 *      Author: Erik
 */

#include "main.h"

RedEncoder initRedEncoder(Encoder encoder, long refreshTime)
{
	long currentTime = micros();
	int currentEncoder = encoderGet(encoder);
	double velocity = 0;
	RedEncoder newEncoder = {encoder, &currentTime, &currentEncoder, &velocity};
	return newEncoder;
}

int getRedEncoder(RedEncoder encoder)
{
	return encoderGet(encoder.encoder);
}

double getRedEncoderVelocity(RedEncoder encoder)
{
	if(micros() - (*encoder.lastReadTime) > 100000)
	{
		if(micros() - (*encoder.lastReadTime) < 1000000)
		{
			int currentEncoder = encoderGet(encoder.encoder);

			double velocity = (double) ((double) (currentEncoder - (*encoder.lastEncoder)) /
				(double) (micros() - (*encoder.lastReadTime)));

			*encoder.lastEncoder = encoderGet(encoder.encoder);

			*encoder.lastReadTime = micros();

			velocity *= 100000;

			encoder.velocity = &velocity;
			//lcdPrint(uart1, 1, "%d", encoderGet(encoder.encoder));
		}
		else
		{
			(*encoder.lastEncoder) = encoderGet(encoder.encoder);
			(*encoder.lastReadTime) = micros();
			(*encoder.velocity) = 0;
			//encoderReset((*encoder).encoder);
			//lcdSetText(uart1, 1, "Timeout");
		}

	}

	return (*encoder.velocity);
}
