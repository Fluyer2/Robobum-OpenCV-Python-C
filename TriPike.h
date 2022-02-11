/*
 * TriPike.h
 *
 *  Created on: 18 Nov 2018
 *      Author: domen
 */

#ifndef TRIPIKE_H_
#define TRIPIKE_H_

int Motor1Init();
int Motor2Init();
void Motor1(int Speed);
void Motor2(int Speed);
void FiveMill();
void MoveDist(int DIR, int DIST);
int PID1(int WantedSpeed, int ActualSpeed);
int PID2(int WantedSpeed, int ActualSpeed);
int Motor1SpeedVal();
int Motor2SpeedVal();

#endif /* TRIPIKE_H_ */
