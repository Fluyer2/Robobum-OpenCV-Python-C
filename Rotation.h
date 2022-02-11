/*
 * Rotation.h
 *
 *  Created on: 30 Nov 2018
 *      Author: domen
 */

#ifndef ROTATION_H_
#define ROTATION_H_

void DeltaDistOneEVa();
float DeltaDistOneMill();
void RotateCW_CCW(int Dir, int WantedAngle);
void MoveMotorOne(int MovedDist, int WantedDistance, int DIR);
void DeltaDistTwoEVa();
void MoveMotorTwo(int MovedDist, int WantedDistance, int DIR);

#endif /* ROTATION_H_ */
