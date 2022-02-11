/*
 * VariableThrash.c
 *
 *  Created on: 14 Dec 2018
 *      Author: domen
 */

int MotorSpeed1Value = 0;
int MotorSpeed2Value = 0;

void MotorSpeed1Set(int MotorSpeed){
	MotorSpeed1Value = MotorSpeed;
}
int MotorSpeed1(){
	return MotorSpeed1Value;
}

void MotorSpeed2Set(int MotorSpeed){
	MotorSpeed2Value = MotorSpeed;
}

int MotorSpeed2(){
	return MotorSpeed2Value;
}
