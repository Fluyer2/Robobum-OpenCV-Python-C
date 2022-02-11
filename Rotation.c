/*
 * Rotation.c
 *
 *  Created on: 30 Nov 2018
 *      Author: domen
 */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "TriPike.h"
#include <string.h>
#include "VariableThrash.h"
#include <math.h>


#define PI  3.14159265359
#define WHEEL_CIRCUM  PI*70.0
#define ROTATE_DIST_BOTH  PI*170.0
#define ROTATE_DIST_SINGLE  PI*170.0*2.0
#define FULL_ENCODER_VAL  4741.44
#define ENCODER_CONSTANT_BOTH  (ROTATE_DIST_BOTH*FULL_ENCODER_VAL)/(WHEEL_CIRCUM*360.0)
#define ENCODER_CONSTANT_SINGLE (ROTATE_DIST_SINGLE*FULL_ENCODER_VAL)/(WHEEL_CIRCUM*360.0)

//int PotOne = 0;
int32_t PotOne;
int32_t PotTwo;
void MoveMotorOne(float MovedDist, float WantedDistance, float DIR){
	float Speed;
	float SpeedIncrement;

	SpeedIncrement = (10.0f/(WantedDistance/2.0f));//(MAXSPEED/ ZELJENA POT POLOVIC)

	if(MovedDist < (round(WantedDistance/2.0f))){   //CE SE SE NI PREMAKNIL ZA POL ZELJENE VREDNOSTI
		Speed = SpeedIncrement * MovedDist;
		Motor1(DIR*Speed + DIR*5);
	}
	else if(MovedDist > (round(WantedDistance/2.0f))){	//CE SE JE ZE PREMAKNIL ZA POL
		Speed = SpeedIncrement * (WantedDistance - MovedDist);
		Motor1(DIR*Speed + DIR*5);
	}

}

void DeltaDistOneEVa(){
	PotOne =  PotOne + MotorSpeed1();

	if(PotOne > 4250000000){
		PotOne = 0;
	}
}

void DeltaDistTwoEVa(){
	PotTwo =  PotTwo + MotorSpeed2();

	if(PotTwo > 4250000000){
		PotTwo = 0;
	}
}

void MoveMotorTwo(int MovedDist, int WantedDistance, int DIR){
	int Speed;
	float SpeedIncrement;

	SpeedIncrement = (10.0f/(WantedDistance/2.0f));

	if(MovedDist < (round(WantedDistance/2.0f))){   //CE SE SE NI PREMAKNIL ZA POL ZELJENE VREDNOSTI
		Speed = SpeedIncrement * MovedDist;
		Motor2(DIR*Speed + DIR*5);
	}
	else if(MovedDist > (round(WantedDistance/2.0f))){	//CE SE JE ZE PREMAKNIL ZA POL
		Speed = SpeedIncrement * (WantedDistance - MovedDist);
		Motor2(DIR*Speed + DIR*5);
	}

}

void RotateCW_CCW(int Dir, int WantedAngle){ /// CW = -1 or CCW = 1
	float RotateValBoth;
	float RotateValSingle;
	RotateValBoth = WantedAngle * ENCODER_CONSTANT_BOTH;
	RotateValSingle = WantedAngle * ENCODER_CONSTANT_SINGLE;
	PotOne = 0;
	PotTwo = 0;
	if(Dir == 1){
		Motor1(0);
		Motor2(0);
		while(((PotOne + PotTwo)/2) < RotateValBoth){
			MoveMotorOne(PotOne, RotateValBoth, 1);
			MoveMotorTwo(PotTwo, RotateValBoth, -1);
		}
		Motor1(0);
		Motor2(0);
	}

	if(Dir == -1){
		Motor1(0);
		Motor2(0);
		while(((PotOne + PotTwo)/2) < RotateValBoth){
			MoveMotorOne(PotOne, RotateValBoth, -1);
			MoveMotorTwo(PotTwo, RotateValBoth, 1);
		}
		Motor1(0);
		Motor2(0);
	}


	//OKOLI DESNEGA KOLESA
	if(Dir == 2){
		Motor1(0);
		Motor2(0);
		while((PotOne) < (RotateValSingle)){
			MoveMotorOne(PotOne, RotateValSingle, 1);
		}
		Motor1(0);
	}

	if(Dir == -2){
		Motor1(0);
		Motor2(0);
		while((PotOne) < (RotateValSingle)){
			MoveMotorOne(PotOne, RotateValSingle, -1);
		}
		Motor1(0);
	}
	//OKOLI LEVEGA KOLESA
	if(Dir == 3){
			Motor1(0);
			Motor2(0);
			while((PotTwo) < (RotateValSingle)){
				MoveMotorTwo(PotTwo, RotateValSingle, 1);
			}
			Motor2(0);
		}
	if(Dir == -3){
				Motor1(0);
				Motor2(0);
				while((PotTwo) < (RotateValSingle)){
					MoveMotorTwo(PotTwo, RotateValSingle, -1);
				}
				Motor2(0);
			}
}


