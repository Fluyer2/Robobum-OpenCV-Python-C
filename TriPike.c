/*
 * TriPike.c
 *
 *  Created on: 18 Nov 2018
 *      Author: domen
 */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "VariableThrash.h"
#include <math.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define TIM2DIR ((TIM2->CR1 >> 4) & 1)  // 0 = Up counter  1 = Down counter
#define TIM4DIR ((TIM4->CR1 >> 4) & 1)

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

float deriv1;
int integ1;
int err1S;
int PWM1;

int P1 = 50;
int I1 = 10;
int D1 = 2;

float deriv2;
int integ2;
int err2;
int err2S;
int PWM2;

int P2 = 50;
int I2 = 10;
int D2 = 2;

int Start2, Start1, Start0 = 0;

int SpeedM1Debug;
int SpeedM2Debug;

int DistOne = 0;
int DistTwo = 0;

/*int Time(){
	int millis = HAL_GetTick();
	return millis;
}*/

/*int DeltaTime(int DeltaTime, int Place){   ///PAZI SAJ CE NI UPORABLJEN BO PRVA ZIHER ŠLA SKOZI SAJ BO TIMENEW ZELO VELIK PROTI TIME OLD////
	int TimeStartValue;

	TimeTest = TimeStart[Place];

	if(TimeStart[Place] == 0){
		TimeStart[Place] = Time();
	}


	TimeStartValue = Time() - TimeStart[Place];
	if(TimeStartValue < DeltaTime)
	{
		return 0;
	}
	else
	{
		TimeStart[Place] = 0;
		return 1;
	}

}*/

void FiveMill(){
	Start0 = 1;
	Start1 = 1;
	Start2 = 1;
}

/*void Motor1SpeedVal(){ ////////////KO TE BO ZAJEBAVALO SE SPOMNI NA OVERFLOW//////
	int MotorSpeed1;
	if(TIM2->CNT < 100){  ///Counter gre gor
		MotorSpeed1 = TIM2->CNT;
	}
	else if(TIM2->CNT > 64900){
		MotorSpeed1 = 65000 - TIM2->CNT;
	}
	else{
		MotorSpeed1 = 0;
	}

	TIM2->CNT = 0;

	MotorSpeed1Set(MotorSpeed1);
}*/

void Motor1SpeedVal(){
	int SpeedM1 = 0;
	if(TIM2DIR == 0){ //Counting up
		SpeedM1 = TIM2->CNT - 32500;
	}
	else if(TIM2DIR == 1){
		SpeedM1 = (TIM2->CNT - 32500)*(-1);
	}
	SpeedM1Debug = SpeedM1;
	TIM2->CNT = 32500;
	MotorSpeed1Set(SpeedM1);
}


void PID1(int WantedSpeed, int ActualSpeed){
	int err1;
	if(0 < WantedSpeed){
		if(Start0 == 1){
		err1 = WantedSpeed - ActualSpeed;
		integ1 = integ1 + (err1*1);
		deriv1 = (err1 - err1S)/1.0;
		PWM1 = P1*err1 + I1*integ1 + D1*deriv1;
		err1S = err1;
		Start0 = 0;
		}
	}

	if(WantedSpeed == 0){ ///Ko mu rečem naj se ustavi resetira vse skupaj
		PWM1 = 0;
		err1 = 0;
		err1S = 0;
		integ1 = 0;
	}

	//PWM1 = constrain(PWM1, 0, 3600);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,constrain(PWM1, 0, 3600));
}

int Motor1Init(int Speed){
//////////PREVERI V KATERO SMER SE NAJ VRTI KOLO////////////
	if(Speed < 0){
		HAL_GPIO_WritePin(M1INA_GPIO_Port, M1INA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1INB_GPIO_Port, M1INB_Pin, GPIO_PIN_RESET);
		Speed = Speed * (-1);
	}
	else if(Speed > 0){
		HAL_GPIO_WritePin(M1INA_GPIO_Port, M1INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1INB_GPIO_Port, M1INB_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(M1INA_GPIO_Port, M1INA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1INB_GPIO_Port, M1INB_Pin, GPIO_PIN_SET);
	}
	/*if(Speed == 0){
		HAL_GPIO_WritePin(M1INA_GPIO_Port, M1INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1INB_GPIO_Port, M1INB_Pin, GPIO_PIN_RESET);
	}*/
	return Speed;
///////////////////NASTAVI HITROST///////////////////////////

}

void Motor1(int Speed){ /// -40 do 40
	int WantedSpeed = Motor1Init(Speed); // V katero smer se naj vrti in vrne zeljeno pozitivno hitrost
	PID1(WantedSpeed, MotorSpeed1());
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
}

void Motor2SpeedVal(){
	int SpeedM2 = 0;
	if(TIM4DIR == 0){ //Counting up
		SpeedM2 = TIM4->CNT - 32500;
	}
	else if(TIM4DIR == 1){
		SpeedM2 = (TIM4->CNT - 32500)*(-1);
	}
	SpeedM2Debug = SpeedM2;
	TIM4->CNT = 32500;
	MotorSpeed2Set(SpeedM2);
}


void PID2(int WantedSpeed, int ActualSpeed){
	if(WantedSpeed > 0){
		if(Start1 == 1){
			err2 = WantedSpeed - ActualSpeed;
			integ2 = integ2 + (err2*1);
			deriv2 = (err2 - err2S)/1;
			PWM2 = P2*err2 + I2*integ2 + D2*deriv2;
			err2S = err2;
			Start1 = 0;
		}
	}

	if(WantedSpeed == 0){ ///CE MU RECEM NAj SE USTAVI SE BO USTAVIL POCASI
		PWM2 = 0;
		err2 = 0;
		err2S = 0;
		integ2 = 0;
	}
	PWM2 = constrain(PWM2, 0, 3600);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,PWM2);
}

int Motor2Init(int Speed){
//////////PREVERI V KATERO SMER SE NAJ VRTI KOLO////////////
	if(Speed < 0){
		HAL_GPIO_WritePin(M2INA_GPIO_Port, M2INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2INB_GPIO_Port, M2INB_Pin, GPIO_PIN_SET);
		Speed = Speed * (-1);
	}
	else if(Speed > 0){
		HAL_GPIO_WritePin(M2INA_GPIO_Port, M2INA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2INB_GPIO_Port, M2INB_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(M2INA_GPIO_Port, M2INA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2INB_GPIO_Port, M2INB_Pin, GPIO_PIN_SET);
	}
	return Speed;
///////////////////NASTAVI HITROST///////////////////////////
}

void Motor2(int Speed){ /// -40 do 40
	int WantedSpeed = Motor2Init(Speed); // V katero smer se naj vrti in vrne zeljeno pozitivno hitrost
	PID2(WantedSpeed, MotorSpeed2());
}


void MoveDist(int DIR, int DIST){ //DIST v milimetrih
	/*int32_t DistOne = 0;
	int32_t DistTwo = 0;*/
	int End1 = 0;
	int End2 = 0;
	while((End1 == 0) || (End2 == 0)){
		if(Start2 == 1){
			if(DistOne<(round(DIST*100000)/4638)){
				Motor1(10*DIR);
				DistOne = (DistOne + MotorSpeed1());
			}
			else{
				Motor1(0);
				End1 = 1;
				DistOne = 0;
			}


			if(DistTwo<(round(DIST*100000)/4638)){
				Motor2(10*DIR);
				DistTwo = (DistTwo + MotorSpeed2());
			}
			else{
				Motor2(0);
				End2 = 1;
				DistTwo = 0;
			}
			Start2 = 0;
		}
	}
}
