#include "TriPike.h"
#include "Rotation.h"
#include "main.h"

int Speed;
int Zelena = 0;
int Speed1 = 0;

void SerialSpeedCalc(char str[]){
	int Smer;

	Smer = (str[0] - 48);
	Speed = 10*(str[1]-48) + (str[2]-48);
	Speed1 = Speed;

	if(Zelena == 1){
		Motor1(-5);
		Motor2(5);
		if ((Speed < 15) && (Speed != 0)){
			Zelena = 0;
		}
	}
	else if(Zelena == 2){
		Motor1(5);
		Motor2(-5);
		if ((Speed < 15) && (Speed != 0)){
			Zelena = 0;
		}
	}
	else if(Smer == 1){ //ZAVIJA V LEVO
		Motor1((10 - Speed1) + Speed1);
		Motor2(10 - Speed1);
	}
	else if(Smer == 2){//ZAVIJA V DESNO
		Motor2((10 - Speed1) + Speed1);
		Motor1(10 - Speed1);
	}
	else if(Smer == 3){//pojdi naravnost
		Motor1(10);
		Motor2(10);
	}
	else if(Smer == 4){//Zelena na Desni
		MoveDist(1,50);
		RotateCW_CCW(-1, 90);
		MoveDist(1,25);
		Zelena = 2;
	}
	else if(Smer == 5){//Zelena na Levi
		MoveDist(1,50);
		RotateCW_CCW(1,90);
		MoveDist(1,25);
		Zelena = 1;
	}
	else if(Smer == 6){
		RotateCW_CCW(1,90);
	}
	else{
		Motor2(0);
		Motor1(0);
	}

}
