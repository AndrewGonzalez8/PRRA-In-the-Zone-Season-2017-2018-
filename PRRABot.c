#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, in2,    reversepot,     sensorPotentiometer)
#pragma config(Sensor, in3,    fourbarpot,     sensorPotentiometer)
#pragma config(Sensor, in8,    potmob,         sensorPotentiometer)
#pragma config(Sensor, dgtl1,  Encoder_R,      sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  Encoder_L,      sensorQuadEncoder)
#pragma config(Motor,  port1,           baseleftFront, tmotorVex393TurboSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           fourbarleft,   tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           armleft,       tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           baseleftBack,  tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           mobile,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           roller,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           baserightBack, tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           armright,      tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           fourbarright,  tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          baserightFront, tmotorVex393TurboSpeed_HBridge, openLoop, reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

//PID constants

float Kp=0.5;//0,5
float Ki=0;
float Kd=0.5;//0.5
float Gp=0.6;
float Gi=0;
float Gd=0.4;
float Ap=0.5;
float Ai=0;
float Ad=0.2;
float Mp=0.5;
float Mi=0;
float Md=0;
float Cp=0.5;
float Ci=0;
float Cd=0.3;

static float MaximumorMinimum;
static float MaximumorMinimumArm;
static float MotorMaximum = 127;
static float MotorMinimum = -127;

void Base_Forward(float Distance_I, float Seconds)
{
	Distance_I *= 24.66; //360/(4.0*3.14)
	Distance_I = round(Distance_I);
  Seconds*= (1000/15);
  float Power, Error, Integral = 0, Derivative, Prev_Error = 0, Error_E;
	SensorValue[Encoder_R]=0;
	SensorValue[Encoder_L]=0;
	//while(-1*(SensorValue[Encoder_R]) < Distance_I)
	for(int counter =0; counter<Seconds;	counter++) {
		Error = (Distance_I - -1*(SensorValue[Encoder_R])); //Goal distance - current distance

		//Calculate error to adjust

		Integral = Integral + Error;	//Calculate integral part of correction

		if(Error == 0){ //passed goal distance
			Integral=0;
		}

		Derivative = Error - Prev_Error; //Calculate derivative part of correction
		Prev_Error = Error;

		Power = (Error*Kp) + (Integral*Ki) + (Derivative*Kd);	//Set power correction for motors

		if(Power>127){
			Power=127;
		}

		if(Power < -127){
			Power=-127;
		}

		if(-1*(SensorValue[Encoder_R]) > -1*(SensorValue[Encoder_L]))//Adjust motor power for encoder difference / Right > Left
		{
			Error_E=(-1*(SensorValue[Encoder_R])- (-1*(SensorValue[Encoder_L])));
			motor[baseleftFront] = Power;
			motor[baseleftBack] = Power;
			motor[baserightFront] = Power-Error_E;
			motor[baserightBack] = Power-Error_E;
		}

		else if(-1*(SensorValue[Encoder_R]) < -1*(SensorValue[Encoder_L]))//Adjust motor power for encoder difference / Left > Right
		{
			Error_E= (-1*(SensorValue[Encoder_L]) - (-1*(SensorValue[Encoder_R])));
			motor[baseleftFront] = Power-Error_E;
			motor[baseleftBack] = Power-Error_E;
			motor[baserightFront] = Power;
			motor[baserightBack] = Power;
		}

		else
		{
			motor[baseleftFront] = Power;
			motor[baseleftBack] = Power;
			motor[baserightFront] = Power;
			motor[baserightBack] = Power;
		}

		wait1Msec(25);
	  datalogAddValue(1,Error);
	}
	motor[baseleftFront] = 0;
	motor[baseleftBack] = 0;
	motor[baserightFront] = 0;
	motor[baserightBack] = 0;
}

void Base_Backward(float Distance_I, float Seconds)
{
	Distance_I *= 24.66; //360/(4.0*3.14)
	Distance_I = round(Distance_I);
  Seconds*= (1000/15);
  float Power, Error, Integral = 0, Derivative, Prev_Error = 0, Error_E;
	SensorValue[Encoder_R]=0;
	SensorValue[Encoder_L]=0;
	//while(-1*(SensorValue[Encoder_R]) < Distance_I)
	for(int counter =0; counter<Seconds;	counter++) {
		Error = Distance_I - SensorValue[Encoder_R]; //Goal distance - current distance

		//Calculate error to adjust

		Integral = Integral + Error;	//Calculate integral part of correction

		if(Error == 0){ //passed goal distance
			Integral=0;
		}

		Derivative = Error - Prev_Error; //Calculate derivative part of correction
		Prev_Error = Error;

		Power = (Error*Kp) + (Integral*Ki) + (Derivative*Kd);	//Set power correction for motors

		if(Power>127){
			Power=127;
		}

		if(Power < -127){
			Power=-127;
		}

		if(SensorValue[Encoder_R] > SensorValue[Encoder_L])//Adjust motor power for encoder difference / Right > Left
		{
			Error_E=SensorValue[Encoder_R]-SensorValue[Encoder_L];
			motor[baseleftFront] = -Power;
			motor[baseleftBack] = -Power;
			motor[baserightFront] = -Power+Error_E;
			motor[baserightBack] = -Power+Error_E;
		}

		else if(SensorValue[Encoder_R] < SensorValue[Encoder_L])//Adjust motor power for encoder difference / Left > Right
		{
			Error_E= SensorValue[Encoder_L] - SensorValue[Encoder_R];
			motor[baseleftFront] = -Power+Error_E;
			motor[baseleftBack] = -Power+Error_E;
			motor[baserightFront] = -Power;
			motor[baserightBack] = -Power;
		}

		else
		{
			motor[baseleftFront] = -Power;
			motor[baseleftBack] = -Power;
			motor[baserightFront] = -Power;
			motor[baserightBack] = -Power;
		}

		wait1Msec(25);
	  datalogAddValue(1,Error);
	}
	motor[baseleftFront] = 0;
	motor[baseleftBack] = 0;
	motor[baserightFront] = 0;
	motor[baserightBack] = 0;
}


void Turn_Left(float Degree, float Seconds)//**********************************************************************************************************************
{
 datalogClear();
 SensorValue[in1]=0;
 Degree= round(Degree*(10));
 float Power, Integral = 0, Derivative, Prev_Error = 0;
 float Error;
  Seconds*= (1000/15);


 for(int counter =0; counter<Seconds;	counter++)															 //Turn Right
	{
		Error=(Degree - SensorValue[in1]);						           //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction

			if(Error==0)																						 //Adjust integral error
			{Integral=0;}

		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=(Error*Gp)+Integral*Gi+Derivative*Gd;									 //Set power correction for motors

			if(Power>127)
			{
				Power=127;
				}

			motor[baseleftFront] = -Power;
			motor[baseleftBack] = -Power;
			motor[baserightFront] = Power;
			motor[baserightBack] = Power;

			wait1Msec(15);
		}
			motor[baseleftFront] = 0;
			motor[baseleftBack] = 0;
			motor[baserightFront] = 0;
			motor[baserightBack] = 0;
}

void Turn_Right(float Degree, float Seconds)//**********************************************************************************************************************
{
 SensorValue[in1]=0;
 Degree= round(Degree*(-10));
 float Power, Integral, Derivative, Prev_Error;
 float Error;
  Seconds*= (1000/15);


 for(int counter =0; counter<Seconds;	counter++)															 //Turn Right
	{
		Error=(Degree - SensorValue[in1]);						           //Calculate error to adjust
		Integral=Integral+Error;	//Calculate integral part of correction
		datalogClear();


			if(Error==0)																						 //Adjust integral error
			{Integral=0;}

		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=-1*(Error*Gp)+Integral*Gi+-1*(Derivative*Gd);									 //Set power correction for motors

			if(Power>100)
			{
				Power=100;
				}

			motor[baseleftFront] = Power;
			motor[baseleftBack] = Power;
			motor[baserightFront] = -Power;
			motor[baserightBack] = -Power;

			wait1Msec(15);
		}
	motor[baseleftFront] = 0;
			motor[baseleftBack] = 0;
			motor[baserightFront] = 0;
			motor[baserightBack] = 0;
}

void Correct(float Degree, float Seconds)//**********************************************************************************************************************
{
 Degree= round(Degree*(-10));
 float Power, Integral, Derivative, Prev_Error;
 float Error;
  Seconds*= (1000/15);


 for(int counter =0; counter<Seconds;	counter++)															 //Turn Right
	{
		Error=(Degree - SensorValue[in1]);						           //Calculate error to adjust
		Integral=Integral+Error;	//Calculate integral part of correction
		datalogClear();


			if(Error==0)																						 //Adjust integral error
			{Integral=0;}

		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=-1*(Error*Gp)+Integral*Gi+-1*(Derivative*Gd);									 //Set power correction for motors

			if(Power>100)
			{
				Power=100;
				}

			motor[baseleftFront] = Power;
			motor[baseleftBack] = Power;
			motor[baserightFront] = -Power;
			motor[baserightBack] = -Power;

			wait1Msec(15);
		}
	motor[baseleftFront] = 0;
			motor[baseleftBack] = 0;
			motor[baserightFront] = 0;
			motor[baserightBack] = 0;
}


task Arm_Position
	{
		int Power, Error, Integral=0, Derivative, Prev_Error=0;
	Error=MaximumorMinimumArm - SensorValue[reversepot];          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Ap+Integral*Ai+Derivative*Ad;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{Power=MotorMinimum;
				}

		motor[armleft]=Power;
		motor[armright]=Power;

		wait1Msec(15);

}

void Arm_Up(int Height, float Seconds)//*******************************************************************************************************************
{
	Seconds*=(1000/15);
	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=Height - SensorValue[reversepot];          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Ap+Integral*Ai+Derivative*Ad;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
			}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
			}



		motor[armleft]=Power;
		motor[armright]=Power;

		wait1Msec(25);
	}
		motor[armleft]=0;
		motor[armright]=0;
}

void Mobile_Down(int Height, float Seconds)//*******************************************************************************************************************
{
	Seconds*=(1000/15);
	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=SensorValue[potmob] - Height;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Mp+Integral*Mi+Derivative*Md;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
				}

		motor[mobile]=-Power;


		wait1Msec(15);
	}
	motor[mobile]=0;
}

void Chain_Down(int Height, float Seconds)//*******************************************************************************************************************
{
	Seconds*=(1000/15);
	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=SensorValue[fourbarpot] - Height;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Cp+Integral*Ci+Derivative*Cd;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
				}

		motor[fourbarleft]=Power;
    motor[fourbarright]=-Power;

		wait1Msec(15);
	}
motor[fourbarleft]=0;
    motor[fourbarright]=0;

}


void Sevenconesintothe20pointzone()
{
	Arm_Up(800,1);
Mobile_Down(3920,1);//Stretch mogo intake
Base_Forward(52,2);//Forward until intaking the mogo
Correct(0,0.5);//Robot allignment
Mobile_Down(870,1);//Intaking the mogo into the base
Arm_Up(500,0.5);//Makes the arm down to score the preload
wait1Msec(300);
motor[roller] = -127;//1st Cone outtake
wait1Msec(200);
motor[roller] = 0;
motor[roller] = 75;//Turns the roller to intake the next cone
wait1Msec(100);
Base_Forward(10,0.3);//Runs until reaching the next cone
Arm_Up(800,0.5);//Arm up in order to intake
Chain_Down(1400,0.5);//Makes the chain go down to intake the cone
wait1Msec(300);
Arm_Up(360,0.3);//Arm goes down
wait1Msec(300);
Arm_Up(700,0.5);//Arm goes up
Chain_Down(2760,0.5);//Chain goes up and aligns the cone with the mogo
wait1Msec(200);
motor[roller] = 0;
motor[roller] = -127;//The roller outtakes the 2nd cone
wait1Msec(200);
motor[roller] = 0;
Arm_Up(1300,0.5);//The arm goes up in order to not knock down the cone that is placed on the driver loads plate
Base_Backward(32,1.5);//Runs backward until it reaches the driver loads plate
Turn_Right(90,1);//Turns 90 degrees facing the driver loads plate
Base_Backward(3,0.5);//Goes a little bit backward to intake the cone
Arm_Up(900,0.8);//Arm goes up
Chain_Down(1400,0.5);//Chain goes down
motor[roller] = 100;//The roller starts
Arm_Up(650,0.5);//Arm goes down
wait1Msec(200);
Arm_Up(1100,0.5);//After intaking the cone the arm goes up
Chain_Down(2760,0.5);//The chain goes up
Arm_Up(650,0.5);//The arm goes down to align the cone with the mogo
motor[roller] = 0;
wait1Msec(150);
motor[roller] = -100;//Outtakes the 3rd cone
wait1Msec(300);
motor[roller] = 0;
Arm_Up(1200,0.5);//Arm goes up
Chain_Down(1400,0.5);//Chain goes down
motor[roller] = 100;//Roller starts intaking
Arm_Up(650,0.5);//The arm goes down
wait1Msec(200);
Arm_Up(1200,0.5);//Arm goes up
Chain_Down(2760,0.5);//Chain goes up
Arm_Up(650,0.5);//The arm goes down to align the cone with the mogo
motor[roller] = 0;
wait1Msec(150);
motor[roller] = -100;//Roller outtakes 4th cone
wait1Msec(300);
motor[roller] = 0;
Arm_Up(1300,0.5);//Arm goes up
Chain_Down(1400,0.5);//chain goes down
motor[roller] = 100;//Roller starts intaking
Arm_Up(650,0.5);//Arm goes down
wait1Msec(200);
Arm_Up(1300,0.5);//Arm goes up
Chain_Down(2760,0.5);//Chain goes up
Arm_Up(650,0.5);//The arm goes down to align the cone with the mogo
motor[roller] = 0;
wait1Msec(150);
motor[roller] = -100;//Outtakes the 5th cone
wait1Msec(300);
motor[roller] = 0;
Arm_Up(1500,0.5);//Arm goes up
Chain_Down(1400,0.5);//Chain goes down
motor[roller] = 100;//Roller starts intaking
Arm_Up(650,0.5);//Arm goes down
wait1Msec(200);
Arm_Up(1500,0.5);//Arm goes up
Chain_Down(2760,0.5);//Chain goes up
Arm_Up(650,0.5);//The arm goes down to align the cone with the mogo
motor[roller] = 0;
wait1Msec(150);
motor[roller] = -100;//Outtakes the 6th cone
wait1Msec(300);
motor[roller] = 0;
Arm_Up(1700,0.5);//Arm goes up
Chain_Down(1400,0.5);//Chain goes down
motor[roller] = 100;//Roller starts intaking
Arm_Up(650,0.5);//Arm goes down
wait1Msec(200);
Arm_Up(1700,1.2);//Arm goes up
Chain_Down(2760,0.5);//Chain goes up
Arm_Up(650,0.5);//The arm goes down to align the cone with the mogo
motor[roller] = 0;
wait1Msec(150);
motor[roller] = -100;//Outtakes the 7th and final cone
wait1Msec(300);
motor[roller] = 0;//Stops the roller
Turn_Right(90,2);//Turns 90 degrees right
Base_Forward(40,1);//Runs until the line
Turn_Right(30,0.5);//Turns 90 degrees
}





void pre_auton()
{
	//Clear gyro and scale it in order for it to work properly. Don't move the robot during these three seconds.
 SensorType[in1] = sensorNone;
 wait1Msec(2000);
 SensorType[in1] = sensorGyro;
 wait1Msec(1000);
 SensorValue[gyro] = 0;
 SensorScale[gyro] = 130;
}



task autonomous()
{
}



task usercontrol()
{
  while (true)
  {
      motor[baseleftFront] = vexRT[Ch2] + vexRT[Ch1];
      motor[baserightFront] = vexRT[Ch2] - vexRT[Ch1];
      motor[baseleftBack] = vexRT[Ch2] + vexRT[Ch1];
      motor[baserightBack] = vexRT[Ch2] - vexRT[Ch1];
      motor[armleft]= vexRT[Ch3];
      motor[armright]= vexRT[Ch3];

		if(vexRT[Btn7U]==1)
	{
		motor[mobile] = 127;
	}


	else if(vexRT[Btn7D]==1)
	{
		motor[mobile] = -127;
	}



	else
	{
		motor[mobile] = 0;
	}





	if(vexRT[Btn6D]==1)
	{
		motor[roller] = 127;
	}


	else if(vexRT[Btn6U]==1)
	{
		motor[roller] = -127;
	}


	else
	{
		motor[roller] = 0;
	}


	if(vexRT[Btn5U]==1)
	{
		motor[fourbarleft] = -127;
		motor[fourbarright] = 127;
	}


	else if(vexRT[Btn5D]==1)
	{
		motor[fourbarleft] = 127;
		motor[fourbarright] = -127;
	}


	else
	{
		motor[fourbarleft] = 0;
		motor[fourbarright] = 0;
	}



  }
}
