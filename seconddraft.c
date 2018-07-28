#define WHEEL 18
#define AXLE 11.46875
#define MINDETECTIONS1 20
#define MINDETECTIONS2 22
#define TENNIS 1
#define BOTTLE 2
#define FAVG 13
#define TABLEY 75
#define TABLEX 180
#define WHEELTOFRONT 23
#define LENGTHROB 20 //IDEK4SURE
#define OUT 0
#define IN 1
#define MAXVAL 65536
#define SPEED 40

#define ALTS(A) ((A)==0 ? 3 : 0)

//TODO: code theta r polar -> cartesian

float GPSx = 0.0;
float GPSy = 0.0;

float theta = 0.0;

int initTime;
void updateGPS(float x,float y);
int detectionState();
void moveRob(float distance, int speed);
void rotateRob(float angle, int speed);
void printTime();
void align(int time);
void edgeRob(float distance, int sensor, int speed,float coeff, bool bottle);
void lineFollow(int speed);
void checkRob ();
void pickBalls ();
void dumpBalls ();
void inputValue();
void pivotRob(float angle, int speed, int disabledMotor);
bool onTable(int sensor);
int rOL = 1;



task main()
{
	edgeRob(100/*change later*/,S1,40,0.75,true);
	inputValue();
	initTime = nSysTime;
	/*
	edgealign
	move forward (edgefollow)
	make a U-turn
	edgefollow
	snake around again
	when it sees bottle, will turn towards right in first run, left in other 2
	continue to the next table when it reaches end using color sensor (we know how long the table is)
	go forward to next table
	turn 90 and edgealign then turn 90, line follow
	turn 90, edgefollow to fence
	figure out how to drop the balls
	ending we will have to figure out
	*/
	//checkRob();
	//pickBalls();
	rotateRob(180,SPEED);
	//align(3000);
	rotateRob(-90,SPEED);
	edgeRob(100/*change later*/,S1,40,0.75,false);
	moveRob(-20,SPEED);
	pivotRob(-90,30,motorD);
	moveRob(20,40);
	rotateRob(70,30);
	//moveRob(2,50);
	align(10000);
	rotateRob(-185,30);
	edgeRob (99999999,S4, SPEED, 0.0, false);
	align(9000);
	rotateRob(90,SPEED);
	edgeRob(150,S4,50,0,false);
	rotateRob(90,40);
	edgeRob(100,S4,40,0.75,true);
	moveRob(LENGTHROB+10,100);
	sleep(2000);
	rotateRob(rOL*-90,SPEED);
	edgeRob(MAXVAL,(rOL == 1) ? S4 : S1,SPEED,0.50,false);
	rotateRob(rOL*90,50);
	edgeRob(TABLEY,(rOL == 1) ? S4 : S1,SPEED,0.55,false);
	rotateRob(rOL*90,SPEED);
	edgeRob(23,S1,SPEED,0.3,false);
	rotateRob(-180,30);
	edgeRob(245678910,S1,SPEED,0,false);
	align(5000);
	//edgeRob(MAXVAL,S4,SPEED,0,false);
	edgeRob(-36774213,S1,-30,0.0,false);
	/*
	dumpBalls();
	sleep (2000);
	//edgeRob (MAXVAL,S1,50,false);//
	//align (3000);//
	//rotateRob(90,50);//
	//edgeRob (TABLEY,S1,50,false);//getting back to the beginning
	//rotateRob(90,50);//
	//edgeRob (MAXVAL,S1,50,false);//
	//rotateRob(-90,50);//
	//moveRob(TABLEX,50);//
	//align(3000);//


	/*
	rotateRob(10,-90);
	moveRob(10,50); //UNIT TEST GOOD
	displayCenteredBigTextLine(0,"%f",GPSx);
	displayCenteredBigTextLine(2,"%f",GPSy);
	displayCenteredBigTextLine(4,"%f",theta);
	*/

}


void updateGPS(float x,float y)
{
	GPSx = x;
	GPSy = y;
}

int detectionState()//TODO: replace with get bottle fxn that's an int
{
	if(getUSDistance(S1)<MINDETECTIONS1)
	{
		if(getUSDistance(S2)<MINDETECTIONS2)
		{
			return BOTTLE;
		}else
		{
			return TENNIS;
		}
	}else
	{
		return 0;
	}
}

void moveRob(float distance, int speed)
{
	speed*=-1;
	if(speed < 0)
	{
		speed *= -1;
		distance *= -1;
	}
	GPSy += (float)distance * sgn(speed) * sinDegrees(theta);
	GPSx += (float)distance * sgn(speed) * cosDegrees(theta);
	int encoder[2];
	encoder[0] = getMotorEncoder(motorA);
	encoder[1] = getMotorEncoder(motorD);
	setMotorSpeed(motorA,speed*sgn(distance));
	setMotorSpeed(motorD,speed*sgn(distance));
	distance = distance * (360 / WHEEL);
	encoder[0] += distance;
	encoder[1] += distance;
	int flag = sgn(encoder[0]-getMotorEncoder(motorA));
	while(flag+sgn(encoder[0]-getMotorEncoder(motorA)) && flag+sgn(encoder[1]-getMotorEncoder(motorD))){}
	setMotorSpeed(motorA,0);
	setMotorSpeed(motorD,0);
}

void rotateRob(float angle, int speed)
{
	if(speed<0)
	{
		speed*=-1;
		angle*=-1;
	}
	int encoder[2];
	encoder[0] = getMotorEncoder(motorA);
	encoder[1] = getMotorEncoder(motorD);
	setMotorSpeed(motorA,speed*sgn(angle));
	setMotorSpeed(motorD,speed*-sgn(angle));
	float distance = angle * (AXLE * PI * (360.0 / WHEEL)) / 360.0;
	encoder[0] += distance;
	encoder[1] -= distance;
	int flag[2];
	flag[0] = sgn(encoder[0]-getMotorEncoder(motorA));
	flag[1] = sgn(encoder[1]-getMotorEncoder(motorD));
	while(flag[0]+sgn(encoder[0]-getMotorEncoder(motorA)) && flag[1]+sgn(encoder[1]-getMotorEncoder(motorD))){}
	setMotorSpeed(motorA,0);
	setMotorSpeed(motorD,0);
	theta+=angle;
}



void pivotRob(float angle, int speed, int disabledMotor)
{
	angle*=2;
	if(speed<0)
	{
		speed*=-1;
		angle*=-1;
	}
	int encoder[2];
	encoder[0] = getMotorEncoder(motorA);
	encoder[1] = getMotorEncoder(motorD);
	setMotorSpeed(motorA,speed*sgn(angle));
	setMotorSpeed(motorD,speed*-sgn(angle));
	setMotorSpeed(disabledMotor,0);
	float distance = angle * (AXLE * PI * (360.0 / WHEEL)) / 360.0;
	encoder[0] += distance;
	encoder[1] -= distance;
	int flag[2];
	flag[0] = sgn(encoder[0]-getMotorEncoder(motorA));
	flag[1] = sgn(encoder[1]-getMotorEncoder(motorD));
	while(flag[0]+sgn(encoder[0]-getMotorEncoder(motorA)) && flag[1]+sgn(encoder[1]-getMotorEncoder(motorD))){}
	setMotorSpeed(motorA,0);
	setMotorSpeed(motorD,0);
	theta+=angle;
}


void printTime()
{
	int milliTime = nSysTime - initTime;
	int milliseconds = milliTime % 1000;
	int seconds = (milliTime / 1000) % 60 ;
	int minutes = ((milliTime / (1000*60)) % 60);
	displayCenteredBigTextLine(0,"%02d:%02d:%02d", minutes, seconds, milliseconds);
}

void align(int time)
{
	int i = 0;
	while(1){
		setMotorSpeed(motorA,(FAVG-SensorValue[S1])/2);
		setMotorSpeed(motorD,(FAVG-SensorValue[S4]));
		if(++i>=time){
			break;
		}
	}
}

bool onTable(int sensor)
{
	if(SensorValue[sensor]<FAVG)
	{
		return (bool)OUT;
	}else
	{
		return (bool)IN;
	}
}
void pivot(float angle, int motorX, int speed)
{
	if(speed<0)
	{
		speed*=-1;
		angle*=-1;
	}
	int encoder[2];
	encoder[0] = getMotorEncoder(motorA);
	encoder[1] = getMotorEncoder(motorD);
	setMotorSpeed(motorA,speed*sgn(angle));
	setMotorSpeed(motorD,speed*-sgn(angle));
	setMotorSpeed (motorX,0);
	float distance = angle * (AXLE * PI * (360.0 / WHEEL)) / 360.0;
	distance *= 2;
	encoder[0] += distance;
	encoder[1] -= distance;
	int flag[2];
	flag[0] = sgn(encoder[0]-getMotorEncoder(motorA));
	flag[1] = sgn(encoder[1]-getMotorEncoder(motorD));
	while(flag[0]+sgn(encoder[0]-getMotorEncoder(motorA)) || flag[1]+sgn(encoder[1]-getMotorEncoder(motorD))){}
	setMotorSpeed(motorA,0);
	setMotorSpeed(motorD,0);
	theta+=angle;
}

bool bottleDetect(int sensor)
{
	//	if (getUSDistance(sensor) < 20)
	{
		//	return true; //bottle is present
	}
	return false;  //bottle is not
}
void checkRob ()
{
	getMotorEncoder(motorA);
	getMotorEncoder(motorD);
	//getMotorEncoder(motorB);
	//getMotorEncoder(motorC);
	getColorReflected(S1);
	getColorReflected(S4);
	getUSDistance(S3);
	playSound(soundBeepBeep);
}
void pickBalls ()
{
	while(1);
	setMotorSpeed(motorC, 50);
}
void dumpBalls ()
{
	setMotorSpeed(motorB,50);
}
void inputValue(){
	int i;
	sleep(600);
	for(i = 1; i < 6; i++){
		if(getButtonPress(i) == 1){
			break;
		}
		if (i == 5)
			i = 0;
	}
	if(i == buttonRight)
	{
		rOL = -1;
	}
	displayCenteredBigTextLine(2, " %d", i);
}

void edgeRob(float distance, int sensor, int speed, float coeff, bool bottle)
{
	distance -= 3;
	distance = distance * (360.0 / (float)WHEEL);
	int initMotorEncoder = abs(getMotorEncoder(motorA));
	int deltaEncoder = 0;
	while(onTable(ALTS(sensor))){
		while(onTable(ALTS(sensor))){
			deltaEncoder = abs(getMotorEncoder(motorA))-initMotorEncoder;
			deltaEncoder = abs(deltaEncoder);
			if(deltaEncoder>= (distance-360))
			{
				coeff= .3;
			}
			if(deltaEncoder >= distance)
			{
				coeff = .06;
			}
			if(bottle)
			{
				if((getTouchValue(S2)||getTouchValue(S3))/*add stuff now*/)
				{
					pivotRob(90,50,ALTS(sensor));
					pivotRob(90,-50,ALTS(sensor));//WEIGHT OF BOTTLE MIGHT ADD TRACTION
				}
			}
			if(sensor){
				setMotorSpeed(motorA,-1*(speed-coeff*(FAVG-SensorValue(sensor))));
				setMotorSpeed(motorD,-1*(speed+coeff*(FAVG-SensorValue(sensor))));
				}else{
				setMotorSpeed(motorA,-1*(speed+coeff*(FAVG-SensorValue(sensor))));
				setMotorSpeed(motorD,-1*(speed-coeff*(FAVG-SensorValue(sensor))));
			}
			//sleep(30);
		}
		//sleep(30);
	}
	setMotorSpeed(motorA,0);
	setMotorSpeed(motorD,0);
}
