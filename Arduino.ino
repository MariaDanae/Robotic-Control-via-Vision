#include <Servo.h>
//#include <NewSoftSerial/NewSoftSerial.h>

//NewSoftSerial mySerial(0, 1);

//////////////////////////////////////////////////////////////////////////////////
Servo servo_right, servo_left, servo_up, servo_claw, servo_arm;
int servoPosition = 90;
char incomingByte;   // for incoming serial data

void wheel_attach();
void wheel_detach();
void arm_attach();
void arm_detach();
void claw_attach();
void claw_detach();


void setup()
{
	Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
	pinMode(13, OUTPUT);

	arm_attach(servo_arm);
	servo_arm.write(90);	//arm up
	delay(400);
	servo_arm.write(65);	//arm down
	delay(400);
	servo_arm.write(90);	//arm up
	delay(400);

	claw_attach(servo_claw);//claw open
	servo_claw.write(40);
	delay(400);
	servo_claw.write(100);	//claw close
	delay(400);
	servo_claw.write(40);	//claw open
	delay(400);
	claw_detach(servo_claw);
}

void loop()
{

	//96/96
	//Serial.print("1");

	if (Serial.available() > 0)
	{
		// read the incoming byte:
		// wait for a second
		incomingByte = Serial.read();
		Serial.println(incomingByte);

		switch (incomingByte)
		{
			//Serial.print(incom)




		case 'w':						//straight
			wheel_attach(servo_left, servo_right);

			servo_right.write(255);
			servo_left.write(0);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;

		case 'd':						//right turn
			wheel_attach(servo_left, servo_right);

			servo_right.write(0);
			servo_left.write(0);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;


		case 'a':						//left turn
			wheel_attach(servo_left, servo_right);

			servo_right.write(255);
			servo_left.write(255);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;

		case 's':						//reverse
			wheel_attach(servo_left, servo_right);

			servo_right.write(0);
			servo_left.write(250);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;

		case 't':						//straight fine tune
			wheel_attach(servo_left, servo_right);

			servo_right.write(100);
			servo_left.write(90);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;

		case 'h':						//right turn fine tune
			wheel_attach(servo_left, servo_right);

			servo_right.write(90);
			servo_left.write(90);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;


		case 'f':						//left turn fine tune
			wheel_attach(servo_left, servo_right);

			servo_right.write(100);
			servo_left.write(100);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;

		case 'y':						//reverse fine tune
			wheel_attach(servo_left, servo_right);

			servo_right.write(90);
			servo_left.write(100);
			delay(25);

			wheel_detach(servo_left, servo_right);
			break;

		case 'q':						//disconnect
			wheel_detach(servo_left, servo_right);
			arm_detach(servo_arm);
			claw_detach(servo_claw);

			break;

		case 'i':						//straight cont.
			wheel_attach(servo_left, servo_right);

			servo_right.write(255);
			servo_left.write(0);

			break;

		case 'j':						//right turn cont.
			wheel_attach(servo_left, servo_right);

			servo_right.write(0);
			servo_left.write(0);

			break;

		case 'l':						//left turn cont.
			wheel_attach(servo_left, servo_right);

			servo_right.write(255);
			servo_left.write(255);

			break;

		case 'k':						//reverse cont.
			wheel_attach(servo_left, servo_right);

			servo_right.write(0);
			servo_left.write(250);

			break;


		case 'g':						//grab
			claw_attach(servo_claw);
			servo_claw.write(40);	//claw open
			delay(400);
			//claw_detach(servo_claw);

			arm_attach(servo_arm);
			servo_arm.write(90);	//arm up
			delay(150);

			servo_arm.write(65);	//arm down
			delay(400);
			arm_detach(servo_arm);

			wheel_attach(servo_left, servo_right);//move towards block

			servo_right.write(255);
			servo_left.write(0);
			delay(300);
			wheel_detach(servo_left, servo_right);


			wheel_attach(servo_left, servo_right);//shimmy right

			servo_right.write(0);
			servo_left.write(0);
			delay(100);

			servo_right.write(255);				//shimmy left
			servo_left.write(255);
			delay(100);

			servo_right.write(255);				//move in
			servo_left.write(0);
			delay(400);
			wheel_detach(servo_left, servo_right);

			claw_attach(servo_claw);
			servo_claw.write(100);	//claw close         
			delay(400);
			//claw_detach(servo_claw);

			arm_attach(servo_arm);
			servo_arm.write(90);	//arm up
			delay(400);

			break;

		case 'r':						//release
			claw_attach(servo_claw);
			servo_claw.write(100);	//claw close
			delay(400);

			arm_attach(servo_arm);
			servo_arm.write(90);	//arm up
			delay(400);

			servo_arm.write(65);	//arm down
			delay(400);

			servo_claw.write(40);	//claw opem
			delay(800);
			//claw_detach(servo_claw);

			servo_arm.write(90);	//arm up
			delay(400);

			wheel_attach(servo_left, servo_right);//reverse away from blocks
			servo_right.write(0);
			servo_left.write(255);
			delay(800);
			wheel_detach(servo_left, servo_right);

			break;

		case 'v':					//arm up
			arm_attach(servo_arm);

			servo_arm.write(90);
			delay(150);

			break;

		case 'b':					//arm down
			arm_attach(servo_arm);

			servo_arm.write(65);
			delay(150);

			break;

		case 'n':					//claw open
			claw_attach(servo_claw);

			servo_claw.write(30);
			delay(400);

			//claw_detach(servo_claw);

			break;

		case 'm':					//claw close
			claw_attach(servo_claw);

			servo_claw.write(100);
			delay(400);

			//claw_detach(servo_claw);

			break;


		}
	}
}

void wheel_attach(Servo servo_left, Servo servo_right)
{
	servo_left.attach(11);
	servo_right.attach(10);
};
void wheel_detach(Servo servo_left, Servo servo_right)
{
	servo_left.detach();
	servo_right.detach();
};
void arm_attach(Servo servo_arm)
{
	servo_arm.attach(12);
};
void arm_detach(Servo servo_arm)
{
	servo_arm.detach();
};
void claw_attach(Servo servo_claw)
{
	servo_claw.attach(8);
};
void claw_detach(Servo servo_claw)
{
	servo_claw.detach();
};


