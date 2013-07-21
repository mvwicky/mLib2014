#define PI 3.1415926
#define dia 560

#define diff 100
#define mid 950

#define high mid + diff
#define low mid - diff

#define thresh 450 

class link {
	private:
		int	lineSensors [5]; 
		int thresh;
		struct driveMotors {
			Motor *motor;
			BackEMF *dEMF;
			int pos;
			float diameter;
			float radiusToMiddle;
		}left , right; 
		struct genMotors {
			Motor *gMotor;
			BackEMF *gEMF;
			int pos;
		}gMot[2];
		struct servos {
			Servo *servo; 
			int pos;
		}gServo[6];
		struct aSensor {
			Analog *aSens;
			int value;
		}ET , smallTopHat[5] , largeTopHat[3] , light , linearSlide , pot;
		struct dSensor {
			Digital *dSens;
			bool isOn;
		}lever[2] , slot[2] , smallTouch[4] , largeTouch[6];
		public:
			link(int , int);
			int buildLeftMotor(float , float);
			int buildRightMotor(float , float);
			int buildGenMotor(int);
			int buildServo(int);
			// Analog Sensor Methods			
			int newET(int); // makes a new ET sensor
			int newSmallTopHat(int , int);
			int newLargeTopHat(int); // makes a new large top hat sensor
			int newLightSensor(int , int); // makes a new light sensor		
			int newLinearSlide(int); // makes a new linear slide		
			int newPot(int); // makes a new potentiometer					
			int getET(); // gets value from ET sensor
			int getSmallTopHat(int); // gets the value from a small top hat sensor
			int getLargeTopHat(); // gets the values from the large top hat sensor
			int getLightSensor(int); // gets the value from a light sensor
			int getLinearSlide(); // gets the value from the linear slide
			int getPot(); // gets the values from the potentiometer
			// Digital Sensor Methods
			int newLever(int , int); // makes a new lever
			int newSlot(int , int); // makes a new slot
			int newSmallTouch(int , int); // makes a new small touch
			int newLargeTouch(int , int); // makes a new large touch
			bool getLever(int); // gets lever on/off
			bool getSlot(int); // gets slot on/off
			bool getSmallTouch(int); // gets small touch on/off
			bool getLargeTouch(int); // gets large touch on/off
			// Driving Methods
			int linkDriveDirect(int , int);  
			int driveFor(int , int);
			int driveDistance(int , float);
			// Sensor Use Methods
			int setLines(int , int , int , int , int); // this method sets the indicies for the line sensors (number them from left to right)
			bool smallTopHatOn(int);
			bool sensorsOn(bool conditions []);

};

link::link(int lport , int rport , int threshold) {
	left.motor = new Motor(lport);
	right.motor = new Motor(rport);
	left.dEMF = new BackEMF(lport);
	right.dEMF = new BackEMF(rport);
	thresh = threshold;
}

int link::buildLeftMotor(float diameter , float radiusToMiddle) {
	left.motor->clearPositionCounter();
	left.motor.pos = left.dEMF->value();
	left.diameter = diameter;
	left.radiusToMiddle = radiusToMiddle;
	return 0;
}

int link::buildRightMotor(float diameter , float radiusToMiddle) {
	right.motor->clearPositionCounter();
	right.motor.pos = right.dEMF->value();
	right.diameter = diameter;
	right.radiusToMiddle = radiusToMiddle;
	return 0;
}

int link::buildGenMotor(int port) {
	gMot[port].gMotor = new Motor(port);
	gMot[port].gEMF = new BackEMF(port);
	gMot[port].gMotor->clearMotorPositionCounter();
	gMot[port].pos = right.bMot->value();
	return 0;
}

int link::buildServo(int port) {
	gServo[port].servo = new Servo(port);
	gServo[port].servo->enable();
	gServo[port].pos = gServo[port].servo->position();
	return 0;
}

/* Analog Sensor Creation Methods */

int link::newET(int port) {
	ET.aSens = new Analog(port);
	ET.aSens->setPullup(0);
	ET.value = ET.aSens->value();
	return 0;
}

int link::newSmallTopHat(int index , int port) {
	smallTopHat[index].aSens = new Analog(port);
	smallTopHat[index].aSens->setPullup(1);
	smallTopHat[index].value = newSmallTouch[index].aSens->value();
	return 0;
}

int link::newLargeTopHat(int port) {
	largeTopHat.aSens = new Analog(port);
	largeTopHat.aSens->setPullup(1);
	largeTopHat.value = largeTopHat.aSens->value();
	return 0;
}

int link::newLightSensor(int index , int port) {
	light[index].aSens = new Analog(port);
	light[index].aSens->setPullup(1);
	light[index].value = light[index].aSens->value();
	return 0;
}

int link::newLinearSlide(int port) {
	linearSlide.aSens = new Analog(port);
	linearSlide.aSens->setPullup(1);
	linearSlide.value = linearSlide.aSens->value();
	return 0;
}

int link::newPot(int port) {
	pot.aSens = new Analog(port);
	pot.aSens->setPullup(1);
	pot.value = pot.aSens->value();
	return 0;
}

/*  Analog Sensor Getter Methods */

int link::getET() {
	ET.value = ET.aSens->value();
	return ET.value;
}

int link::getSmallTopHat(int index) {
	smallTopHat[index].value = smallTopHat[index].aSens->value();
	return smallTopHat[index].value;
}

int link::getLargeTopHat() {
	largeTopHat.value = largeTopHat.aSens->value();
	return largeTopHat.value;
}

int link::getLightSensor(int index) {
	light[index].value = light[index].aSens->value();
	return light[index].value;
}

int link::getLightSensor() {
	linearSlide.value = linearSlide.aSens->value();
	return linearSlide.value;
}

int link::getPot() {
	pot.value = pot.aSens->value();
	return pot.value;
}

/* Digital Sensor Creation Methods */

int link::newLever(int index , int port) {
	lever[index].dSens = new Digital(port);
	lever[index].isOn = lever[index].dSens->value();
	return 0;
}

int link::newSlot(int index , int port) {
	slot[index].dSens = new Digital(port);
	slot[index].isOn = slot[index].dSens->value();
	return 0;
}

int link::newSmallTouch(int index , int port) {
	smallTouch[index].dSens = new Digital(port);
	smallTouch[index].isOn = smallTouch[index].dSens->value();
	return 0;
}

int link::newLargeTouch(int index , int port) {
	largeTouch[index].dSens = new Digital(port);
	largeTouch[index].isOn = largeTouch[index].dSens->value();
	return 0;
}

/* Digital Sensor Getter Methods */

bool link::getLever(int index) {
	lever[index].isOn = lever[index].dSens->value();
	return lever[index].isOn;
}

bool link::getSlot(int index) {
	slot[index].isOn = slot[index].dSens->value();
	return slot[index].isOn;
}

bool link::getSmallTouch(int index) {
	smallTouch[index].isOn = smallTouch[index].dSens->value();
	return smallTouch[index].isOn;
}

int link::getLargeTouch(int index) {
	largeTouch[index].isOn = largeTouch[index].dSens->value();
	return largeTouch[index].isOn;
}

/* Driving Methods */	

int link::linkDriveDirect(int lspeed , int rspeed) {
	left.motor->moveAtVelocity(lspeed);
	right.motor->moveAtVelocity(rspeed);
	return 0;
}

int link::driveFor(int speed , int mtime) {
	left.motor->moveAtVelocity(speed);
	right.motor->moveAtVelocity(speed);
	msleep(mtime);
	return 0;
}

int link::driveDistance(int speed , float distance) {
	float ticks = (distance * 1000) / (left.diameter * PI);
	left.motor->moveRelativePosition(speed , (int)ticks);
	right.motor->moveRelativePosition(speed , (int)ticks);
	left.motor->blockMotorDone();
	right.motor->blockMotorDone();
	return 0;
}

/* Sensor Use Methods */

int link::setLines(int sens1 , int sens2 = 9 , int sens3 = 9 , int sens4 = 9 , int sens5 = 9) {
	lineSensors = [sens1 , sens2 , sens3 , sens4 , sens5 , sens6];
	return 0;
}

bool link::smallTopHatOn(int index) {
	smallTopHat[index].value = smallTopHat[index].aSens->value();
	if (smallTopHat[index].value <= thresh)
		return true;
	if (smallTopHat[index].value > thresh)
		return false;
}

bool link::sensorsOn(bool conditions []) {
	int conds = sizeof(conditions) - 1;
	
}