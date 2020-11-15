#include <MovingAverageFilter.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();

AccelStepper stepper1(1, 29, 31); //belt drive
AccelStepper stepper2(1, 30,28); //scissor lift
AccelStepper stepper3(1, 27,25); //rotate brush
AccelStepper stepper4(1, 19, 17); //brush  400 steps
AccelStepper stepper5(1, 23,18);//1,23,21 //cups  ,1100 steps between cups

#define EN1  9
#define EN2  6
#define EN3  7
#define EN4  32
#define EN5  37

#define MOTOR1_MS1 14
#define MOTOR1_MS2 10
#define MOTOR2_MS1 12
#define MOTOR2_MS2 8
#define MOTOR3_MS1 11
#define MOTOR3_MS2 4
#define MOTOR4_MS1 36
#define MOTOR4_MS2 35
#define MOTOR5_MS1 33
#define MOTOR5_MS2 34

const long MAX_STEPS_X = 8000;
const long MAX_STEPS_Y = 30000;
const long MAX_STEPS_BRUSH = 1500;

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(


// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

// Calibration
float calibrationDistances[4] = {0};

typedef struct Position {
    long x;
    long y;
} Position;

Position calibrationPositions[4] = {0};

float length1, length2;
float x,y;
float xPos,yPos;
float angle = 0.0;



float angle_stepsize = 0.01;
float newAngle_stepsize = .5;
float angleTarget1;
float angleTarget2;
int brushLength = 1500;
int circleDirection;
int cup = 0;
int cup1 = 500;
int cup2 = 1000;
int cup3 = 1500;
int cup4 = 2000;
int counter1 = 0;


void setup() {
    Serial.begin(115200);

    // Configure each stepper
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(EN3, OUTPUT);
    pinMode(EN4, OUTPUT);
    pinMode(EN5, OUTPUT);

    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, HIGH);
    digitalWrite(EN3, HIGH);
    digitalWrite(EN4, HIGH);
    digitalWrite(EN5, HIGH);

    pinMode(MOTOR1_MS1, OUTPUT); digitalWrite(MOTOR1_MS1, HIGH);
    pinMode(MOTOR1_MS2, OUTPUT); digitalWrite(MOTOR1_MS2, HIGH);
    pinMode(MOTOR2_MS1, OUTPUT); digitalWrite(MOTOR2_MS1, HIGH);//LOW original
    pinMode(MOTOR2_MS2, OUTPUT); digitalWrite(MOTOR2_MS2, LOW);//HIGH original
    pinMode(MOTOR3_MS1, OUTPUT); digitalWrite(MOTOR3_MS1, HIGH);
    pinMode(MOTOR3_MS2, OUTPUT); digitalWrite(MOTOR3_MS2, HIGH);
    pinMode(MOTOR4_MS1, OUTPUT); digitalWrite(MOTOR4_MS1, LOW);//LOW
    pinMode(MOTOR4_MS2, OUTPUT); digitalWrite(MOTOR4_MS2, LOW);//LOW
    pinMode(MOTOR5_MS1, OUTPUT); digitalWrite(MOTOR5_MS1, LOW);//HIGH
    pinMode(MOTOR5_MS2, OUTPUT); digitalWrite(MOTOR5_MS2, HIGH);//LOW

    //digitalWrite(EN1, LOW); //Pull enable pin low to allow motor control
    // Change these to suit your stepper if you want
    stepper1.setMaxSpeed(2000);
    stepper2.setMaxSpeed(1500);//2000
    stepper3.setMaxSpeed(1000);
    stepper4.setMaxSpeed(1000);
    stepper5.setMaxSpeed(500);

    stepper1.setAcceleration(1000);
    stepper2.setAcceleration(1500);//2000
    stepper3.setAcceleration(1000);
    stepper4.setAcceleration(1000);
    stepper5.setAcceleration(500);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2);
    steppers.addStepper(stepper3);
    steppers.addStepper(stepper4);
    steppers.addStepper(stepper5);

    delay(2000);

    vl.begin();

    calibrateCanvas();
}

void enableMotion(bool enable) {
    digitalWrite(EN1, enable ? LOW : HIGH);
    digitalWrite(EN2, enable ? LOW : HIGH);
    digitalWrite(EN3, enable ? LOW : HIGH);
    digitalWrite(EN4, enable ? LOW : HIGH);
    digitalWrite(EN5, enable ? LOW : HIGH);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setMotionTarget(long x, long y, long brushLength, long cup) {
    // FIXME: move this offset calculation to the calibration phase
    int minDist = 1000;
    for (int i = 0; i < 4; i++) {
        if (calibrationDistances[i] < minDist) {
            minDist = calibrationDistances[i];
        }
    }

    // TODO: adjust brush depth based on calibration information
    int brushLengthOffset = 0;

    long positions[5];

    positions[0] = x;
    positions[1] = y;
    positions[2] = 0;
    positions[3] = brushLength + brushLengthOffset;
    positions[4] = cup;

    steppers.moveTo(positions);
}

float getDistance() {
    const int sampleCount = 64;
    int samplesObtained = 0;
    float distance = 0;

    for (int i = 0; i < sampleCount; i++) {
        int range = vl.readRange();
        int status = vl.readRangeStatus();

        if (status == VL6180X_ERROR_NONE) {
            distance += range;
            samplesObtained += 1;
        }

        delay(1);
    }

    return distance / samplesObtained;
}

void calibrateCanvas() {
    enableMotion(true);

    int distanceAtBottom = getDistance();

    long canvasBottomY = 0;
    /*for (; canvasBottomY < MAX_STEPS_Y; canvasBottomY += 100) {
        setMotionTarget(0, canvasBottomY, 0, 0);
        steppers.runSpeedToPosition();

        long distance = getDistance();

        if (distance < 8 * distanceAtBottom / 10) {
            break;
        }
    }*/

    canvasBottomY += 5000;

    setMotionTarget(0, canvasBottomY, 0, 0);
    steppers.runSpeedToPosition();

    int ex = 7000;
    int ey = 2 * MAX_STEPS_Y / 3;

    int distance;

    Serial.println("=========================");
    setMotionTarget(ex, canvasBottomY, 0, 0);
    steppers.runSpeedToPosition();
    delay(100);
    distance = getDistance();
    calibrationDistances[0] = distance;
    calibrationPositions[0].x = ex;
    calibrationPositions[0].y = canvasBottomY;
    Serial.println(distance);
    delay(1000);

    setMotionTarget(ex, canvasBottomY + ey, 0, 0);
    steppers.runSpeedToPosition();
    delay(100);
    distance = getDistance();
    calibrationDistances[1] = distance;
    calibrationPositions[1].x = ex;
    calibrationPositions[1].y = canvasBottomY + ey;
    Serial.println(distance);
    delay(1000);

    setMotionTarget(0, canvasBottomY + ey, 0, 0);
    steppers.runSpeedToPosition();
    delay(100);
    distance = getDistance();
    calibrationDistances[2] = distance;
    calibrationPositions[2].x = 0;
    calibrationPositions[2].y = canvasBottomY + ey;
    Serial.println(distance);
    delay(1000);

    setMotionTarget(0, canvasBottomY, 0, 0);
    steppers.runSpeedToPosition();
    delay(100);
    distance = getDistance();
    calibrationDistances[3] = distance;
    calibrationPositions[3].x = 0;
    calibrationPositions[3].y = canvasBottomY;
    Serial.println(distance);
    delay(1000);

    for (int i = 0; i < 4; i++) {
        Serial.println("Starting eval...");

        setMotionTarget(ex, canvasBottomY, 0, 0);
        steppers.runSpeedToPosition();
        delay(1000);

        setMotionTarget(ex, canvasBottomY + ey, 0, 0);
        steppers.runSpeedToPosition();
        delay(1000);

        setMotionTarget(0, canvasBottomY + ey, 0, 0);
        steppers.runSpeedToPosition();
        delay(1000);

        setMotionTarget(0, canvasBottomY, 0, 0);
        steppers.runSpeedToPosition();
        delay(1000);
    }

    // Go home and stop there
    setMotionTarget(0, 0, 0, 0);
    steppers.runSpeedToPosition();
    while (true);
/*
    while (true) {
        int range = vl.readRange();
        int status = vl.readRangeStatus();

        if (status == VL6180X_ERROR_NONE) {
            int output = rangeFilter.process(range);
            Serial.println(output);
        }

        delay(10);
    }
    */
}

void loop() {
    long positions[5]; // Array of desired stepper positions
    delay(2000);


    int b = 1;

    digitalWrite(EN1,LOW);
    digitalWrite(EN2,LOW);
    digitalWrite(EN3,LOW);
    digitalWrite(EN4,LOW);
    digitalWrite(EN5,LOW);
    circleDirection = random(1,3);
    dip();
    for( int i = 0; i < 3; i++){///for loop #1,   3 time loop start
        int times = random(2,8);
        Serial.print("times  =  ");Serial.println(times);
        for(int i= 0; i < times; i++){//  for loop #2, point to point start
            x = random(0,10000);
            y = random(5000,25000);
            if(counter1 == 0){
                brushLength = 0;
                steppers.moveTo(positions);
                steppers.runSpeedToPosition(); }
            else{brushLength = 1500;}
            brushLength = 1500;
            positions[0] = x;
            positions[1] = y;//scizzor
            positions[2] = 0;
            positions[3] = brushLength;
            positions[4] = cup;
            steppers.moveTo(positions);
            steppers.runSpeedToPosition();


            counter1++;

        }/////////////////////////////// for loop #2, point to point end
        counter1 = 0;
        int circleTimes = random(2,5);
        for( int j = 0; j < circleTimes; j++){/// for loop #3,  start circles

            circleDirection = random(1,3);
            xPos = random(1000,7000);
            yPos = random(5000,20000);
            length1 = random(1000,2000);
            //length2 = length1*5;
            //float modulator = random(200,500)*.01;
            float modulator = random(100,300)*.01;

            length2 = length1*modulator;
            //Serial.print("modulator  =  ");Serial.println(modulator);
            // Serial.print("length1  =  ");Serial.println(length1);
            //  Serial.print("length2  =  ");Serial.println(length2);

            angle = random(0,628)*.01;
            angleTarget1 = angle + random(314,628)*.01; //  angle is less than angleTarget1, angleTarget1 is more than angle
            angleTarget2 = angle - random(314,628)*.01; // angle is more than angleTarget2, angleTarget2 is less than angle

            ///////////////////////////////////go to first circle
            if(x >= 0 && y >= 0){
                x =  xPos + length1 *cos(angle);
                y = yPos + length2 * sin(angle);
                positions[0] = x;
                positions[1] =  y;
                positions[2] = 0;
                positions[3] = 0;//brushLength;
                positions[4] = cup;
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();

                x =  xPos + length1 *cos(angle);
                y = yPos + length2 * sin(angle);
                positions[0] = x;
                positions[1] =  y;
                positions[2] = 0;
                positions[3] = brushLength;
                positions[4] = cup;
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();
                //digitalWrite(EN4,HIGH);
            }
            delay(500);
            //digitalWrite(EN4,HIGH);
            while(angle < angleTarget1 && circleDirection == 1 && x > 0 && y > 0){/////////////////////circle
                x =  xPos + length1 *cos(angle);
                y = yPos + length2 * sin(angle);
                positions[0] = x;
                positions[1] =  y;
                positions[2] = 0;
                positions[3] = brushLength;
                positions[4] = cup;
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();

                angle += angle_stepsize;
                /*
                if(brushLength <= 500 && angle < 4.71){
                  brushLength +=1; digitalWrite(EN4,LOW);}
                  else{ digitalWrite(EN4,HIGH);}
                  if(angle > 4.71 && brushLength > 300){
                    brushLength -=1; digitalWrite(EN4,LOW);}
                    else{ digitalWrite(EN4,HIGH);}
                     //Serial.println(angle);

                     */
            }//////////////////////////////////////end clockwise circle

            while(angle > angleTarget2 && circleDirection == 2 && x > 0 && y > 0){///////////////////// start counter clockwise circle
                x =  xPos + length1 *cos(angle);
                y = yPos + length2 * sin(angle);
                positions[0] = x;
                positions[1] =  y;
                positions[2] = 0;
                positions[3] = brushLength;
                positions[4] = cup;
                steppers.moveTo(positions);
                steppers.runSpeedToPosition();

                angle -= angle_stepsize;
                /*
                if(brushLength <= 500 && angle < 4.71){
                  brushLength +=1; digitalWrite(EN4,LOW);}
                  else{ digitalWrite(EN4,HIGH);}
                  if(angle > 4.71 && brushLength > 300){
                    brushLength -=1; digitalWrite(EN4,LOW);}
                    else{ digitalWrite(EN4,HIGH);}
                     //Serial.println(angle);

                     */
            }//////////////////////////////////////end  counter clockwise circle


            angle = 0;
            //digitalWrite(EN4,LOW);///////////retract brush
            positions[0] = x;
            positions[1] =  y;
            positions[2] = 0;
            positions[3] = 0;
            positions[4] = cup;
            steppers.moveTo(positions);
            steppers.runSpeedToPosition();
            //digitalWrite(EN4,HIGH);


        }// for loop #3

        dip();
    }//for loop #1 end
    /////////////////////////////back to home
    positions[0] = 0;
    positions[1] =  0;
    positions[2] = 0;
    positions[3] = 0;
    positions[4] = cup;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();


}

void dip(){
    //digitalWrite(EN4,LOW);
    cup = random(1,5);
    if(cup == 1){cup = cup1;}
    if(cup == 2){cup = cup2;}
    if(cup == 3){cup = cup3;}
    if(cup == 4){cup = cup4;}
    Serial.print("cup =  "); Serial.println(cup);

    long positions[5];
    positions[0] = 0;//x
    positions[1] = 4000;// y  scizzor up
    positions[2] = 4200;//rotate
    positions[3] = 0;//brush
    positions[4] = cup;//cups
    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); // Blocks until all are in position
    delay(500);//1
    positions[0] = 0;//x
    positions[1] = 4000;// y  scizzor hold
    positions[2] = 4200;//rotate
    positions[3] = 0;//brush
    positions[4] = cup;//550;//cups
    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); // Blocks until all are in position
    delay(500);//1


    positions[0] = 0;//x
    positions[1] = 800;// y  scizzor dip
    positions[2] = 4200;//rotate
    positions[3] = 0;//brush
    positions[4] = cup;//550;//cups
    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); // Blocks until all are in position
    delay(500);//2

    positions[0] = 0; //x
    positions[1] = 4000; // y scizzor up
    positions[2] = 4200; //rotate
    positions[3] = 0;//brush
    positions[4] = cup;//cups
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    positions[0] = 0;//x
    positions[1] = 4000;// y  scizzor up
    positions[2] = 0;//rotate
    positions[3] = 0;//brush
    positions[4] = cup;//cups
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    // digitalWrite(EN4,HIGH);
    /*
      positions[0] = 3000;
    positions[1] = 10000;//scizzor up
    positions[2] = 0;
    positions[3] = 0;
    positions[4] = 0;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    */
}
