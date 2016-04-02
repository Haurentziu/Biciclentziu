#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

#include <IRremote.h>
#include <IRremoteInt.h>

#define irPin 11
//Stuff for the accelometer
MPU6050 mpu;
Quaternion q;
uint8_t intStatus;
int16_t ax, ay, az, gx, gy, gz;
uint16_t packetSize = 42;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float ypr[3]; //[yaw, pitch, roll]
VectorFloat g;
volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

//ir receiver
IRrecv irrecv(irPin);
decode_results result;
int irResult;
int lastResult;

uint64_t lastResultTime;

//PID Controller
float p, i, d;

int uaie;

void setup() {
  mpu.initialize();
  mpu.setXAccelOffset(-4767);
  mpu.setYAccelOffset(-2098);
  mpu.setZAccelOffset(1068);
  mpu.setXGyroOffset(58);
  mpu.setYGyroOffset(21);
  mpu.setZGyroOffset(44);
  if(mpu.dmpInitialize() == 0){
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  irrecv.enableIRIn();

  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  if(Serial.available() > 0){
    uaie = Serial.parseInt();
  }
  if (irrecv.decode(&result)) {
    irResult = result.value;
    
    if(irResult != 0xFFFFFFFF){
      lastResult = irResult;
    }
    lastResultTime = millis();
   Serial.println(lastResult, HEX);
 /*   Serial.println(lastResult, HEX);*/
    irrecv.resume();
  }      

  if(millis() - lastResultTime > 150){
    lastResult = 0;
  }

  
  
  if(lastResult == 0xFFFFA05F){
    digitalWrite(10, HIGH);
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(10, LOW);
    digitalWrite(13, LOW);
  }
  
//  getAngles();
  
/*  Serial.print("ypr\t");
  Serial.print(ypr[0]*180/3.1415);
  Serial.print("\t");
  Serial.print(ypr[1]*180/3.1415);
  Serial.print("\t");
  Serial.println(ypr[2]*180/3.1415);*/
}

void getAngles(){
   while (!mpuInterrupt && fifoCount < packetSize) {

  }

  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();

  //waits for the correct available data length
  while(fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();
  intStatus =  mpu.getIntStatus();
  if((intStatus & 0x10) || fifoCount == 1024) //checks for overflow
    mpu.resetFIFO();
  else if(intStatus & 0x02){
    //reads a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
  
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&g, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &g);
   }
}

void getIRData(){
  
}

float getDistance(){
 float d = 0;
 return d;
}

void setMotorSpeed(int pin, int speed){
}

void goForward(int speed){
}

void goBackward(int speed){
}

void turnLeft(int speed){
  
}

void turnRight(int speed){
}



