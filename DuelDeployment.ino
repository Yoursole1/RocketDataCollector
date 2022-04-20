#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <SD.h>

Servo zero;
Servo oneTwo;
Servo twoFour;

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1020)

const int MPU_ADDR = 0x68; 

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z; 
int16_t temperature; 

char tmp_str[7]; 

char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

Adafruit_BMP3XX bmp;

const int t = 10;
float accelsX[t];
float accelsY[t];
float accelsZ[t];
double alts[t];


File myFile;

void setup() {
  Serial.begin(9600);
  while(!Serial){true;}


  if (!bmp.begin_I2C()) {
    Serial.println("I2C FAILURE BMP");
    while (1);
  }
 

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  

//  pinMode(7, OUTPUT);
//  pinMode(8, OUTPUT);
//  pinMode(9, OUTPUT);
//  zero.attach(7);
//  oneTwo.attach(8);
//  twoFour.attach(9);
//
//  zeroServo(zero);
//  zeroServo(oneTwo);
//  zeroServo(twoFour);
//  delay(1000);
//  setServo(zero, 90);
//  setServo(oneTwo, 90);
//  setServo(twoFour, 90);
//  delay(1000);
//  zeroServo(zero);
//  zeroServo(oneTwo);
//  zeroServo(twoFour);



//------------------------------------------
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("This is a test file :)");
    myFile.println("testing 1, 2, 3.");
    for (int i = 0; i < 20; i++) {
      myFile.println(i);
    }
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
//------------------------------------------
}


int i = 0;
void loop() {
  if(i>t){
    printData();
    i = -100;
  }
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  



  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7*2, true);

  

  accelsX[i] = Wire.read()<<8 | Wire.read();
  accelsY[i] = Wire.read()<<8 | Wire.read();
  accelsZ[i] = Wire.read()<<8 | Wire.read(); 

//  gyro_x = Wire.read()<<8 | Wire.read();
//  gyro_y = Wire.read()<<8 | Wire.read();
//  gyro_z = Wire.read()<<8 | Wire.read();
  
  alts[i] = bmp.readAltitude(SEALEVELPRESSURE_HPA); //METERS


  
  delay(1000);
  i++;
}



void zeroServo(Servo servo){
  rotateServo(servo, -1000);
}

void setServo(Servo servo, int ang){
  int angle = servo.read();

  if(ang < 0){
    ang = 0;
  }else if(ang > 180){
    ang = 180;
  }

  rotateServo(servo, ang - angle);
}

void rotateServo(Servo servo, int deg){
  int sign = deg/abs(deg);//1 or -1
 
  int angle = servo.read();
 
  if(angle + deg > 180){
    deg = 180-angle;
  }else if(angle + deg < 0){
    deg = -1*angle;
  }

  int j = 0;
  for(int i = angle; j < abs(deg); i+=sign){
    servo.write(i);
    j++;
  }
}
//float accelsX[t];
//float accelsY[t];
//float accelsZ[t];
//double alts[t];
void printData(){
  Serial.begin(9600);
  while(!Serial){
   
  }
  
  Serial.println("accelX-----------------------");
  for(int i = 0; i < t; i++){
    Serial.println(accelsX[i]);
  }

  Serial.println("accelY-----------------------");
  for(int i = 0; i < t; i++){
    Serial.println(accelsY[i]);
  }

  Serial.println("accelZ-----------------------");
  for(int i = 0; i < t; i++){
    Serial.println(accelsZ[i]);
  }
  Serial.println("Height-----------------------");
  for(int i = 0; i < t; i++){
    Serial.println(alts[i]);
  }
}
