/************************************************************************************************
~DEVICES~
1 x ARDUINO BOARD (UNO, NANO, OR PRO MINI)
1 x MPU6050 - ACCELEROMETER & GYROSCOPE
1 x AT09 - BLE MODULE
2 x BC547 - NPN TRANSISTOR

~CONNECTIONS~
ARDUINO A4  -  MPU6050 SDA
ARDUINO A5  -  MPU6050 SCL
ARDUINO D2  -  MPU6050 INT
ARDUINO D3  -  AT09 STATE PIN
ARDUINO D5  -  AT09 VCC (CONNECTED USING BC547)
ARDUINO D6  -  MPU6050 VCC (CONNECTED USING BC547)
ARDUINO D8  -  AT09 TXD
ARDUINO D9  -  AT09 RXD

SLAVE ADDRESS:      0x04A316AA24CB

BC547 CONNECTIONS:  3.3V OR 5V FROM ARDUINO TO VCC OF COMPONENT
                    GROUND OF COMPONENT TO COLLECTOR OF BC547
                    DIGITAL PIN OF ARDUINO TO BASE OF BC547
                    EMITTER OF BC547 TO GROUND OF ARDUINO

PRE-REQUISITE:      AT+ROLE0             -  COMMAND SENT TO AT09  -  ENABLES SLAVE MODE
************************************************************************************************/

//#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AltSoftSerial.h>
#include <LowPower.h>

Adafruit_MPU6050 mpu;

const int wakeUpPin = 2;
const int bleSTATEPin = 3;
const int bleVCC = 5;
const int mpuVCC = 6;

//SoftwareSerial ble_device(8,9);
AltSoftSerial ble_device;

int nUnconn = 0;
int bleSTATE;

void setup() {  
  Serial.begin(9600);
  ble_device.begin(9600);

  pinMode(bleVCC, OUTPUT);
  pinMode(mpuVCC, OUTPUT);

  pinMode(wakeUpPin, INPUT);
  pinMode(bleSTATEPin, INPUT);
  
  digitalWrite(bleVCC, LOW);
  digitalWrite(bleSTATEPin, LOW);
  digitalWrite(mpuVCC, LOW);

  delay(100);

}

void loop() {
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); 

  digitalWrite(bleVCC, HIGH);
  delay(10);

  String cmd = "AT+SLEEP\r\n";

  for(int i = 0; i < cmd.length(); i++) {
        ble_device.write(cmd[i]);
      }

  delay(350);

  pinMode(bleSTATEPin, INPUT);
  bleSTATE = digitalRead(bleSTATEPin);

  if (bleSTATE == 0){
    nUnconn += 1;  
  }

  else if (bleSTATE == 1) {
    nUnconn = 0;

    delay(10000);
  }

  Serial.println(bleSTATE);
  Serial.println(nUnconn);


  delay(10);

  if (nUnconn >= 2880) {
    digitalWrite(mpuVCC, HIGH);
    digitalWrite(bleVCC, LOW);

    delay(100);

    mpu.begin();

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(3);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    delay(100);
    
    attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW);
    
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    
    detachInterrupt(digitalPinToInterrupt(wakeUpPin));
    delay(10);

    digitalWrite(mpuVCC, LOW); 
    nUnconn = 0;
  }

  digitalWrite(bleVCC, LOW);
}

void wakeUp() {

}
