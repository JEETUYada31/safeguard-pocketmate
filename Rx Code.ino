/************************************************************************************************
~DEVICES~
1 x ARDUINO BOARD (UNO, NANO, OR PRO MINI)
1 x MPU6050 - ACCELEROMETER & GYROSCOPE
1 x AT09 - BLE MODULE
2 x BC547 - NPN TRANSISTOR
3 x TACTILE PUSH BUTTONS
1 x BUZZER

~CONNECTIONS~
ARDUINO A4  -  MPU6050 SDA
ARDUINO A5  -  MPU6050 SCL
ARDUINO D2  -  MPU6050 INT
ARDUINO D5  -  AT09 VCC (CONNECTED USING BC547)
ARDUINO D6  -  MPU6050 VCC (CONNECTED USING BC547)
ARDUINO D8  -  AT09 TXD
ARDUINO D9  -  AT09 RXD
ARDUINO D10 -  BUZZER
ARDUINO D11 -  5 FEET BUTTON
ARDUINO D12 -  10 FEET BUTTON
ARDUINO D13 -  15 FEET BUTTON

SLAVE ADDRESS:      0x04A316AA24CB

BC547 CONNECTIONS:  3.3V OR 5V FROM ARDUINO TO VCC OF COMPONENT
                    GROUND OF COMPONENT TO COLLECTOR OF BC547
                    DIGITAL PIN OF ARDUINO TO BASE OF BC547
                    EMITTER OF BC547 TO GROUND OF ARDUINO

PRE-REQUISITE:      AT+ROLE1             -  COMMAND SENT TO AT09  -  ENABLES MASTER MODE
                    AT+BAND04A316AA24CB  -  COMMAND SENT TO AT09  -  CONNECTS AT09 TO SLAVE AT09 
************************************************************************************************/

#include <LowPower.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AltSoftSerial.h>

Adafruit_MPU6050 mpu;
AltSoftSerial ble_device;

const int wakeUpPin = 2;
const int bleVCC = 5;
const int mpuVCC = 6;
const int buzzer = 10;
const int button1 = 11;
const int button2 = 12;
const int button3 = 13;

volatile bool b1State = LOW;
volatile bool b2State = LOW;
volatile bool b3State = LOW;

int desiredRange = 5;
    
void setup() {
  Serial.begin(9600);
  ble_device.begin(9600);

  pinMode(bleVCC, OUTPUT);
  pinMode(mpuVCC, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  pinMode(wakeUpPin, INPUT);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  PCICR |= 0b00000001;
  PCMSK0 |= 0b00111000;
  
  digitalWrite(bleVCC, LOW);
  digitalWrite(mpuVCC, HIGH);
  digitalWrite(buzzer, LOW);

  delay(100);

  mpu.begin();

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(3);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  delay(100);
}

void loop() {

  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW);
    
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    
  detachInterrupt(digitalPinToInterrupt(wakeUpPin));

  if (b1State) {
    desiredRange = 5;
  }

  else if (b2State) {
    desiredRange = 10;    
  }

  else if (b3State) {
    desiredRange = 15;    
  }

  Serial.println(desiredRange);

  if(mpu.getMotionInterruptStatus()) {

    unsigned long long timer = millis();
    
    bool connected = false;

    digitalWrite(mpuVCC, LOW);
    delay(500);
    digitalWrite(bleVCC, HIGH);

    while (millis() - timer <= 20000) {
      delay(100);     

      Serial.println(connected);

      char c;
      String temp;

      String cmd = "AT+RSSI?\r\n";

      for(int i = 0; i < cmd.length(); i++) {
        ble_device.write(cmd[i]);
      }
      delay(100);
  
      while (ble_device.available() > 0) {
        c = ble_device.read();
        Serial.write(c);

        temp += c;

        if (temp == "Connected\r\n" && !connected){
          connected = true;          
        }

        if (temp.substring(0, 8) == "OK+RSSI:" && temp.length() == 13) {
          int rssiVal = temp.substring(8, temp.length() - 2).toInt();

          //Distance = 10 ^ ((Measured Power – RSSI)/(10 * N))

          if (rssiVal != 127 && rssiVal != 0) {
            float distance = pow(10.0, ((float)(-69 - rssiVal)/(10 * 2.44)));
            distance = distance * 3.281;
            Serial.print("DISTANCE:\t");
            Serial.println(distance, 9);

            if (distance > desiredRange) {
              Serial.print("BEEP");        
              
              digitalWrite(buzzer, HIGH);
            }
          }
        }
        delay(10);
      }
    }

    digitalWrite(bleVCC, LOW);    

    if (!connected) {
      for(int i = 0; i < 20; i++){
        digitalWrite(buzzer, HIGH);
        delay(500);        
        digitalWrite(buzzer, LOW);
        delay(500);
      }    
    }

    digitalWrite(mpuVCC, HIGH);    

    delay(100);

    mpu.begin();

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(3);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
  }
}

void wakeUp() {

}

ISR(PCINT0_vect) {
  if (digitalRead(button1) == LOW) {
    b1State = HIGH;
    b2State = LOW;
    b3State = LOW;
  }
  if (digitalRead(button2) == LOW) {
    b1State = LOW;
    b2State = HIGH;
    b3State = LOW;    
  }
  if (digitalRead(button3) == LOW) {
    b1State = LOW;
    b2State = LOW;
    b3State = HIGH;  
  }
}