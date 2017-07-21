/*
  arduino_motor_controller

  This sketch turns on the motor pulsing through digital
  pin 3, reads the voltage on pin A0 to be configured as
  the photodiode voltage. Pins A1 and A2 are configured
  as the potentiometer reference and output pins to test
  how much resistance the signal has passed through 
  (a voltage divider). The photodiode voltage and ratio
  of pins A1 and A2 are passed through the serial
  interace to be handled in the Python code.
*/
int motorPin = 3;       // any digital pin should work
int potSupplyPin = 2;
int motorOnTime = 10;
int pdSensorPin = A0;   // any analog pins should work
int potRefPin = A1;
int potOutPin = A2;
int relayControlPin = 5;
int relayControlValue = 1;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);  // configure motor pin as an output
  pinMode(potSupplyPin, OUTPUT);
  pinMode(relayControlPin, OUTPUT);
  digitalWrite(potSupplyPin, HIGH);
  digitalWrite(relayControlPin, relayControlValue);
  while (! Serial);     // block until serial connection established
}

// the loop routine runs over and over again forever:
void loop() {
    // check if start command is received
    while (! Serial.available());
    String commandString = Serial.readString();
    if (commandString == "START") {
      // check is stop command is received,
      // if it is, exit the loop and wait for start
      while (Serial.readString() != "STOP") {
        int potRefValue = analogRead(potRefPin);
        float potRefVoltage = potRefValue * (5.0 / 1023); 
        int potOutValue = analogRead(potOutPin);
        float potOutVoltage = potOutValue * (5.0 / 1023);
        float potGain = potOutVoltage / potRefVoltage;  // get the gain ratio to convert back
                                                        // to resistance on the Python side
                                                        
         if ((potGain >= 0.99) || (potGain <= 0.01)) {
          relayControlValue = !relayControlValue;
          digitalWrite(motorPin, LOW);
          digitalWrite(relayControlPin, relayControlValue);
          digitalWrite(motorPin, HIGH);
          delay(500);
        }
        
        digitalWrite(motorPin, HIGH);
        delay(motorOnTime);
        digitalWrite(motorPin, LOW);
        Serial.println("MEASURE");
        Serial.flush();
        
        int pdSensorValue = analogRead(pdSensorPin);    // read the raw value 0 to 1023
        float pdVoltage = pdSensorValue * (5.0 / 1023.0); // convert to a voltage 0 to 5V
        Serial.println(pdVoltage, 3);    // print the voltage to the serial interface
        Serial.flush();

        potRefValue = analogRead(potRefPin);
        potRefVoltage = potRefValue * (5.0 / 1023); 
        potOutValue = analogRead(potOutPin);
        potOutVoltage = potOutValue * (5.0 / 1023);
        potGain = potOutVoltage / potRefVoltage;  // get the gain ratio to convert back
                                                        // to resistance on the Python side
        
        Serial.println(potGain, 3);
        Serial.flush();
        while (! Serial.available());
      }
    }
    else if (commandString == "SKIP") {
      while (! Serial.available());
      float targetGain = Serial.parseFloat();
      int potRefValue = analogRead(potRefPin);
      float potRefVoltage = potRefValue * (5.0 / 1023); 
      int potOutValue = analogRead(potOutPin);
      float potOutVoltage = potOutValue * (5.0 / 1023);
      float startingGain = potOutVoltage / potRefVoltage;
      int gainCounter = 0;
      if (abs(startingGain - targetGain) > 0.0005) {
        float currentGain = startingGain;
        digitalWrite(motorPin, HIGH);
        delay(2000);
        while (abs(currentGain - targetGain) > 0.0005) {
          potRefValue = analogRead(potRefPin);
          potRefVoltage = potRefValue * (5.0 / 1023); 
          potOutValue = analogRead(potOutPin);
          potOutVoltage = potOutValue * (5.0 / 1023);
          currentGain = potOutVoltage / potRefVoltage;
          if (abs(currentGain - targetGain) < 0.0005) {
            digitalWrite(motorPin, LOW);
            Serial.println(currentGain, 3);
            Serial.flush();
            break;
          }
          Serial.println(currentGain, 3);
          Serial.flush();
          if ((currentGain >= 0.99) || (currentGain <= 0.01)) {
            relayControlValue = !relayControlValue;
            digitalWrite(motorPin, LOW);
            digitalWrite(relayControlPin, relayControlValue);
            digitalWrite(motorPin, HIGH);
            delay(500);
          } 
          if (abs(currentGain - startingGain) < 0.01) {
            gainCounter = gainCounter + 1;
            if (gainCounter == 2) {
              digitalWrite(motorPin, LOW);
              Serial.println(777.777, 3);
              Serial.flush();
              break;
            }
            delay(2000);
          }
        }
        digitalWrite(motorPin, LOW);
      }
      else {
        Serial.println(startingGain, 3);
        Serial.flush();
      }
    }
    else if (commandString == "SWITCH") {
      relayControlValue = !relayControlValue;
      digitalWrite(relayControlPin, relayControlValue);
    }
    else if (commandString == "RES") {
      while(! Serial.available());
      motorOnTime = Serial.parseInt();
      Serial.println(motorOnTime);
      Serial.flush();
    }
    else if (commandString == "SETPD") {
      while(! Serial.available());
      float targetVoltage = Serial.parseFloat();
      float currentVoltage = analogRead(pdSensorPin) * (5.0 / 1023.0);
      int potRefValue = analogRead(potRefPin);
      float potRefVoltage = potRefValue * (5.0 / 1023); 
      int potOutValue = analogRead(potOutPin);
      float potOutVoltage = potOutValue * (5.0 / 1023);
      float startingPotGain = potOutVoltage / potRefVoltage;
      int gainCounter = 0;
      if (abs(targetVoltage - currentVoltage) > 0.1) {
	      digitalWrite(motorPin, HIGH);
        delay(2000);
	      while(abs(targetVoltage - currentVoltage) > 0.1) {
	        currentVoltage = analogRead(pdSensorPin) * (5.0 / 1023.0);
          if (abs(currentVoltage - targetVoltage) < 0.1) {
            digitalWrite(motorPin, LOW);
            Serial.println(currentVoltage, 3);
            Serial.flush();
            break;
          }
          potRefValue = analogRead(potRefPin);
          potRefVoltage = potRefValue * (5.0 / 1023); 
          potOutValue = analogRead(potOutPin);
          potOutVoltage = potOutValue * (5.0 / 1023);
          float currentPotGain = potOutVoltage / potRefVoltage;
          Serial.println(currentVoltage, 3);
          Serial.flush();
          if ((currentPotGain >= 0.99) || (currentPotGain <= 0.01)) {
            relayControlValue = !relayControlValue;
            digitalWrite(motorPin, LOW);
            digitalWrite(relayControlPin, relayControlValue);
            digitalWrite(motorPin, HIGH);
            delay(500);
          }
          if (abs(currentPotGain - startingPotGain) < 0.01) {
            gainCounter = gainCounter + 1;
            if (gainCounter == 2) {
              digitalWrite(motorPin, LOW);
              Serial.println(777.777, 3);
              Serial.flush();
              break;
            }
            delay(2000);
          }
        } 
   	    digitalWrite(motorPin, LOW);
      }
      else {
        Serial.println(currentVoltage, 3);
        Serial.flush();
      }
    }
    else if (commandString == "GETRES") {
      Serial.println(motorOnTime);
      Serial.flush();
    }
    else if (commandString == "GETDIR") {
      Serial.println(relayControlValue);
      Serial.flush();
    }
    else if (commandString == "GETPOS") {
      int potRefValue = analogRead(potRefPin);
      float potRefVoltage = potRefValue * (5.0 / 1023); 
      int potOutValue = analogRead(potOutPin);
      float potOutVoltage = potOutValue * (5.0 / 1023);
      float potGain = potOutVoltage / potRefVoltage;
      Serial.println(potGain, 3);
      Serial.flush();
    }
    else if (commandString == "GETVOLT") {
      float currentVoltage = analogRead(pdSensorPin) * (5.0 / 1023.0);
      Serial.println(currentVoltage, 3);
      Serial.flush();
    }
    else if (commandString == "TIMESWEEP") {
      while (Serial.readString() != "STOP") {
        float currentVoltage = analogRead(pdSensorPin) * (5.0 / 1023.0);
        Serial.println(currentVoltage, 3);
        Serial.flush();
      }
    }
}
