// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

int dirByte = 0;
int speedByte = 0;

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
}

void loop()
{
  Serial.println("1 = Forward | 2 = Reverse | 3 = Turn Left | 4 = Turn Right | 0 = Stop");
  Serial.print("Which Direction would you like?: ");

  while(Serial.available() == 0)
    {
      
    }
  
  if(Serial.available() > 0)
    {
      dirByte = Serial.parseInt();
      Serial.print(dirByte, DEC);
    }
    
  Serial.println("");
  Serial.print("What percent speed would you like? (0 to 100): ");

  while(Serial.available() == 0)
    {
      
    }

  if(Serial.available() > 0)
    {
      speedByte = Serial.parseInt();
      Serial.print(speedByte);
    }

  
  Wire.beginTransmission(8); // transmit to device #4
  Wire.write(dirByte);        // sends five bytes
  Wire.write(speedByte);      // sends one byte
  Wire.endTransmission();    // stop transmitting

  Serial.println("");
  Serial.print("dirByte: ");
  Serial.println(dirByte);

  Serial.print("speedByte: ");
  Serial.println(speedByte);
  Serial.println("------------------------");

  delay(500);
}
