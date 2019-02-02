
#include <Servo.h>
#include <Wire.h>

int percent = 0;
int PWMval = 0;

int speedByte = 0;
int motorByte = 0;

Servo myservo1;  //creating servo objects for each individual talon
Servo myservo2;  
Servo myservo3; 
Servo myservo4;
Servo myservo5;
Servo myservo6;


  void setup() 
    {
      Wire.begin(8);                  //begin i2c address 
      Wire.onReceive(receiveEvent);   //to be exectued each time i2c transaction occurs
      Serial.begin(9600);             //serial baud rate
      Serial.println("Starting");
      myservo1.attach(3);  // attaches the servo on pin 5 to the servo object
      myservo2.attach(5);
      myservo3.attach(6);
      myservo4.attach(9);
      myservo5.attach(10);
      myservo6.attach(11);
    }

  void loop() 
    {
      delay(100);
    }
  
  

  void receiveEvent(int howMany)
    {
 
     motorByte = Wire.read();
     speedByte = Wire.read();
          
       if (bitRead(motorByte,0) == 1){    //read bit 0 of motorbyte looking for motor 1 listening
          Serial.print("MotorByte: ");
          Serial.println(motorByte);      // print the integer
  
          Serial.print("SpeedByte: ");    
          Serial.println(speedByte);      // print the integer
     
          percent = ((speedByte-50)*2);   //duty cycle calculation
          Serial.print("Percent: ");
          Serial.println(percent);
          
          PWMval = map(percent, -100, 100, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180)
          myservo1.write(PWMval);                        // sets the servo position according to the scaled value
          Serial.print("PWMval: ");
          Serial.println(PWMval);

          Serial.println("Done with event");
          Serial.println("");
       }
       else if (bitRead(motorByte,1) == 1){
          Serial.print("MotorByte: ");
          Serial.println(motorByte);      // print the integer
  
          Serial.print("SpeedByte: ");    
          Serial.println(speedByte);      // print the integer
     
          percent = ((speedByte-50)*2);
          Serial.print("Percent: ");
          Serial.println(percent);
          
          PWMval = map(percent, -100, 100, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180)
          myservo2.write(PWMval);                        // sets the servo position according to the scaled value
          Serial.print("PWMval: ");
          Serial.println(PWMval);

          Serial.println("Done with event");
          Serial.println("");
       }
       else if (bitRead(motorByte,2) == 1){
          Serial.print("MotorByte: ");
          Serial.println(motorByte);      // print the integer
  
          Serial.print("SpeedByte: ");    
          Serial.println(speedByte);      // print the integer
     
          percent = ((speedByte-50)*2);
          Serial.print("Percent: ");
          Serial.println(percent);
          
          PWMval = map(percent, -100, 100, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180)
          myservo3.write(PWMval);                        // sets the servo position according to the scaled value
          Serial.print("PWMval: ");
          Serial.println(PWMval);

          Serial.println("Done with event");
          Serial.println("");
       }
       else if (bitRead(motorByte,3) == 1){
          Serial.print("MotorByte: ");
          Serial.println(motorByte);      // print the integer
  
          Serial.print("SpeedByte: ");    
          Serial.println(speedByte);      // print the integer
     
          percent = ((speedByte-50)*2);
          Serial.print("Percent: ");
          Serial.println(percent);
          
          PWMval = map(percent, -100, 100, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180)
          myservo4.write(PWMval);                        // sets the servo position according to the scaled value
          Serial.print("PWMval: ");
          Serial.println(PWMval);

          Serial.println("Done with event");
          Serial.println("");
       }
       else if (bitRead(motorByte,4) == 1) {
          Serial.print("MotorByte: ");
          Serial.println(motorByte);      // print the integer
  
          Serial.print("SpeedByte: ");    
          Serial.println(speedByte);      // print the integer
     
          percent = ((speedByte-50)*2);
          Serial.print("Percent: ");
          Serial.println(percent);
          
          PWMval = map(percent, -100, 100, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180)
          myservo5.write(PWMval);                        // sets the servo position according to the scaled value
          Serial.print("PWMval: ");
          Serial.println(PWMval);

          Serial.println("Done with event");
          Serial.println("");
       }
       else if (bitRead(motorByte,5) == 1){
          Serial.print("MotorByte: ");
          Serial.println(motorByte);      // print the integer
  
          Serial.print("SpeedByte: ");    
          Serial.println(speedByte);      // print the integer
     
          percent = ((speedByte-50)*2);
          Serial.print("Percent: ");
          Serial.println(percent);
          
          PWMval = map(percent, -100, 100, 1000, 2000);     // scale it to use it with the servo (value between 0 and 180)
          myservo6.write(PWMval);                        // sets the servo position according to the scaled value
          Serial.print("PWMval: ");
          Serial.println(PWMval);

          Serial.println("Done with event");
          Serial.println("");
       }
       else{
          Serial.print("Invalid motorByte entered.");
       }

    }
