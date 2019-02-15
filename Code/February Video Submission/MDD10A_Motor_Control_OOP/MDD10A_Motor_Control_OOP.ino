class littleMotors
{
  int pwmpin;
  int dirpin;
  int speedByte;
  int x;

  public:

    void attachPWM(int pwmpinset)
      {
        pwmpin = pwmpinset;
      }

    void attachDirPin(int diopin)
      {
        dirpin = diopin;
      }
  
    void setSpeedByte(int speedpwm)
      {
        x = map(speedpwm,0, 100, 0 ,255);
        analogWrite(pwmpin,x);
      }

};

littleMotors motor1a;
littleMotors motor1b;
littleMotors motor2a;
littleMotors motor2b;
int motorSelected = 0;
int speedSelected = 0;
int directionSelected = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() 
{
// put your main code here, to run repeatedly:

     Serial.println("What motor would you like to move ?( 1 | 2 | 3 | 4)");
     while (Serial.available() == 0)
       {
    
       }  
     motorSelected = Serial.parseInt();
     //serialFlush();
   

    Serial.println("How fast would you like to that motor to go? 0 to 100");
    while (Serial.available() == 0)
      {

      }
    speedSelected = Serial.parseInt();
    //serialFlush();
   
    Serial.println("What direction would you like that motor to go? (1 = Forward | 2 = Backward);");
       while (Serial.available() == 0)
       {

       }
       directionSelected = Serial.parseInt();
       //serialFlush();

      switch (motorSelected)
      {
        case 1:
          Serial.print("Turning motor 1: ");
          if (directionSelected == 1)
          {
            Serial.print("Forward");
          }
          else if (directionSelected == 2)
          {
            Serial.print("Backward");
          }
          else
          {
            Serial.print("NO DIRECTION");
          }
          Serial.println(" at speed: ");
          Serial.println(speedSelected);
          motor1a.attachPWM(3);
          motor1a.attachDirPin(2);
          motor1a.setSpeedByte(speedSelected);
          break;

         case 2:
          Serial.print("Turning motor 2: ");
          if (directionSelected == 1)
          {
            Serial.print("Forward");
          }
          else if (directionSelected == 2)
          {
            Serial.print("Backward");
          }
          else
          {
            Serial.print("NO DIRECTION");
          }
          Serial.print(" at speed: ");
          Serial.println(speedSelected);
          motor1a.attachPWM(5);
          motor1a.attachDirPin(4);
          motor1a.setSpeedByte(speedSelected);
          break;

         case 3:
          Serial.print("Turning motor 3: ");
          if (directionSelected == 1)
          {
            Serial.print("Forward");
          }
          else if (directionSelected == 2)
          {
            Serial.print("Backward");
          }
          else
          {
            Serial.print("NO DIRECTION");
          }
          Serial.print(" at speed: ");
          Serial.println(speedSelected);
          motor1a.attachPWM(6);
          motor1a.attachDirPin(7);
          motor1a.setSpeedByte(speedSelected);
          break;

         case 4:
          Serial.print("Turning motor 4: ");
          if (directionSelected == 1)
          {
            Serial.print("Forward");
          }
          else if (directionSelected == 2)
          {
            Serial.print("Backward");
          }
          else
          {
            Serial.print("NO DIRECTION");
          }
          Serial.print(" at speed: ");
          Serial.println(speedSelected);
          motor1a.attachPWM(9);
          motor1a.attachDirPin(8);
          motor1a.setSpeedByte(speedSelected);
          break;

         default:
          Serial.println("This is the default case, something has gone wrong");
      }
}

void serialFlush()
{
 while(Serial.available() > 0) 
 {
   char t = Serial.read();
 }
}
