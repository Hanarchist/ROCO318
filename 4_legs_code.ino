                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     /*-----------------------------------------------------------------------------------------------------
  ----------------------------------------------------------------------------------------------------
  ---------------------------------------------------------------------------------------------------

            _____   ____   _____ ____ ____  __  ___
           |  __ \ / __ \ / ____/ __ \___ \/_ |/ _ \
           | |__) | |  | | |   | |  | |__) || | (_) |
           |  _  /| |  | | |   | |  | |__ < | |> _ <
           | | \ \| |__| | |___| |__| |__) || | (_) |
           |_|  \_\\____/ \_____\____/____/ |_|\___/
                    SIMPLE SERVO CONTROLLER

  This is some super simple code for controlling servos via id and angle values.
  This has been made for the Arduino Nano with a PCA9685 16 channel i2c PWM breakout board.

                         -:Wiring:-
                ARDUINO NANO --->    PCA9685
                     A5      --->      SCL
                     A4      --->      SDA
                    GND      --->      GND
                     5V      --->      VCC

  THIS CODE UTILISES THE ADAFRUIT PWM SERVO DRIVER LIBRARY, TO ADD IT TO YOUR ARDUINO IDE:
  - Click on; Sketch
  - Click on; Include Library
  - Click on; Manage Libraries...
  - In the search bar that appears, type in "Adafruit PWM Servo Driver" and click install on the one that appears with the same name.
  Now you have the neccessary libraries to run the following code.

                     -:USING THE CODE:-
                      ----------------
  When uploaded you can then send simple commands to the board to control your servos.
  It must be in the following format:
    ServoNumber,ServoAngle

    So for example:

    UpdateServo(6, 90);

    This will move servo number 6 to an angle of 90 degrees.

  That is everything you need to know to use the board

  Debugging:

  If you send your servo an angle value and it does not line up, then you may need to calibrate the servo minimum and maximum pulse lengths
  Currently they have been calibrated for a range of 0 to 180 degrees for the Towerpro MG996R
  #define SERVOMIN  135 // This is the 'minimum' pulse length count (out of 4096) For the MG996R this wants to be 135
  #define SERVOMAX  482 // this is the 'maximum' pulse length count (out of 4096) For the MG996R this wants to be 482

  ------------------------------------------------------------------------------------------------
  -------------------------------------------------------------------------------------------------
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  135 // This is the 'minimum' pulse length count (out of 4096) For the MG996R this wants to be 135
#define SERVOMAX  482 // this is the 'maximum' pulse length count (out of 4096) For the MG996R this wants to be 482

Adafruit_PWMServoDriver PCA9685 = Adafruit_PWMServoDriver();

int ServoPositions[6][3] = {
    {90,70,40},       //servo foot up
    {40,70,90},
    {110,90,60}, //servo5 side ball up
    {60,90,110},
    {110,90,70},       // servo6 top ballacross
    {70,90,110}
  }; 
  /*
   2 - front left foor
   3 - front right foot
   4 - back right foot
   13 - back left foot
   5 - side servo, front left leg
   6 - top servo, front left leg
   7 -  side servo, front right leg
   8 - top servo, front right leg
   9 - side servo, back right leg
   10 - top servo, back right leg
   11 - side servo, back left leg
   12 - top servo, back left leg
    
    order is 
    2,4 up
    5,9 up
    6,10 forward
    8,12 back
    5,9 down
    2,4 down
    6,10 back
    3,13 up
    7,11 up
    8,12 forward
    6,10 back
    7,11 down
    3,13 down
   */


//----------------------------------------------------------------------------//
void setup() {
  Serial.begin(115200); //Nice faster speed
  PCA9685.begin();
  PCA9685.setPWMFreq(50); //Maximum speed this can be set to is 1600
  Wire.setClock(400000); // Fast 400khz I2C Mode
  Serial.println("PCA9685 Servo Controller Connected & Ready");
  
}


/*----------------------------------------------------------------------------
   __  __          _____ _   _
  |  \/  |   /\   |_   _| \ | |
  | \  / |  /  \    | | |  \| |
  | |\/| | / /\ \   | | | . ` |
  | |  | |/ ____ \ _| |_| |\  |
  |_|  |_/_/    \_\_____|_| \_|
  ----------------------------------------------------------------------------*/
void loop() {

  LegsMovePair(3,2,0);  //first pair of feet up - 3
  LegsMovePair(5,11,2);  //side servo up  - 5
  LegsMovePair(8,6,4); //top servo forwards - 8
  LegsMovePair(10,12,5); //top servo back - 12
  LegsMovePair(5,11,3);  //side servo down - 5
  LegsMovePair(3,2,1);  //feet down - 3
  LegsMovePair(8,6,5); //top servo back- 8
  LegsMovePair(4,13,0); //feet up - 13
  LegsMovePair(9,7,2); //side servo up - 9
  LegsMovePair(10,12,4); //top servo forward 12
  LegsMovePair(8,6,5); //top servo back- 8
  LegsMovePair(9,7,3); //side servo down - 9
  LegsMovePair(4,13,1); //feet down - 13
  LegsMovePair(10,12,4); //top servo back - 12


//  for (int ServoNumber = 1; ServoNumber <= 16; ServoNumber++) // set all servos to 90 degrees
//  {
//    UpdateServo(ServoNumber, 90);
//  }
//  delay(1000);
//
//  for (int ServoNumber = 1; ServoNumber <= 16; ServoNumber++) // set all servos to 80 degrees
//  {
//    UpdateServo(ServoNumber, 80);
//  }
//  delay(1000);
//
//  for (int ServoNumber = 1; ServoNumber <= 16; ServoNumber++ )// set all servos to 100 degrees
//  {
//    UpdateServo(ServoNumber, 100);
//  }
//  delay(1000);
//
//
}
//End of Main
//----------------------------------------------------------------------------//






/*----------------------------------------------------------------------------
   ______ _    _ _   _  _____ _______ _____ ____  _   _  _____
  |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
  | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___
  |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \
  | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
  |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/
  ----------------------------------------------------------------------------*/
void UpdateServo (int Id, int Angle) {

  double pulselength = map(Angle, 0, 180, SERVOMIN, SERVOMAX); //map the degree value to pulselength value
  PCA9685.setPWM(Id, 0, pulselength); //Send the angle to the appropiate servo

}
void LegsMovePair (int servoNum1,int servoNum2,int angleSet) {
  for (int stepp = 0; stepp <3; stepp++)
  {
    UpdateServo(servoNum1,ServoPositions[angleSet][stepp]);
    UpdateServo(servoNum2,ServoPositions[angleSet][stepp]);
  }
  delay(500);
}
