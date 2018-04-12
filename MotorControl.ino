/*
 * BLDC_congroller 1.0
 * Arduino
 * by Patrick Eells
 *  based on instructable by David Glaser
 *  http://www.instructables.com/id/BLDC-Motor-Control-with-Arduino-salvaged-HD-motor/?ALLSTEPS
 *
 * Designed to work with the ST L6234 3-Phase Motor Driver IC
 *
 * Runs a 3D printed 3phase motor counterclockwise
 *
 * Motor speed is controlled by a single potentiometer
 * Motor position is determined with three Hall-Effect sensors

 * The Arduino receives outputs from 3 hall sensors (pins 2,3,4)
 * and converts their combination to 6 different commutation steps
 * PWM outputs on pins 9,10,11, at 32 kHz (corresponding to EN 1,2,3 respectively
 * 3 DO on pins 5,6,7 (IN 1,2,3)
 * Analog in 0 is connected to a potentiometer to change the PWM duty and switch motoring on and off.
 * 0-218: off
 * 219-1023: on
 * There are many lines commented out that were used for debugging by
 * printing various values to the serial connection.
 */

int HallState1; //Variables for the three hall sensors (3,2,1)
int HallState2;
int HallState3;
int HallVal = 1; //binary value of all 3 hall sensors

int Speed = 0; //speed level of the motor
int mOff = 0;
int throttle = 0; //this variable is used with analog in to measure the position of the throttle potentiometer

void setup() {
  pinMode(2,INPUT);    // Hall 1
  pinMode(3,INPUT);    // Hall 2
  pinMode(4,INPUT);    // Hall 3

// Outputs for the L6234 Motor Driver
  pinMode(5,OUTPUT);   // IN 1
  pinMode(6,OUTPUT);   // IN 2
  pinMode(7,OUTPUT);   // IN 3
  pinMode(9,OUTPUT);   // EN 1
  pinMode(10,OUTPUT);  // EN 2
  pinMode(11,OUTPUT);  //  EN 3


  //Serial.begin(9600); //uncomment this line if you will use the serial connection
  // also uncomment Serial.flush command at end of program.

/* Set PWM frequency on pins 9,10, and 11
// this bit of code comes from
http://usethearduino.blogspot.com/2008/11/changing-pwm-frequency-on-arduino.html
*/
  // Set PWM for pins 9,10 to 32 kHz
  //First clear all three prescaler bits:
  int prescalerVal = 0x07; //create a variable called prescalerVal and set it equal to the binary number "00000111"                                                       number "00000111"                                                      number "00000111"
  TCCR1B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"

  //Now set the appropriate prescaler bits:
  int prescalerVal2 = 1; //set prescalerVal equal to binary number "00000001"
  TCCR1B |= prescalerVal2; //OR the value in TCCR0B with binary number "00000001"

  // Set PWM for pins 3,11 to 32 kHz (Only pin 11 is used in this program)
  //First clear all three prescaler bits:
  TCCR2B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"

  //Now set the appropriate prescaler bits:

  TCCR2B |= prescalerVal2; //OR the value in TCCR0B with binary number "00000001"//First clear all three prescaler bits:

}
//MAIN LOOP OF THE PRGROM
void loop(){

   //time = millis();
  //prints time since program started
  //Serial.println(time);
  //Serial.print("\n");

  throttle = analogRead(0); //value of the throttle potentiometer
  Speed = map(throttle, 128, 1023, 0, 255); //motoring on is mapped to the top half of potentiometer
  mOff = map(throttle, 0, 127, 0, 255); //motoring off is mapped to the bottom half of the pot
  //Speed = 100; //used for debugging

  HallState1 = digitalRead(2);  // read input value from Hall 1
  HallState2  = digitalRead(3);  // read input value from Hall 2
  HallState3  = digitalRead(4);  // read input value from Hall 3
  //digitalWrite(8, HallState1);  // LEDs turned on when corresponding sensor is high - originally used for debugging
  //digitalWrite(9, HallState2);
  //digitalWrite(10, HallState3);

  HallVal = (HallState1) + (2*HallState2) + (4*HallState3); //Computes the binary value of the 3 Hall sensors

  /*Serial.print("H 1: "); // used for debugging
  Serial.println(HallState1);
  Serial.print("H 2: ");
  Serial.println(HallState2);
  Serial.print("H 3: ");
  Serial.println(HallState3);
  Serial.println(" ");
  */

  //Serial.println(mSpeed);
  //Serial.println(HallVal);
  //Serial.print("\n");

  // Monitor transistor outputs
  //delay(1000);
  /*T1 = digitalRead(2);
  //T1 = ~T1;
  T2 = digitalRead(4);
  //T2 = ~T2;
  T3 = digitalRead(5);
  //T3 = ~T3;
  Serial.print(T1);
  Serial.print("\t");
  Serial.print(T2);
  Serial.print("\t");
  Serial.print(T3);
  Serial.print("\n");
  Serial.print("\n");
  Serial.print(digitalRead(3));
  Serial.print("\t");
  Serial.print(digitalRead(9));
  Serial.print("\t");
  Serial.println(digitalRead(10));
  Serial.print("\n");
  Serial.print("\n");
  //delay(500);
  */

// Commutation for Motoring
// Each binary number has a case that corresponds to different transistors being turned on
// Bit Math is used to change the values of the output
// For tutorial on bitmath with the Arduino: http://www.arduino.cc/playground/Code/BitMath
// PORTD contains the outputs for the IN pins on the L6234 driver
// that determine whether the upper or lower transistor of each phase is used
// The outputs for the EN pins are controlled by the Arduino command analogWrite, which
// sets the duty of the PWM (0 = OFF, 255 = ON or throttle value that is controlled by the potentiometer).
/* for Counter clockwise motoring
  if (throttle > 511){
      switch (HallVal)
       {
        case 3:
          //PORTD = B011xxx00;  // Desired Output for pins 0-7 xxx refers to the Hall inputs, which should not be changed
          PORTD  &= B00011111;
          PORTD  |= B01100000;  //

          analogWrite(9,Speed); // PWM on Phase A (High side transistor)
          analogWrite(10,0);  // Phase B off (duty = 0)
          analogWrite(11,255); // Phase C on - duty = 100% (Low side transistor)
          break;
        case 1:
          //PORTD = B001xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;  //
          PORTD  |= B00100000;  //

          analogWrite(9,Speed); // PWM on Phase A (High side transistor)
          analogWrite(10,255); //Phase B on (Low side transistor)
          analogWrite(11,0); //Phase B off (duty = 0)
          break;
        case 5:
          //PORTD = B101xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;  //
          PORTD  |= B10100000;

          analogWrite(9,0);
          analogWrite(10,255);
          analogWrite(11,Speed);
          break;
        case 4:
          //PORTD = B100xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;
          PORTD  |= B10000000;  //

          analogWrite(9,255);
          analogWrite(10,0);
          analogWrite(11,Speed);
          break;
        case 6:
        //PORTD = B110xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;
          PORTD = B11000000;  //

          analogWrite(9,255);
          analogWrite(10,Speed);
          analogWrite(11,0);
          break;
        case 2:
          //PORTD = B010xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;
          PORTD  |= B01000000;  //

          analogWrite(9,0);
          analogWrite(10,Speed);
          analogWrite(11,255);
          break;
       }
     }
 // commutation off for motoring off
   else{
          //PORTD = B000xxx00;  // Desired Output for pins 0-7
            analogWrite(9,0);
            analogWrite(10,0);
            analogWrite(11,0);
            analogWrite(5,0);
            analogWrite(6,0);
            analogWrite(7,0);
          }


   //time = millis();
  //prints time since program started
  //Serial.println(time);
  //Serial.print("\n");
  //Serial.flush(); //uncomment this if you will use serial port for debugging
}
*/

// for counterClockwise motoring
if (throttle > 128){
      switch (HallVal)
       {
        case 4:
          //PORTD = B011xxx00;  // Desired Output for pins 0-7 xxx refers to the Hall inputs, which should not be changed
          PORTD  &= B00011111;
          PORTD  |= B01100000;  //

          analogWrite(9,Speed); // PWM on Phase A (High side transistor)
          analogWrite(10,0);  // Phase B off (duty = 0)
          analogWrite(11,255); // Phase C on - duty = 100% (Low side transistor)
          break;
        case 6:
          //PORTD = B001xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;  //
          PORTD  |= B00100000;  //

          analogWrite(9,Speed); // PWM on Phase A (High side transistor)
          analogWrite(10,255); //Phase B on (Low side transistor)
          analogWrite(11,0); //Phase B off (duty = 0)
          break;
        case 2:
          //PORTD = B101xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;  //
          PORTD  |= B10100000;

          analogWrite(9,0);
          analogWrite(10,255);
          analogWrite(11,Speed);
          break;
        case 3:
          //PORTD = B100xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;
          PORTD  |= B10000000;  //

          analogWrite(9,255);
          analogWrite(10,0);
          analogWrite(11,Speed);
          break;
        case 1:
        //PORTD = B110xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;
          PORTD = B11000000;  //

          analogWrite(9,255);
          analogWrite(10,Speed);
          analogWrite(11,0);
          break;
        case 5:
          //PORTD = B010xxx00;  // Desired Output for pins 0-7
          PORTD  &= B00011111;
          PORTD  |= B01000000;  //

          analogWrite(9,0);
          analogWrite(10,Speed);
          analogWrite(11,255);
          break;
       }
     }
 // commutation off for motoring off
   else{
          //PORTD = B000xxx00;  // Desired Output for pins 0-7
            analogWrite(9,0);
            analogWrite(10,0);
            analogWrite(11,0);
            analogWrite(5,0);
            analogWrite(6,0);
            analogWrite(7,0);
          }


   //time = millis();
  //prints time since program started
  //Serial.println(time);
  //Serial.print("\n");
  //Serial.flush(); //uncomment this if you will use serial port for debugging
}
