#include <MeMCore.h>
//Pin Definitions
#define ULTRASONIC_PIN 12 //12 for port 1, 10 for port 2
#define IR_RECEIVER_PIN A0 //connected to IR detector via port 4
#define LDR_PIN A1 //connected to LDR via port 4F
#define DECODER_B A3  //connected to decoder 1B via port 3
#define DECODER_A A2  //connected to decoder 1A via port 3

//Value Definitions
#define SPEED_OF_SOUND 345 //Default speed of sound 
#define TURNING_TIME 300 // The time duration (ms) for turning
#define FORWARD_TIME 700 
#define RED 1
#define GREEN 2
#define ORANGE 3
#define PINK 4
#define BLUE 5
#define WHITE 6

//Global Variables
MeBuzzer buzzer; // create the buzzer object
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeRGBLed led(0, 3); // shines colour detected by Mbot for verification
MeLineFollower FindBlack(PORT_2); //library line sensor function
uint8_t motorSpeed = 255; //full speed for max points
long colour_array[3] = {0}; //holds values for detected R,G,B

//setup
void setup() {
  delay(2000);
  //Set decoder pins to output
  pinMode(DECODER_A, OUTPUT); 
  pinMode(DECODER_B, OUTPUT);
    //set IR and LDR pins to input
  pinMode(IR_RECEIVER_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  //set both decoder pins to LOW so LEDs will be off 
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  //set led pin as 13
  led.setpin(13);
  Serial.begin(9600);
}

/*
*main loop for the entire maze-run
*/
void loop() {
  //checks for black line
  int yes_black = FindBlack.readSensors();
  //less than S1_OUT_S2_OUT means left sensor blacked or right sensor blacked or both blacked; if "none blacked", proceed to adjust_angle();
  if (yes_black < S1_OUT_S2_OUT) {
    stop_motor();
    check_colour();
  }
  else {
    adjust_angle();
  }
}
/*
 * Stops the motors.
 */
void stop_motor(){
  leftMotor.stop();
  rightMotor.stop();
}
/*
 * Make the robot go forward.
 */
void go_forward() {
  leftMotor.run(-motorSpeed); //left always negative for forward
  rightMotor.run(motorSpeed);
}
/*
 * Moves the robot backwards by a given time(delay).
 * Stop the motors afterwards.
 */
void reverse(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop_motor();
}
/*
 * Turn the robot right by a given time(delay).
 * Stop the motors afterwards.
 */
void right_turn(int time) {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop_motor();
}
/*
 * Turns the robot left by a given time(delay).
 * Stop the motors afterwards.
 */
void left_turn(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(time);
  stop_motor();
}
/*
 * Turns the robot right by a given time(turn_time),
 * moves it forward by a given time(fwd_time),
 * do a stop to break motion
 * and then turn it right again by a given time(turn_time).
 */
void right_turn_2_grid(int turn_time, int fwd_time) {
  right_turn(turn_time);
  go_forward();
  delay(fwd_time);
  stop_motor();
  right_turn(turn_time);
}
/*
 * Turns the robot left by a given time(turn_time),
 * moves it forward by a given time(fwd_time),
 * do a stop to break motion
 * and then turn it left again by a given time(turn_time).
 */
void left_turn_2_grid(int turn_time, int fwd_time) {
  left_turn(turn_time);
  go_forward();
  delay(fwd_time);
  stop_motor();
  left_turn(turn_time);
}
/*
 * Turns the robot to the right by a given time(tuned with ‘/2-20’ for eg so that the Mbot can turn exactly 90 degrees),
 * move back for 100ms so that Mbot can adjust to tight angles
 * Turn the robot right again by 90 degrees, this time a little less as our Mbot appears to require that to turn exactly 90 degrees
 * Stop motor to clear previous turn command; wait for next command
 */
void rotate_back(int time) {
  right_turn(time / 2 - 20);
  reverse(100);
  right_turn(time / 2 - 25);
  right_turn(time);
  stop_motor();
}
 /*
 * utilises ultrasonic sensor to detect distance of Mbot from the left wall
 */
float distance_left() {
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_PIN, HIGH, 3000); //3000s is the max timeout duration
  float distance = (((float)SPEED_OF_SOUND * (float)duration / 10000.0) / 2.0) - 3.5;
    //Serial.println(duration);
    //Serial.print("distance: ");
    //Serial.println(distance);
    //Serial.println("cm");
   delay(10);
//short delay to smoothen the adjusting frequency while also maintaining high levels of responsiveness
  return distance;
}

/*
 *This function adjusts Mbot’s angle based on its proximity to the left wall.
 *Ultrasonic sensor is utilised to do either a slow steer or fast steer depending 
 *on how close the Mbot is to the left wall.
 *When no left wall is sensed, this function will then utilise IR to steer based 
 *on the distance from the right wall.
 */
void adjust_angle() {
  float distance = distance_left();
  float distance_r = distance_right();


  if (distance > 15.5 || distance < 0.0) {
    //fast steer to left
    if(distance_r > 800){
      leftMotor.run(0.3 * -motorSpeed);
      rightMotor.run(motorSpeed);
      return;
    //fast steer to right
    }else if(distance_r<=500){
      leftMotor.run(-motorSpeed);
      rightMotor.run(0.3 * motorSpeed);
      return;
    }
    go_forward();
    return;
  }
  //using 7cm as the mid point
  //fast steer to right
  if (distance < 3) {
    leftMotor.run(-motorSpeed);
    rightMotor.run(0.3*motorSpeed);
    return;
  }
  //slow steer to left
  if (distance > 9) {
    leftMotor.run(-0.7*motorSpeed);
    rightMotor.run(motorSpeed);
    return;
  }
  //fast steer to left
  if (distance > 11) {
    leftMotor.run(-0.3*motorSpeed);
    rightMotor.run(motorSpeed);
    return;
  }
  //slow steer to the right
  if (distance < 5) {
    leftMotor.run(-motorSpeed);
    rightMotor.run(0.7*motorSpeed);
    return;
  }
  //if position correct, proceed forward
  go_forward();
}


/*
 * Shine Red LED
 */


void shine_red() {
  digitalWrite(DECODER_B, HIGH);
  digitalWrite(DECODER_A, HIGH);
}
/*
 * Shine Green LED
 */


void shine_green() {
  digitalWrite(DECODER_B, HIGH);
  digitalWrite(DECODER_A, LOW);
}


/*
 * Shine Blue LED
 */


void shine_blue() {
  digitalWrite(DECODER_B, LOW);
  digitalWrite(DECODER_A, HIGH);
}


/*
 * Identifies the colour given by the RGB values detected
 * in colour_array. 
 * Colours are determined with the boolean analysis method on
 * either R or G or B values from colour_array.
 */
int identify_colour() {
  int red = colour_array[0];
  int green = colour_array[1];
  int blue = colour_array[2];


 if (red<900) {
    if (blue>910) {
      led.setColor(0, 0, 255);
      led.show();
    //  Serial.println("Blue");
       return BLUE;
    }


    else
    {
      led.setColor(0, 255, 0);
      led.show();
  //   Serial.println("Green");
      return GREEN;
    }
 } else
  {
      if (green < 890)
      {
        if (green > 855)
        {
          led.setColor(255, 192, 203);
          led.show();
          // Serial.println("Pink");
          return PINK;
        } 
        else if (green > 825)
          {
            led.setColor(255, 165, 0);
            led.show();
            // Serial.println("Orange");
            return ORANGE;
          } else{
                 led.setColor(255, 0, 0);
                 led.show();
                 //Serial.println("Red");
                 return RED;
                }
      } else
      {
        led.setColor(255, 255, 255);
        led.show();
        //Serial.println("White");
        return WHITE;
       }
  }
}


 
/*
* Turn based on colour
* Turning time will be adjusted if necessary by doing arithmetic to TURNING_TIME
*/
void action(int colour) {
  if (colour == RED) {
    left_turn(TURNING_TIME);
    return;
  }


  if (colour == GREEN) {
    right_turn(TURNING_TIME);
    return;
  }


  if (colour == ORANGE) {
    rotate_back(TURNING_TIME);
    return;
  }


  if (colour == PINK) {
    left_turn_2_grid(TURNING_TIME+12, FORWARD_TIME);
    return;
  }


  if (colour == BLUE) {
    right_turn_2_grid(TURNING_TIME, FORWARD_TIME);
    return;
  }


  if (colour == WHITE) {
    endSong();
    return;
  }
}
/*
* The greatest endsong of the lab: Rick Astley - Never Gonna Give You Up
*/
void endSong() {
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0


  int tempo = 114;


  int melody[] = {


    NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,


    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8, //23
    NOTE_E5, 4, NOTE_D5, 2, REST, 4,


  };


  int notes = sizeof(melody) / sizeof(melody[0]) / 2;


  int wholenote = (60000 * 4) / tempo;


  int divider = 0, noteDuration = 0;
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    buzzer.tone(melody[thisNote], noteDuration);
  }
}


/**
 * Shines RGB
 * For a loop of 5 iterations per R,G and B
 * Omit negative LDR values
 * colour_array[] collects the avg LDR values for R,G,B
**/
void check_colour() {
 
  for (int i = 0; i < 3; i++)
  {
    if (i == 0)
    {
      shine_red();
    }


    else if (i == 1)
    {
      shine_green();
    }


    else
    {
      shine_blue();
    }
   
    int total = 0;
    int flag = 5;


    delay(200);


    for (int j = 0; j < 5; j++)
    {
      long reading = (analogRead(LDR_PIN)); /*- black_array[i]) * 255) / (white_array[i] - black_array[i]);*/
      //previously used grey diff, now no more
      //Serial.println(reading);


      if (reading <= 0)
      {
        flag -= 1;
      }


      if (reading > 0)
      {
        total += reading;
      }
    }
   //avg LDR value per R,G,B = total/flag
    colour_array[i] = total / flag;
 
   Serial.println(colour_array[i]);


    digitalWrite(DECODER_B, LOW);
    digitalWrite(DECODER_A, LOW);


    delay(100);
  }
   // Proceeds to send colour array to identify the colour
    int colour = identify_colour();
   // action will be taken depending on what the colour is
    action(colour);
}


/*
 * Takes in the drop in voltage of the IR sensor, 
 * reading=800==3cm from right wall
 * reading=500==3cm from left wall
 * Above is according to calibrated data from experiments
 */
float distance_right() {
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  float reading = analogRead(IR_RECEIVER_PIN) ;
  return reading;
}




