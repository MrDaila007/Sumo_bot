// Wiring Connections.

#include <SharpIR.h>
#define model 20150


#define LED0          13  // Onboard LED 0 dublicate on PBC
#define LED1          4   // PBC LED 1
#define Start_PIN     2   // Start module start pin
#define KILL_PIN      12  // Start module kill pin
#define BUZZ          9   // Buzzer 
#define BUTTON        8   // Start button
#define EDGE_L        7   // Left edge sensor
#define EDGE_R        7   // Right edge sensor
#define EDGE_M        7   // Right edge sensor
#define OPPONENT_L    A5  // Left opponent sensor
#define OPPONENT_R    A4  // Right opponent sensor
#define OPPONENT_FR   A1  // Front right opponent sensor
#define OPPONENT_FC   A2  // Front center opponent sensor
#define OPPONENT_FL   A0  // Front left opponent sensor

//#define DEBUG_ENABLE
//#define SUMO_INTERFACE



////////////////////////////////////
//////////// SPEED ////////////////
//////////////////////////////////
#define MAX_SPEED 255
#define STRAIGHT_SPEED 150
#define SEARCH_SPEED_1 50
#define SEARCH_SPEED_2 100
#define ROTATE_TANK_SPEED 150
#define ROTATE_SPEED 100
#define BRAKEOUT_SPEED 250
#define ATACK_SPEED 200

////////////////////////////////

bool mode = 0;        // Mode 0: For 2S
// Mode 1: For 3S
int difference = 0;
uint32_t myTimer1;
int attackZone = 0; //0-12

SharpIR Sharp_R(OPPONENT_FR, model);
SharpIR Sharp_L(OPPONENT_FL, model);

/*
  int PWMA = 5;
  int PWMB = 6;
  int BIN_1 = 11;
  int BIN_2 = 10;
  int AIN_1 = 9;
  int AIN_2 = 8;
*/




//#ifdef TA6586
int AIN_1 = 10;
int AIN_2 = 11;
int BIN_1 = 6;
int BIN_2 = 3;



// Direction.
#define LEFT    0
#define RIGHT   1

// Global variables.
uint8_t searchDir = LEFT;


// Configure the motor driver.


/*******************************************************************************
   Start Routine
   This function should be called once only when the game start.
 *******************************************************************************/
void startRoutine() {
  // Start delay.

  drive(-MAX_SPEED, -MAX_SPEED);
  // Turn right around 45 degress.
  delay(400);

  drive(ROTATE_TANK_SPEED, 0);

  delay(300);

  // Go straight.
  drive(STRAIGHT_SPEED, STRAIGHT_SPEED);

  delay(100);

  // Turn left until opponent is detected.
  drive(-0, ROTATE_TANK_SPEED);

  uint32_t startTimestamp = millis();
  while (digitalRead(OPPONENT_FC)) {
    // Quit if opponent is not found after timeout.
    if (millis() - startTimestamp > 200) {
      break;
    }
  }

}


/*******************************************************************************
   Back Off
   This function should be called when the ring edge is detected.
 *******************************************************************************/
void backoff(uint8_t dir) {
  // Stop the motors.

  // Reverse.
  drive(-BRAKEOUT_SPEED, -BRAKEOUT_SPEED);
#ifdef DEBUG_ENABLE
  Serial.println("Reverse");
#endif

  delay(350);

  // Stop the motors.
  drive(0, 0);

  delay(100);

  // Rotate..
  if (dir == LEFT) {
    drive(-ROTATE_SPEED, ROTATE_SPEED);

#ifdef DEBUG_ENABLE
    Serial.println("Dir_Left");
#endif


  } else  {
    drive(ROTATE_SPEED, -ROTATE_SPEED);

#ifdef DEBUG_ENABLE
    Serial.println("Dir_Right");
#endif
  }
  delay(100);

  // Start looking for opponent.
  // Timeout after a short period.
  uint32_t uTurnTimestamp = millis();
  while (millis() - uTurnTimestamp < 200) {
    // Opponent is detected if either one of the opponent sensor is triggered.
    if ( digitalRead(OPPONENT_FC) ||
         statusL() ||
         statusR() ||
         !digitalRead(OPPONENT_L) ||
         !digitalRead(OPPONENT_R) ) {

      // Stop the motors.
      drive(0, 0);
      delay(50);

#ifdef DEBUG_ENABLE
      Serial.println("BrakeOut Trig");
#endif



      // Return to the main loop and run the attach program.
      return;
    }
  }

  // If opponent is not found, move forward and continue searching in the main loop..

  drive(STRAIGHT_SPEED, STRAIGHT_SPEED);

  delay(100);
}


/*******************************************************************************
   Search
 *******************************************************************************/
void search() {
  // Move in circular motion.
  if (searchDir == LEFT) {
    drive(SEARCH_SPEED_1, SEARCH_SPEED_2);
#ifdef DEBUG_ENABLE
    Serial.println("Search_Right");
#endif
  }
  else if (searchDir == RIGHT) {
    drive(SEARCH_SPEED_2, SEARCH_SPEED_1);
#ifdef DEBUG_ENABLE
    Serial.println("Search_Right");
#endif
  }


}


/*******************************************************************************
   Attack
   Track and attack the opponent in full speed.
   Do nothing if opponent is not found.
 *******************************************************************************/
void attack() {
  uint32_t attackTimestamp = millis();

  // Opponent in front center.
  // Go straight in full speed.
  if (digitalRead(OPPONENT_FC)) {
    drive(ATACK_SPEED, ATACK_SPEED);
#ifdef DEBUG_ENABLE
    Serial.println("Attack Front");
#endif
  }

  // Opponent in front left.
  // Turn left.
  else if (statusL()) {

    drive(0, ROTATE_TANK_SPEED);
#ifdef DEBUG_ENABLE
    Serial.println("Attack Front left");
#endif
  }

  // Opponent in front right.
  // Turn right.
  else if (statusR()) {

    drive(ROTATE_TANK_SPEED, 0);
#ifdef DEBUG_ENABLE
    Serial.println("Attack Front right");
#endif
  }

  // Opponent in left side.
  // Rotate left until opponent is in front.
  else if (!digitalRead(OPPONENT_L)) {
    drive(-ROTATE_SPEED, ROTATE_SPEED);
#ifdef DEBUG_ENABLE
    Serial.println("Attack LEFT");
#endif
    while (digitalRead(OPPONENT_FC)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 400) {
        break;
      }
    }
  }

  // Opponent in right side.
  // Rotate right until opponent is in front.
  else if (!digitalRead(OPPONENT_R) == 0) {

    drive(ROTATE_SPEED, -ROTATE_SPEED);
#ifdef DEBUG_ENABLE
    Serial.println("Attack RIGHT");
#endif
    while (digitalRead(OPPONENT_FC)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 400) {
        break;
      }
    }
  }

}




/*******************************************************************************
   Setup
   This function runs once after reset.
 *******************************************************************************/
void setup() {

#ifdef DEBUG_ENABLE || SUMO_INTERFACE
  Serial.begin(9600);
#endif

  pinMode(Start_PIN, INPUT);
  pinMode(KILL_PIN, INPUT);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(EDGE_L, INPUT);
  pinMode(EDGE_R, INPUT);

  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);

  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);

  bool currentMode = 0;


  // Turn off the LEDs.
  // Those LEDs are active low.
  digitalWrite(LED0, 0);
  digitalWrite(LED1, 0);

  // Stop the motors.

  drive(0, 0);

  while (!digitalRead(Start_PIN)) {
    // While waiting, show the status of the edge sensor for easy calibration.
    if (!digitalRead(EDGE_L)) {
      digitalWrite(LED1, HIGH);
    } else {
      digitalWrite(LED1, LOW);
    }
    /*
        if (!digitalRead(EDGE_R)) {
           digitalWrite(LED0, HIGH);
        } else {
          digitalWrite(LED0, LOW);
        }
    */
    if (digitalRead(BUTTON))
    {
      if (currentMode == 0)
      {
        digitalWrite(LED0, 0);
        mode = 1;
        //blink(2);
        difference = 80;
        currentMode == 1;

      }
      else if (currentMode == 1)
      {
        digitalWrite(LED0, 0);
        mode = 0;
        //blink(1);
        difference = 0;
        currentMode == 0;
      }
    }


#ifdef DEBUG_ENABLE
    bool FRONT_State = digitalRead(OPPONENT_FC);
    bool FRONT_R_State = statusR();
    bool FRONT_L_State = statusL();
    bool RIGHT_State = digitalRead(OPPONENT_R);
    bool LEFT_State = digitalRead(OPPONENT_L);
    bool LINE_State = digitalRead(EDGE_L);

    Serial.print(FRONT_L_State);
    Serial.print(" ");
    Serial.print(FRONT_State);
    Serial.print(" ");
    Serial.print(FRONT_R_State);
    Serial.print(" ");
    Serial.print(LEFT_State);
    Serial.print(" ");
    Serial.print(RIGHT_State);
    Serial.print(" ");

    Serial.print(!LINE_State);
    Serial.println(" ");
#endif

  }

  // Wait until button is released.
  while (!digitalRead(Start_PIN));

  // Turn on the LEDs.
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);

  startRoutine();
}

/*******************************************************************************
   Main program loop.
 *******************************************************************************/
void loop() {
  // Edge is detected on the left.
  if (digitalRead(EDGE_L)) {
    // Back off and make a U-Turn to the right.
    backoff(RIGHT);

    // Toggle the search direction.
    searchDir ^= 1;
  }

  // Edge is detected on the right.


  // Edge is not detected.
  else {
    // Keep searching if opponent is not detected.
    if ( digitalRead(OPPONENT_FC) &&
         statusL() &&
         statusR() &&
         !digitalRead(OPPONENT_L) &&
         !digitalRead(OPPONENT_R) ) {
      search();
#ifdef DEBUG_ENABLE
      Serial.println("GO SEARCH void");
#endif

    }

    // Attack if opponent is in view.
    else {
      attack();
#ifdef DEBUG_ENABLE
      Serial.println("GO to Attack void");
#endif
    }
  }

  // Stop the robot if the button is pressed.
  if (!digitalRead(Start_PIN)) {
    // Stop the motors.
    drive(0, 0);


    digitalWrite(LED0, LOW);
    // Loop forever here.
    while (1) {
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, HIGH);
      delay(1000);
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, LOW);
      delay(1000);
    }
  }

}

void drive(int left, int right) {

  // функция для управления скоростью и направления вращения моторов

  left = constrain(left, -255, 255);   // жестко ограничиваем диапозон значений
  right = constrain(right, -255, 255); // жестко ограничиваем диапозон значений

  if (left >= 0)
  {
    digitalWrite(AIN_2, LOW);
    analogWrite(AIN_1, abs(left));
  }
  else
  {
    digitalWrite(AIN_2, HIGH);
    analogWrite(AIN_1, abs(left));
  }
  if (right >= 0)
  {
    digitalWrite(BIN_1, LOW);
    analogWrite(BIN_2, abs(right));
  }
  else
  {
    digitalWrite(BIN_1, HIGH);
    analogWrite(BIN_2, abs(right));
  }
}

bool statusL()
{
  bool statusLeft;
  int disL  = Sharp_L.distance();
  if (disL < 60)
  {
    statusLeft = true;
  }
  else {
    statusLeft = false;
  }
  return statusLeft;

}
bool statusR()
{
  bool statusRight;
  int disR = Sharp_R.distance();
  if (disR < 60)
  {
    statusRight = true;
  }
  else {
    statusRight = false;
  }
  return statusRight;

}
void blink(uint8_t num)
{
  bool on = 0;
  uint8_t n = 0;
  for (int n = 0; n < num;  n++)
  {
    if (millis() - myTimer1 >= 300)
    { // ищем разницу (300 мс)
      myTimer1 = millis();              // сброс таймера
      if (on)
      {
        digitalWrite(LED0, 0);
      }
      else
      {
        digitalWrite(LED0, 1);
      }
    }
  }
}


#ifdef SUMO_INTERFACE
void sumo_int()
{
  bool FRONT_State = digitalRead(OPPONENT_FR);
  bool FRONT_R_State = statusR();
  bool FRONT_L_State = statusL();
  bool RIGHT_State = digitalRead(OPPONENT_R);
  bool LEFT_State = digitalRead(OPPONENT_L);
  //////////////////////////////////////////////
  //////////////////LOGIC//////////////////////
  ////////////////////////////////////////////

  if ( digitalRead(OPPONENT_FC) &&
       statusL() &&
       statusR() &&
       !digitalRead(OPPONENT_L) &&
       !digitalRead(OPPONENT_R))
  {
    attackZone = 0;
  }
  else if (digitalRead(OPPONENT_L))
  {
    attackZone = 1;
  }
  else if (statusL())
  {
    attackZone = 2;
  }
  else if (statusR())
  {
    attackZone = 5;
  }
  else if (digitalRead(OPPONENT_R))
  {
    attackZone = 6;
  }
  else if (digitalRead(OPPONENT_FC) && statusL())
  {
    attackZone = 7;
  }
  else if (digitalRead(OPPONENT_FC))
  {
    attackZone = 8;
  }
  else if (digitalRead(OPPONENT_FC) && statusR())
  {
    attackZone = 9;
  }
  else if (digitalRead(OPPONENT_FC) && statusL() && statusR())
  {
    attackZone = 12;
  }





  ///////////////////////////////////////////
  Serial.print(0);
  Serial.print(","); //Seperation mark
  Serial.print(attackZone);
  Serial.print(",");  //Seperation mark
  Serial.print(0);
  Serial.print(",");  //Seperation mark
  Serial.print(0);
  Serial.print(",");  //Seperation mark
  Serial.print(Sharp_R.distance());
  Serial.print(",");  //Seperation mark
  Serial.print(Sharp_L.distance());
  Serial.print(",");  //Seperation mark
  Serial.print(0);
  Serial.print(",");  //Seperation mark
  Serial.print(0);
  Serial.print(",");  //Seperation mark
  Serial.print('>'); //END Mark
}
#endif
