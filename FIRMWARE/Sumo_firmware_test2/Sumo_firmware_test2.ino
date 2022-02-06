// Wiring Connections.

#include <SharpIR.h>
#define model 20150


#define LED0          13   // Onboard LED 0
#define LED1          4   // Onboard LED 1
#define Start_PIN     2
#define KILL_PIN      12
#define BUZZ          9
#define BUTTON        8  // Start button
#define EDGE_L        7  // Left edge sensor
#define EDGE_R        7  // Right edge sensor
#define EDGE_M        7  // Right edge sensor
#define OPPONENT_L    A5   // Left opponent sensor
#define OPPONENT_R    A4 // Right opponent sensor
#define OPPONENT_FR   A1  // Front right opponent sensor
#define OPPONENT_FC   A2  // Front center opponent sensor
#define OPPONENT_FL   A0 // Front left opponent sensor

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
int AIN_1 = 11;
int AIN_2 = 10;
int BIN_1 = 6; 
int BIN_2 = 3; 



// Direction.
#define LEFT    0
#define RIGHT   1

// Global variables.
uint8_t searchDir = LEFT;


// Configure the motor driver.


/*******************************************************************************
 * Start Routine
 * This function should be called once only when the game start.
 *******************************************************************************/
 void startRoutine() {
  // Start delay.

 drive(-250, -250);
  // Turn right around 45 degress.
delay(180);

  drive(250, 0);

  delay(300);

  // Go straight.
  drive(150, 150);

  delay(300);

  // Turn left until opponent is detected.
    drive(-0, 150);

  uint32_t startTimestamp = millis();
  while (digitalRead(OPPONENT_FC)) {
    // Quit if opponent is not found after timeout.
    if (millis() - startTimestamp > 200) {
      break;
    }
  }
  
 }


/*******************************************************************************
 * Back Off
 * This function should be called when the ring edge is detected.
 *******************************************************************************/
void backoff(uint8_t dir) {
  // Stop the motors.

  drive(0, 0);

  delay(100);
  
  // Reverse.
  drive(-150, -150);

  delay(200);

  // Stop the motors.
 drive(0, 0);

  delay(100);

  // Rotate..
  if (dir == LEFT) {
    drive(-150, 150);
    Serial.println("Dir_Left");

  } else  {
    drive(150, -150);
    Serial.println("Dir_Right");

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
          Serial.println("BrakeOut Trig");
      // Stop the motors.
    drive(0, 0);
    delay(50);



      // Return to the main loop and run the attach program.
      return;
    }
  }

  // If opponent is not found, move forward and continue searching in the main loop..

  drive(150, 150);

  delay(100);
}


/*******************************************************************************
 * Search
 *******************************************************************************/
void search() {
  // Move in circular motion.
  if (searchDir == LEFT) {
    Serial.println("Search_Right");
    drive(100, 150);
  } 
  else if (searchDir == RIGHT) {
    Serial.println("Search_Right");
    drive(150, 100);
  }
  

}


/*******************************************************************************
 * Attack
 * Track and attack the opponent in full speed.
 * Do nothing if opponent is not found.
 *******************************************************************************/
void attack() {
  uint32_t attackTimestamp = millis();

  // Opponent in front center.
  // Go straight in full speed.
  if (digitalRead(OPPONENT_FC)) {
    drive(150, 150);
    Serial.println("Attack Front");

  }

  // Opponent in front left.
  // Turn left.
  else if (statusL()) {

    drive(-0, 150);
    Serial.println("Attack Front left");

  }

  // Opponent in front right.
  // Turn right.
  else if (statusR()) {

    drive(150, -0);
    Serial.println("Attack Front right");

  }

  // Opponent in left side.
  // Rotate left until opponent is in front.
  else if (!digitalRead(OPPONENT_L)) {
    drive(-150, 150);
    Serial.println("Attack LEFT");

    while (digitalRead(OPPONENT_FC)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 200) {
        break;
      }
    }
  }

  // Opponent in right side.
  // Rotate right until opponent is in front.
  else if (!digitalRead(OPPONENT_R) == 0) {

    drive(150,-150);
        Serial.println("Attack RIGHT");
    while (digitalRead(OPPONENT_FC)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 250) {
        break;
      }
    }
  }
  
}




/*******************************************************************************
 * Setup
 * This function runs once after reset.
 *******************************************************************************/
void setup() {
  Serial.begin(9600);
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
 

  // Turn off the LEDs.
  // Those LEDs are active low.
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  
  // Stop the motors.

  drive(0,0);

while (!digitalRead(Start_PIN)) {
    // While waiting, show the status of the edge sensor for easy calibration.
    if (!digitalRead(EDGE_L)) {
       digitalWrite(LED1, HIGH);
    } else {
      digitalWrite(LED1, LOW);
    }

    if (!digitalRead(EDGE_R)) {
       digitalWrite(LED0, HIGH);
    } else {
      digitalWrite(LED0, LOW);
    }
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


  }

  // Wait until button is released.
  while (!digitalRead(Start_PIN));

  // Turn on the LEDs.
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);

  startRoutine();
}

/*******************************************************************************
 * Main program loop.
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
      Serial.println("GO SEARCH void");


    }
    
    // Attack if opponent is in view.
    else {
      attack();
      Serial.println("GO to Attack void");

    }
  }

  // Stop the robot if the button is pressed.
  if (!digitalRead(Start_PIN)){
    // Stop the motors.
    drive(0,0);

    
  digitalWrite(LED0, LOW);
    // Loop forever here.
    while (1){
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
  int disL  =Sharp_L.distance();
  if (disL < 50)
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
  if (disR < 50)
  {
    statusRight = true;
  }
  else {
    statusRight = false;
  }
  return statusRight;
  
}
