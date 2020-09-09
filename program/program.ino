/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <vl53l0x_class.h>
#include <neotimer.h>

// Create components.
// distance sansor
TwoWire WIRE1(PB4, PA7); 
VL53L0X sensor_up(&WIRE1, PB0, 0); 
VL53L0X sensor_down(&WIRE1, PA12, 0); 

//pin input 
const int interrupter = PB7;

// pin output for motor
const int InA = PB6; 
const int InB = PB1;
const int SEL = PA11;
const int PWM = PA8;

bool presence_up = false;
bool presence_down = false;

Neotimer timeout_up = Neotimer(500);
Neotimer timeout_down = Neotimer(500);

Neotimer timeout_move_up = Neotimer(2500);
Neotimer timeout_move_down = Neotimer(3500);

Neotimer timeout_presence_out = Neotimer(5000);
Neotimer timeout_presence_in = Neotimer(3000);

long lt_aff=0;
int reset_count = 0;
int val_interrupter = 0;

// Max detection distance 
const int THRESHOLDDISTANCE = 800;
// Min detection distance
const int THRESHOLDDISTANCEMIN = 50;
// default detection distance
const int DEFAULTDISTANCE = THRESHOLDDISTANCE+500;
// time out detection distance
const int TIMEOUTDISTANCE = 500;

enum state_t {
  idle,
  person_detected,
  move_up,
  move_down,
  move_stop,
  screen_reset
};

state_t state = screen_reset;

void getDistance();
void computeAction();
void moveUpScreen();
void moveDownScreen();
void stopScreen();
void resetScreen();
void computeState();
void printState();
void ISR();

bool init_ok_up=true;
bool init_ok_down=true;

/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(9600);

  pinMode(interrupter, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupter), ISR, RISING); 

  pinMode(InA, OUTPUT);
  pinMode(InB, OUTPUT);
  pinMode(SEL ,OUTPUT);
  pinMode(PWM ,OUTPUT);

  // Initialize I2C bus.
  WIRE1.begin();

  // Switch off VL53L0X component.
  sensor_up.VL53L0X_Off();
  sensor_down.VL53L0X_Off();
  delay(1000);
  
  // Initialize VL53L0X top component.
  status = sensor_up.InitSensor(VL53L0x_DEFAULT_DEVICE_ADDRESS+7);
  if(status)
  {
    init_ok_up=false;
    Serial.println("Init sensor_up failed...");
  }
  else
  {
    Serial.println("Init sensor_up ok...");
  }
  delay(1000);
  status = sensor_down.InitSensor(VL53L0x_DEFAULT_DEVICE_ADDRESS+2);
  if(status)
  {
    init_ok_down=false;
    Serial.println("Init sensor_down failed...");
  }
  else
  {
    Serial.println("Init sensor_down ok...");
  }
}

long long watchdog = millis();

/* Loop ----------------------------------------------------------------------*/

void loop() {
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  watchdog= millis();

  getDistance();
  computeState();
  printState();
  
  if(millis()-watchdog>5000)
  {
    Serial.println("Watch dog !");
    state = screen_reset;
  }  
}

/*
 * read the distance data from sensors and check if someone is standing in front of the terminal.
 */
void getDistance()
{
  
  uint32_t distance_up;
  uint32_t distance_down;
  uint32_t distance;
  int status;
  
  if(init_ok_up)
  {
    status = sensor_up.GetDistance(&distance);
    if (status == VL53L0X_ERROR_NONE)
    {
      distance_up=distance;
      timeout_up.reset();
    }
    else if(timeout_up.repeat())
    {
      distance_up=DEFAULTDISTANCE;
    }
    
  }
  
  if(init_ok_down)
  { 
    status = sensor_down.GetDistance(&distance);
    if (status == VL53L0X_ERROR_NONE)
    {
      distance_down=distance;
      timeout_down.reset();
    }
    else if(timeout_down.repeat())
    {
      distance_down=DEFAULTDISTANCE;
    }
  }
  
  if(distance_up>THRESHOLDDISTANCEMIN)
  {
    presence_up=distance_up<THRESHOLDDISTANCE;
  }
  
  if(distance_down>THRESHOLDDISTANCEMIN)
  {
    presence_down=distance_down<THRESHOLDDISTANCE;
  }
  
  Serial.print("distance:|");
  Serial.print(distance_up);
  Serial.print("|");
  Serial.print(distance_down);
  Serial.println("|");
}

/*
 * compute and change the current state 
 */
void computeState(){
  switch(state){
    case idle:
      stopScreen();
      if(presence_down == true){
        if(!timeout_presence_in.started()){
          timeout_presence_in.start();
        }else if(timeout_presence_in.done()){
          state = person_detected;
          timeout_presence_in.reset();
          timeout_presence_in.stop();
        }
      }else{
        state = idle;
        timeout_presence_in.reset();
        timeout_presence_in.stop();
      }
      break;
      
    case person_detected:
      stopScreen();
      if(presence_down == false){
        if(!timeout_presence_out.started()){
          timeout_presence_out.start();
        }else if(timeout_presence_out.done()){
          state = screen_reset;
          timeout_presence_out.reset();
          timeout_presence_out.stop();
        }
      }else if(presence_up == true){
        state = move_up;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
      }else{
        state = move_down;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
      }
      break;
      
    case move_up:
      if(presence_down == true) moveUpScreen();
      if(presence_down == false){
        stopScreen();
        if(!timeout_presence_out.started()){
          timeout_presence_out.start();
        }else if(timeout_presence_out.done()){
          state = screen_reset;
          timeout_move_up.reset();
          timeout_move_up.stop();
          timeout_presence_out.reset();
          timeout_presence_out.stop();
        }
      }else if(presence_up == true){
        if(!timeout_move_up.started()){
          timeout_move_up.start();
        }else if(timeout_move_up.done()){
          state = move_stop;
          timeout_move_up.reset();
          timeout_move_up.stop();
          moveDownScreen();
          delay(200);
        }else{
          state = move_up;
        }
        timeout_presence_out.reset();
        timeout_presence_out.stop();
      }else{
        state = move_stop;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
        timeout_move_up.reset();
        timeout_move_up.stop();
        moveDownScreen();
        delay(1000);
      }
      break;
      
    case move_down:
      if(presence_down == true) moveDownScreen();
      if(presence_down == false){
        stopScreen();
        if(!timeout_presence_out.started()){
          timeout_presence_out.start();
        }else if(timeout_presence_out.done()){
          state = screen_reset;
          timeout_move_down.reset();
          timeout_move_down.stop();
          timeout_presence_out.reset();
          timeout_presence_out.stop();
        }
      }else if(presence_up == false){
        if(!timeout_move_down.started()){
          timeout_move_down.start();
        }else if(timeout_move_down.done()){
          state = move_stop;
          timeout_move_down.reset();
          timeout_move_down.stop();
        }else{
          state = move_down;
        }
        timeout_presence_out.reset();
        timeout_presence_out.stop();
      }else{
        state = move_up;
        timeout_move_down.reset();
        timeout_move_down.stop();
        timeout_presence_out.reset();
        timeout_presence_out.stop();
      }
      break;
      
    case move_stop:
      stopScreen();
      if(presence_down == false){
        if(!timeout_presence_out.started()){
          timeout_presence_out.start();
        }else if(timeout_presence_out.done()){
          state = screen_reset;
          timeout_presence_out.reset();
          timeout_presence_out.stop();
        }
      }else{
        state = move_stop;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
      }
      break;

    case screen_reset:
      resetScreen();
      break;
       
    default:
      break;
  }
}

/*
 * show the state info in the serial
 */
void printState(){
  switch(state){
    case idle:
      Serial.println("state:|idle|");
      break;
    case person_detected:
      Serial.println("state:|person_detected|");
      break;
    case move_up:
      Serial.println("state:|move_up|");
      break;
    case move_down:
      Serial.println("state:|move_down|");
      break;
    case move_stop:
      Serial.println("state:|move_stop|");
      break;
     case screen_reset:
      Serial.println("state:|screen_reset|");
    default:
      break;
  }
}

/*
 * move up the screen 
 */
void moveUpScreen()
{
  Serial.println("motor:|moving_up|");
  analogWrite(PWM, 255);
  digitalWrite(InA,HIGH);
  digitalWrite(InB,LOW);
  digitalWrite(SEL,LOW);
}

/*
 * move down the screen
 */
void moveDownScreen()
{
  Serial.println("motor:|moving_down|");
  analogWrite(PWM, 255);
  digitalWrite(InA,LOW);
  digitalWrite(InB,HIGH);
  digitalWrite(SEL,HIGH);
  
}

/*
 * stop moving the screen
 */
void stopScreen()
{
  Serial.println("motor:|stopping|");
  analogWrite(PWM, 0);
  digitalWrite(InA,LOW);
  digitalWrite(InB,LOW);
  digitalWrite(SEL,HIGH);
}

/*
 * reset the height of screen
 */
void resetScreen()
{
  Serial.println("resetting");
  if(reset_count < 8){
    moveDownScreen();
  }else{
    moveUpScreen();
  }
  reset_count++;
}

/*
 * The method will run when the interrupter is on.
 */
void ISR() {
  if(state == screen_reset){
    stopScreen();
    state = idle;
    reset_count = 0;
  }
}
