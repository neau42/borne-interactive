#ifndef BORNE_INTERACTIVE_H
# define BORNE_INTERACTIVE_H

# include <Wire.h>
# include <vl53l0x_class.h>
# include <neotimer.h>

// Create components.
// distance sensor
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

int reset_count = 0;

// Max detection distance 
#define  THRESHOLDDISTANCE 800
// Min detection distance
#define  THRESHOLDDISTANCEMIN 50
// default detection distance
#define  DEFAULTDISTANCE (THRESHOLDDISTANCE + 500)
// time out detection distance
// const int TIMEOUTDISTANCE = 500;

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

#endif