/* Includes ------------------------------------------------------------------*/
#include "borne_interactive.h"
/* Setup ---------------------------------------------------------------------*/

void setup()
{
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

    if (sensor_up.InitSensor(VL53L0x_DEFAULT_DEVICE_ADDRESS+7))
    {
        init_ok_up = false;
        Serial.println("Init sensor_up failed...");

    }
    else
        Serial.println("Init sensor_up ok...");
    delay(1000);
    if(sensor_down.InitSensor(VL53L0x_DEFAULT_DEVICE_ADDRESS+2))
    {
        init_ok_down = false;
        Serial.println("Init sensor_down failed...");
    }
    else
        Serial.println("Init sensor_down ok...");
}

/* Loop ----------------------------------------------------------------------*/

void loop()
{
    state_t old_state = idle;

    // Led blinking.
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    getDistance();
    computeState();
    if (state != old_state)
    {
        printState();
        old_state = state;
    }
}

void print_distance(uint32_t dist_up, uint32_t dist_down)
{
    Serial.print("distance: |");
    Serial.print(dist_up);
    Serial.print("\t|");
    Serial.print(dist_down);
    Serial.println("\t|");
}

/*
 * read the distance data from sensors and check if someone is standing in front of the terminal.
 */
 
void getDistance()
{
    uint32_t distance_up = DEFAULTDISTANCE;
    uint32_t distance_down = DEFAULTDISTANCE;
    uint32_t distance;

    if(init_ok_up)
    {
        if (sensor_up.GetDistance(&distance) == VL53L0X_ERROR_NONE)
        {
            distance_up = distance;
            timeout_up.reset();
        }
        else if(timeout_up.repeat())
            distance_up = DEFAULTDISTANCE;
    }
    if(init_ok_down)
    { 
        if (sensor_down.GetDistance(&distance) == VL53L0X_ERROR_NONE)
        {
            distance_down = distance;
            timeout_down.reset();
        }
        else if(timeout_down.repeat())
            distance_down = DEFAULTDISTANCE;
    }
    if(distance_up > THRESHOLDDISTANCEMIN)
        presence_up = distance_up < THRESHOLDDISTANCE;
    
    if(distance_down > THRESHOLDDISTANCEMIN)
        presence_down = distance_down < THRESHOLDDISTANCE;
    
    if (presence_down || presence_up)//dbg
        print_distance(distance_up, distance_down);
}

/*
 * compute and change the current state 
 */

void case_idle()
{
    stopScreen();
    if(presence_down == true)
    {
        if(!timeout_presence_in.started())
            timeout_presence_in.start();
        else if(timeout_presence_in.done())
        {
            state = person_detected;
            timeout_presence_in.reset();
            timeout_presence_in.stop();
        }
    }
    else
    {
        state = idle;
        timeout_presence_in.reset();
        timeout_presence_in.stop();
    }
}

void case_person_detected()
{
    stopScreen();
    if(presence_down == false)
    {
        if(!timeout_presence_out.started())
            timeout_presence_out.start();
        else if(timeout_presence_out.done())
        {
            state = screen_reset;
            timeout_presence_out.reset();
            timeout_presence_out.stop();
        }
    }
    else if(presence_up == true)
    {
        state = move_up;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
    }
    else
    {
        state = move_down;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
    }
}

void case_move_up()
{
    if(presence_down == true)
            moveUpScreen();
    if(presence_down == false)
    {
        stopScreen();
        if(!timeout_presence_out.started())
            timeout_presence_out.start();
        else if(timeout_presence_out.done())
        {
            state = screen_reset;
            timeout_move_up.reset();
            timeout_move_up.stop();
            timeout_presence_out.reset();
            timeout_presence_out.stop();
        }
    }
    else if(presence_up == true)
    {
        if(!timeout_move_up.started())
            timeout_move_up.start();
        else if(timeout_move_up.done())
        {
            state = move_stop;
            timeout_move_up.reset();
            timeout_move_up.stop();
            moveDownScreen();
            delay(200);
        }
        else
            state = move_up;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
    }
    else
    {
        state = move_stop;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
        timeout_move_up.reset();
        timeout_move_up.stop();
        moveDownScreen();
        delay(1000);
    }

}

void case_move_down()
{
    if(presence_down == true)
    moveDownScreen();
    if(presence_down == false)
    {
        stopScreen();
        if(!timeout_presence_out.started())
            timeout_presence_out.start();
        else if(timeout_presence_out.done())
        {
            state = screen_reset;
            timeout_move_down.reset();
            timeout_move_down.stop();
            timeout_presence_out.reset();
            timeout_presence_out.stop();
        }
    }
    else if(presence_up == false)
    {
        if(!timeout_move_down.started())
            timeout_move_down.start();
        else if(timeout_move_down.done())
        {
            state = move_stop;
            timeout_move_down.reset();
            timeout_move_down.stop();
        }
        else
            state = move_down;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
    }
    else
    {
        state = move_up;
        timeout_move_down.reset();
        timeout_move_down.stop();
        timeout_presence_out.reset();
        timeout_presence_out.stop();
    }
}

void case_move_stop()
{
    stopScreen();
    if(presence_down == false)
    {
        if(!timeout_presence_out.started())
            timeout_presence_out.start();
        else if(timeout_presence_out.done())
        {
            state = screen_reset;
            timeout_presence_out.reset();
            timeout_presence_out.stop();
        }
    }
    else
    {
        state = move_stop;
        timeout_presence_out.reset();
        timeout_presence_out.stop();
    }
}

void computeState()
{
    switch(state)
    {
        case idle:
            case_idle();
            break;
        case person_detected:
            case_person_detected();
            break;
        case move_up:
            case_move_up();
            break;
        case move_down:
            case_move_down();
            break;
        case move_stop:
            case_move_stop();
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
void printState()
{
    switch(state)
    {
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

void control_motor(int pwm, int ina, int inb, int sel)
{
    analogWrite(PWM, pwm);
    digitalWrite(InA,ina);
    digitalWrite(InB,inb);
    digitalWrite(SEL,sel);  
}

void moveUpScreen()
{
    Serial.println("motor:|moving_up|");
    control_motor(255, HIGH, LOW, LOW);
}

/*
 * move down the screen
 */
void moveDownScreen()
{
    Serial.println("motor:|moving_down|");
    control_motor(255, LOW, HIGH, HIGH);
}

/*
 * stop moving the screen
 */
void stopScreen()
{
    // Serial.println("motor:|stopping|");
    control_motor(0, LOW, LOW, HIGH);
}

/*
 * reset the height of screen
 */
void resetScreen()
{
    Serial.println("resetting");
    if(reset_count < 8)
        moveDownScreen();
    else
        moveUpScreen();
    reset_count++;
}

/*
 * The method will run when the interrupter is on.
 */
void ISR() {
    if(state == screen_reset)
    {
        stopScreen();
        state = idle;
        reset_count = 0;
    }
}
