// since 360 degrees for 48 steps, so each step is 7.5 degrees for full stepping, and for half stepping 96 steps and 3.75 degrees per step
// Johnson Ji with student number of 400499564, last 2 are 64, which is 64 seconds for period
// Hongliang Qi with student number of 400493278, last two number are 78, 78-33=45, which is 45 seconds per period
// For full stepping, each step is 7.5 deg; in period 1 (64 seconds), 48 steps in 64 seconds, each step needs 1.33 seconds.
// For half stepping, totally 96 steps, in period 1 (64 s), each step needs 0.67 s.
// For student 2 with a period of 45 s, each step needs 0.9375 s in full stepping,
// and for half stepping, each step needs 0.46875 s.
#include "mbed.h"
#include <string>
#include "LCD_DISCO_F429ZI.h"
#include "DebouncedInterrupt.h"
#include "chrono"
using namespace std::chrono;
// Motor control pins
DigitalOut s_yellow(PB_4);
DigitalOut s_black(PB_7);
DigitalOut s_red(PC_3);
DigitalOut s_grey(PA_5);
// User input and indicators
InterruptIn user_button(BUTTON1);
DigitalOut led(LED1);      // LED to indicate default speed (blinks when current speed equals default speed)
DigitalOut led2(LED2);
LCD_DISCO_F429ZI lcd;
Ticker stepper;
// Stepper control variables
int num_step = 4;         // Number of steps in current stepping mode (4 for full, 8 for half)
int step = 0;             // Current step index
bool CW = true;           // Direction flag: true means clockwise, false means counterclockwise
bool Full = true;         // Stepping mode flag: true for full stepping, false for half stepping
volatile bool motor_enabled = false;  // Flag to indicate if the motor is enabled
float current_period = 0.0f;  // Current period (seconds per step)
float default_period = 0.0f;  // Default period for the current student/mode
// Student stepper periods (in seconds per step)
// These are the default periods for full stepping; for half stepping we want double the speed so we use half of these values.
float student1_full_period = 1.33f;
float student1_half_period = 0.67f;
float student2_full_period = 0.9375f;
float student2_half_period = 0.46875f;
// External interrupt buttons for control
DebouncedInterrupt change_CW(PE_6);             // Interrupt to change motor direction
DebouncedInterrupt change_stepping_mode(PE_4);    // Interrupt to switch stepping mode (full/half)
DebouncedInterrupt increase_speed(PD_7);          // Interrupt to increase motor speed
DebouncedInterrupt decrease_speed(PD_5);          // Interrupt to decrease motor speed
// Application state definition
enum AppState {
    IDLE,
    Student1,
    Student2
};
volatile AppState state = IDLE; // Initial state is IDLE
// Full stepping state table (4 steps)
void Step1() { s_red = 1; s_grey = 0; s_yellow = 1; s_black = 0; } // Energize coils in specific pattern
void Step2() { s_red = 1; s_grey = 0; s_yellow = 0; s_black = 1; }
void Step3() { s_red = 0; s_grey = 1; s_yellow = 0; s_black = 1; }
void Step4() { s_red = 0; s_grey = 1; s_yellow = 1; s_black = 0; }
void (*full_step_table[])() = {Step1, Step2, Step3, Step4};
// Half stepping state table (8 steps)
void Step1_half() { s_red = 1; s_grey = 0; s_yellow = 1; s_black = 0; }
void Step2_half() { s_red = 1; s_grey = 0; s_yellow = 0; s_black = 0; }
void Step3_half() { s_red = 1; s_grey = 0; s_yellow = 0; s_black = 1; }
void Step4_half() { s_red = 0; s_grey = 0; s_yellow = 0; s_black = 1; }
void Step5_half() { s_red = 0; s_grey = 1; s_yellow = 0; s_black = 1; }
void Step6_half() { s_red = 0; s_grey = 1; s_yellow = 0; s_black = 0; }
void Step7_half() { s_red = 0; s_grey = 1; s_yellow = 1; s_black = 0; }
void Step8_half() { s_red = 0; s_grey = 0; s_yellow = 1; s_black = 0; }
void (*half_step_table[])() = {Step1_half, Step2_half, Step3_half, Step4_half,
                               Step5_half, Step6_half, Step7_half, Step8_half};
void updateLCD();
void updateSystem(bool resetSpeed);
// Stepper interrupt service routine (ISR)
// This routine is called at each tick of the timer to move the motor one step.
void StepperISR() {
    if (!motor_enabled) return; // If the motor is disabled, do nothing
    step = (step + 1) % num_step; // Increment step index with wrap-around
    // Choose the appropriate step based on stepping mode and direction
    if (Full) {  // Full stepping mode
        if (CW == true) {
            // For CW direction, use reverse order in the full step table
            full_step_table[(num_step - 1) - step]();
        } else {
            full_step_table[step]();
        }
    } else {  // Half stepping mode
        if (CW == true) {
            // For CW direction, use reverse order in the half step table
            half_step_table[(num_step - 1) - step]();
        } else {
            half_step_table[step]();
        }
    }
    // LED indication: blink the LED if current speed equals default speed; otherwise, keep LED off.
    if (fabs(current_period - default_period) < 0.001f) {
        led = !led;  // Toggle LED state for blinking
    } else {
        led = 0;     // Turn LED off
    }
}
// ISR for changing the motor direction
void change_CW_ISR() {
    if (state == IDLE) return; // Do nothing if in IDLE state
    CW = !CW;                  // Toggle the CW flag
    updateLCD();               // Update the LCD to reflect the change
}
// ISR for switching the stepping mode (full or half)
void change_mode_ISR() {
    if (state == IDLE) return; // Do nothing if in IDLE state
    Full = !Full;              // Toggle stepping mode
    num_step = Full ? 4 : 8;     // Update the number of steps (4 for full, 8 for half)
    step = 0;                  // Reset the step counter
    updateSystem(false);       // Update system without resetting the current speed ratio
}
// Function to update the LCD display
void updateLCD() {
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    // Display a startup message if in IDLE state
    if (state == IDLE) {
        lcd.DisplayStringAt(0, 10, (uint8_t*)"Press USER BUTTON", LEFT_MODE);
        lcd.DisplayStringAt(0, 40, (uint8_t*)"to start motor", LEFT_MODE);
        return;
    }
    // Display student information based on the current state
    if (state == Student1) {
        lcd.DisplayStringAt(0, 10, (uint8_t*)"Student: Johnson Ji", LEFT_MODE);
        lcd.DisplayStringAt(0, 30, (uint8_t*)"400499564", LEFT_MODE);
        lcd.DisplayStringAt(0, 60, (uint8_t*)"1.33s/0.67s", LEFT_MODE);
    } else {
        lcd.DisplayStringAt(0, 10, (uint8_t*)"Student: Hongliang Qi", LEFT_MODE);
        lcd.DisplayStringAt(0, 30, (uint8_t*)"400493278", LEFT_MODE);
        lcd.DisplayStringAt(0, 60, (uint8_t*)"0.94s/0.47s", LEFT_MODE);
    }
    // Display the current stepping mode and motor direction
    lcd.DisplayStringAt(0, 100, (uint8_t*)(Full ? "Mode: Full" : "Mode: Half"), LEFT_MODE);
    lcd.DisplayStringAt(0, 130, (uint8_t*)(CW ? "Dir: CW" : "Dir: CCW"), LEFT_MODE);
}
// Function to update system status (step period and LCD display)
// Parameter resetSpeed: if true, reset current speed to the default value; if false, maintain the current speed ratio when switching stepping mode.
// Modified for half stepping: In half stepping mode, we want the motor to run twice as fast,
// so the default period is set to half the provided half stepping value.
void updateSystem(bool resetSpeed) {
    if (state == IDLE) {
        motor_enabled = false;
        stepper.detach();
        updateLCD();
        return;
    }
    if (resetSpeed) {
        // Reset speed to default for the current student and mode
        if (state == Student1) {
            // For Student1: in full stepping, use student1_full_period;
            // in half stepping, use half of student1_half_period (i.e., twice the speed)
            current_period = Full ? student1_full_period : (student1_half_period / 2.0f);
        } else {
            // For Student2: in full stepping, use student2_full_period;
            // in half stepping, use half of student2_half_period
            current_period = Full ? student2_full_period : (student2_half_period / 2.0f);
        }
        default_period = current_period; // Update the default period
    } else {
        // When switching stepping mode, maintain the current speed ratio relative to the previous default.
        float speed_ratio = current_period / default_period;
        if (state == Student1) {
            default_period = Full ? student1_full_period : (student1_half_period / 2.0f);
        } else {
            default_period = Full ? student2_full_period : (student2_half_period / 2.0f);
        }
        current_period = default_period * speed_ratio;
    }
    // Enable the motor and attach the ISR with the updated period
    motor_enabled = true;
    stepper.detach();
    //stepper.attach(&StepperISR, current_period);
    stepper.attach(&StepperISR, std::chrono::duration_cast<std::chrono::microseconds>(current_period * 1s));
    updateLCD();
}
// Function to adjust the motor speed (increase or decrease)
// is_increase: true for increasing speed (decreasing the period), false for decreasing speed (increasing the period)
void adjustSpeed(bool is_increase) {
    if (state == IDLE) return;
    float delta = 0.1f;  // Speed adjustment increment
    if (is_increase) {
        // Decrease the period to increase speed, with a lower limit of 0.1 seconds
        current_period = (current_period - delta < 0.1f) ? 0.1f : current_period - delta;
    } else {
        // Increase the period to decrease speed, with an upper limit of 2.0 seconds
        current_period = (current_period + delta > 2.0f) ? 2.0f : current_period + delta;
    }
    // Restart the ticker with the new period
    stepper.detach();
    //stepper.attach(&StepperISR, current_period);
    stepper.attach(&StepperISR, std::chrono::duration_cast<std::chrono::microseconds>(current_period * 1s));
    updateLCD(); // Update the LCD display to show the new speed
}
// ISR for increasing the motor speed
void increase_speed_ISR() {
    adjustSpeed(true);
}
// ISR for decreasing the motor speed
void decrease_speed_ISR() {
    adjustSpeed(false);
}
// User button callback to switch student mode (cycles between Student1 and Student2)
// When switching students, the speed is reset to the default value for that student.
void user_button_pressed() {
    if (state == IDLE) {
        state = Student1; // First press enters Student1 mode
    } else if (state == Student1) {
        state = Student2;
    } else {
        state = Student1;
    }
    CW = true;
    Full = true;
    num_step = 4;
    step = 0;
    updateSystem(true);  // Reset speed to default when switching students
}
int main() {
    // Initialize the LCD display
    lcd.Clear(LCD_COLOR_BLACK);
    updateLCD();
    // Configure interrupts
    user_button.fall(&user_button_pressed);
    change_CW.attach(&change_CW_ISR, IRQ_FALL, 200, true);
    change_stepping_mode.attach(&change_mode_ISR, IRQ_FALL, 200, true);
    increase_speed.attach(&increase_speed_ISR, IRQ_FALL, 200, true);
    decrease_speed.attach(&decrease_speed_ISR, IRQ_FALL, 200, true);
    while (1) {
        // Main loop does nothing; all operations are handled via interrupts
    }
}
