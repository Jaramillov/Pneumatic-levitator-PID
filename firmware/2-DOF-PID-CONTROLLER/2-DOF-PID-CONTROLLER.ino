#include "definitions.h"
#include <Wire.h>

// sampling time - keep at 25ms
float h = 0.025;
float tao = 2;
float alfa = 53.33333333;

float zeta = 1.5;
float wn = 15;
float N = 14;

// calibration offset to read distance as positive with 0 at the bottom of the tube
float calibration = 60;

// actuator limits
float umax = 3.5;
float umin = 0;

// control loop variables
float reference = 12;
float y;
float U;
float P;
float D;
float I;
float usat;
float y_ant;
float beta = 0;

double kp = (tao * (wn * wn) * ((4 * zeta) + 1)) / alfa;
double ki = (2 * tao * (wn * wn * wn)) / alfa;
double kd = (tao * wn * ((2 * zeta) + 2) - 1) / alfa;

double bi = ki * h;
double ad = kd / (kd + N * h);
double bd = kd * N / (kd + N * h);
double br = 0.9;

// equilibrium value of the control signal

void setup() {
    // initialize the sensor
    Wire.begin();
    while(!setupSensor()){
      vTaskDelay(1000);
    }
    setupPwm();
    vTaskDelay(100);

    // define the control task at maximum priority on core 1
    xTaskCreatePinnedToCore(
            controlTask,               // task function name
            "general controller task",
            8192,
            NULL,
            23, // task priority (0-24), 24 being the most critical
            NULL,
            CORE_1
    );
}

/***************************************************************************
*                2-DOF PID CONTROLLER FOR THE PNEUMATIC LEVITATOR
***************************************************************************/

static void controlTask(void *pvParameters) {

    // set how often the task repeats
    const TickType_t taskInterval = 1000 * h;  // every sampling period in ms = 1000*0.025 = 25ms

    // periodic task prototype
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount();

       // read ball position in centimeters
       y = calibration - (float) sensor.readRangeContinuousMillimeters() / 10;

       // 2-DOF PID control law
       P = kp * (beta * reference - y);
       D = ad * D - bd * (y - y_ant);
       U = P + D + I;

       // saturate control signal within actuator limits
       usat = constrain(U, umin, umax);

       voltsToFan(usat);

       // integral term with anti-windup back-calculation
       I = I + bi * (reference - y) + br * (usat - U);
       y_ant = y;

       // print reference, position and filtered control signal to serial monitor
       printf("%0.2f, %0.2f, %0.3f\n", reference, y, movingAverage(usat));

       // critical task: wait exactly taskInterval ms before next activation
       vTaskDelayUntil(&xLastWakeTime, taskInterval);
    }
}

void loop() {
    vTaskDelete(NULL);
}