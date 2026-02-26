#include <VL53L0X.h>

// ESP32 core definitions
#define CORE_0 0   // base core used by some processor tasks
#define CORE_1 1   // user core - place critical tasks here

// PWM configuration
#define FREQUENCY_PWM   25000  // PWM frequency in Hz
#define RESOLUTION_PWM  11     // PWM resolution in bits
#define PIN_PWM         2      // PWM output pin

// scaling factor: converts volts to PWM counts (0-2047 for 11-bit at 12V)
float voltsToPwm = (pow(2, RESOLUTION_PWM) - 1) / 12;

// laser sensor
// connect GPIO 22 to SCL
// connect GPIO 21 to SDA
VL53L0X sensor;

bool setupSensor(void) {
    sensor.setTimeout(100);
    if (!sensor.init()) {
        printf("Sensor not detected\n");
        return false;
    }
    sensor.startContinuous();
    sensor.setMeasurementTimingBudget(20000);
    return true;
}

void setupPwm(void) {
    ledcAttach(PIN_PWM, FREQUENCY_PWM, RESOLUTION_PWM);
    // if using PlatformIO, replace the line above with:
    //ledcSetup(0, FREQUENCY_PWM, RESOLUTION_PWM);
    //ledcAttachPin(PIN_PWM, 0);
}

void voltsToFan(float volts) {
    uint16_t pwm = volts * voltsToPwm;
    ledcWrite(PIN_PWM, pwm);
}

float movingAverage(float newValue) {
    const int filterSize = 20;
    static float filter_values[filterSize] = {0.0};  // stores the last 'filterSize' values
    static int index = 0;                             // current index in the circular buffer
    static float sum = 0.0;                           // running sum of values in the buffer
    static int count = 0;                             // number of values added so far

    // subtract the oldest value and insert the new one
    sum = sum - filter_values[index] + newValue;
    filter_values[index] = newValue;

    // advance index with wraparound (circular buffer)
    index = (index + 1) % filterSize;

    // ramp up count until the buffer is full
    if (count < filterSize) {
        count++;
    }

    // return the average of the values currently in the buffer
    return sum / count;
}