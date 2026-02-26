
#include <VL53L0X.h>


// definición de los nucleos en el esp32
#define CORE_0 0   // nucleo base usado por algunas tareas procesador
#define CORE_1 1   // nucleo para el usuario, poner aqui las tareas críticas

// Esta es la configuración del PWM
#define  FREQUENCY_PWM     25000 // frecuencia del pwa
#define  RESOLUTION_PWM    11  // bits del pwm
#define  PIN_PWM          2   // pin del pwm 1
 


float  voltsToPwm =   (pow(2, RESOLUTION_PWM) - 1) / 12; 


//sensor laser
//conectar GPIO 22 SCL
//conectar GPIO 21 a SDA
VL53L0X sensor;


bool setupSensor(void){
    // we set the laser sensor
    sensor.setTimeout(100);  
    if(!sensor.init()){
        printf("No se detecto sensor\n");
        return false;
        }
    sensor.startContinuous(); 
    sensor.setMeasurementTimingBudget(20000);
    return true;
}

void setupPwm(void){
    ledcAttach(PIN_PWM,  FREQUENCY_PWM , RESOLUTION_PWM);
    // use las siguientes en PLATTFORMIO en vez de la anterior
    //ledcSetup(0,  FREQUENCY_PWM , RESOLUTION_PWM);
    //ledcAttachPin(PWMPIN, 0);
}

void voltsToFan(float volts){
    uint16_t pwm = volts * voltsToPwm;
    ledcWrite(PIN_PWM , pwm); 
}


float movingAverage(float newValue) {
    const int filterSize = 20;
    static float filter_values[filterSize] = {0.0};  // Array to store the last 'FILTER_SIZE' values
    static int index = 0;  // Current index in the array
    static float sum = 0.0;  // Sum of the current values in the array
    static int count = 0;  // Number of values added to the filter

    // Subtract the oldest value from sum and replace it with the new value
    sum = sum - filter_values[index] + newValue;
    filter_values[index] = newValue;

    // Update the index, and wrap around if necessary
    index = (index + 1) % filterSize ;

    // Keep track of how many values have been added to the filter
    if (count < filterSize ) {
        count++;
    }
    // Return the average of the values in the filter
    return sum / count;
}
