#include "definitions.h"
#include <Wire.h>


// tiempo de muestreo dejar en 25ms
float h = 0.025;



// offset de calibracion para detectar la distancia en positivo con 0 en el 
//piso del tubo
float calibration = 40;


//limites del actuador
float umax = 12;
float umin = 0;

// variables del lazo de control
float reference = 20;
float y;
float u;
float usat;
float e;
float kp = 0.1; 

// valor de equilibrio de la señal de control
float ueq = 5;




void setup() {
    // iniciamos el sensor  
    Wire.begin();
    while(!setupSensor()){
      vTaskDelay(1000);
    }
    setupPwm();    
    vTaskDelay(100);
    
    // Asi definimos la tarea de control, de la máxima prioridad en el nucleo 1
    xTaskCreatePinnedToCore(
            controlTask, // nombre de la rutina
            "general controller task",
            8192,
            NULL,
            23, // prioridad de la tarea (0-24) , siendo 24 la prioridad más critica      
            NULL,
            CORE_1
    );  
    

}


/***************************************************************************
*                CONTROL PROPORCIONAL DEL LEVITADOR
***************************************************************************/ 


static void controlTask(void *pvParameters) {

    // Aqui configuro cada cuanto se repite la tarea
    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos = 1000*0.025= 25ms
    
    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount();

       // distancia en centimetros       
       y = calibration - (float) sensor.readRangeContinuousMillimeters()/10 ; 
       
       // control proporcional
       e = reference - y;
       u = kp*e + ueq;
       
       // control proporcional
       usat = constrain(u, umin, umax);
       voltsToFan(usat);
       printf("%0.2f, %0.2f, %0.3f\n", reference, y, movingAverage(usat));
       
       // la tarea es crítica entonces esperamos exactamente taskInterval ms antes de activarla nuevamente
       vTaskDelayUntil(&xLastWakeTime, taskInterval);     

    }

}





void loop() {
    vTaskDelete(NULL);
}