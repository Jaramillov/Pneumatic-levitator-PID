#include "definitions.h"
#include <Wire.h>


// tiempo de muestreo dejar en 25ms no cambiar
float h = 0.025;


// offset de calibracion para detectar la distancia en positivo con 0 en el 
//piso del tubo
float calibration = 60;


//limites del actuador
float umax = 12;
float umin = 0;


// variables de identificación con histeresis
float reference = 30; //poner en la mitad del tubo
float y; // distancia del objeto
float u; //accion de control


// valores del lazo de histeresis
float ueq = 2.95; // control para el equilibrio
float eh = 12;  // valor en centimetros del error de histeresis
float ud = 0.4;  // valor en voltios del cambio en la señal de control

// valores de encendido y apagado de la señal de control
float uon = ueq + ud;
float uoff = ueq - ud; 




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
*               CONTROL CON HISTERESIS PARA IDENTIFICACION
***************************************************************************/ 


static void controlTask(void *pvParameters) {

    // Aqui configuro cada cuanto se repite la tarea
    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos = 1000*0.025= 25ms
    uint32_t n = 0;
    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount();

       // distancia en centimetros       
       y = calibration - (float) sensor.readRangeContinuousMillimeters()/10 ; 
       
      // control con histeresis

      if (y <  reference - eh) {
          u = uon; 
      } else if ((y <  reference + eh) & (u==uon)) {
          u = uon; 
      } else if ((y >  reference- eh) & (u==uoff)) {
          u = uoff;
      }else if (y >  reference + eh){
          u = uoff;
      }

       voltsToFan(3.4);
      
      // imprimimos tambien el tiempo
      float time = n*h;
      
      // vamos a imprimir valores del experimento hasta 120 segundos para salvarlo.
      if (time <= 120){
         printf("%0.3f, %0.3f, %0.2f\n", time, u, y);
         n+=1;
       }
       
       // la tarea es crítica entonces esperamos exactamente taskInterval ms antes de activarla nuevamente
       vTaskDelayUntil(&xLastWakeTime, taskInterval);     

    }

}



void loop() {
    vTaskDelete(NULL);
}
