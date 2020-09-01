
#include <FreeRTOS_ARM.h>

double TenK, VelK;
double VelK1, TNK, TNK1;
double PLK, PLK1,U[51];
double KGen = 1/2;
int cuenta=0;
bool r=false;

void setup() {

  pinMode(52,OUTPUT);
  analogWriteResolution(12);
  analogReadResolution(12);
  
  Serial.begin(115200);

  PLK=0.1;
 
  xTaskCreate(motor, NULL,configMINIMAL_STACK_SIZE , NULL, 1, NULL);

  vTaskStartScheduler(); 

}

static void motor(void* arg){
portTickType xLastWakeTime;

xLastWakeTime = xTaskGetTickCount();
  
  while(1){
  
  r=true;
  digitalWrite(52,r);

  for (int j=50;j>0;j--)  U[j] = U[j-1];
  TNK1=TNK;
  VelK1 = VelK;
  
  U[0]=analogRead(A0)*1/4095; // Acción al actuador
  U[0]=1;
  //PLK=analogRead(A1)*1/4095;  // Par de carga
  PLK1=PLK;
  if (cuenta==30000){
    PLK=1;
    }
  cuenta+=1;
  
  
  //PLK=0;
    
  //Serial.print("Acción al Actuador = ");
  //Serial.println(U[0]);
  //Serial.print("Par de carga = ");
  //Serial.println(PLK);
  
  TNK = (0.9231*TNK1) + (0.04423*(U[49]+U[50])) - PLK+0.9231*PLK1;

  VelK = (0.9997*VelK1) + (0.00015+(TNK+TNK1));

  TenK = VelK * KGen;

  float Vel=VelK*4095/3.3;
  float Ten=TenK*4095/3.3;
  
  analogWrite(DAC0,Vel);
  analogWrite(DAC1,Ten);
  

//  Serial.println("Velocidad = ");
  Serial.println(VelK);

//  Serial.println("Tensión = ");
  //Serial.println(Ten);
    
  r=!r;
  digitalWrite(52,r);
  vTaskDelayUntil(&xLastWakeTime, (10/portTICK_RATE_MS));
  } 
  
 }

void loop() {}
