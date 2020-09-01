//double I,Iph,ID,Ish,Ios,Uoc,A,K,T,Rsh,G,ISCR,KI,Tr,EGO,B,Ior,q;
#include <FreeRTOS_ARM.h>
double Vg,Vc,n,Ns,T1,Voc_T1,Isc_T1,Isc_T2,T2,Voc_T2,Ics_T2,TaK,K0,IL_T1,IL,I0,I0_T1,Xv,dVdI_Voc,k,q,Rs,A,Vt_Ta,Ia;
double Suns,TaC,Va;

int Temp,Volt,Irra;

bool r=false;

void setup() {
  pinMode(52,OUTPUT);
  Va=0;
  analogWriteResolution(12);
  analogReadResolution(12);
  // put your setup code here, to run once:
  q=1.6*pow(10,-19);
  k=1.38*pow(10,-23);
//  ISCR=18.74;
//  Uoc=21.9;
//  KI=0.0017;
//  T=25;
//  Tr=301.18;
//  G=200;
//  Rsh=43993;
  A=1;
  
//  B=1;
//  Ior=2.9259*pow(10,-10);
//  EGO=1.11;
//  Ios=Ior*pow((T+273.15)/Tr,3)*exp((q*EGO*(1/Tr-1/(T+273.15)))/(B*K));
//  Iph=G*(ISCR+KI*(T-25))/1000;
  //Ios=8.74265*pow(10,-364);
//  ID=Ios*(exp(q*Uoc/(A*K*(T+273.15)))-1);
//  I=Iph-ID-Uoc/Rsh;

n=1.12;
Vg=1.12;
Ns=36;
T1=273+25;
Voc_T1=21.06/Ns;
Isc_T1=3.3;//3.8;
Voc_T2=19/Ns;//17.05/Ns;
Isc_T2=3.4;//3.92;
K0=(Isc_T2 - Isc_T1)/(T2 - T1);
I0_T1=Isc_T1/(exp(q*Voc_T1/(n*k*T1))-1);
Xv = I0_T1*q/(n*k*T1) * exp(q*Voc_T1/(n*k*T1));
dVdI_Voc = - 1.15/Ns / 2;
Rs = - dVdI_Voc - 1/Xv;
//Vt_Ta = A * k * TaK / q; 
Serial.begin(115200);
 
 xTaskCreate(panel, NULL,configMINIMAL_STACK_SIZE , NULL, 1, NULL);


 vTaskStartScheduler(); 


}

static void panel(void* arg){
portTickType xLastWakeTime;

xLastWakeTime = xTaskGetTickCount();
  
  while(1){
  r=true;
  digitalWrite(52,r);
//  Suns,TaC,Va
  //TaC=analogRead(A0)*100/4095;  //Temperatura en el panel [0 100]
  TaC=25;
  //Va=analogRead(A1)*18/4095;//17.5/4095;   //Voltaje en la carga [0 18] 
  Va=Va+0.1;
  if (Va>18.5)  Va=0;
  //Suns=analogRead(A2)/4095;  // Irradianza [0 1]
  Suns=1;
  Serial.println("Temperatura en el panel C");
  Serial.println(TaC);
  Serial.println("Voltaje en la carga");
  Serial.println(Va);
  Serial.println("Radiacion solar");
  Serial.println(Suns);
  

  
//  TaC=25;
//  Suns=1;
  //Va=analogRead(A0)*3.3/4095;
  
  TaK = 273 + TaC;
  IL_T1 = Isc_T1 * Suns; 
  IL = IL_T1 + K0*(TaK - T1); 
  I0= I0_T1*pow((TaK/T1),(3/n))*exp(-q*Vg/(n*k)*((1/TaK)-(1/T1)));
  Vt_Ta = A * k * TaK / q;
  Vc = Va/Ns;

  for (int j=1;j<=5;j++){
    Ia=Ia- (IL - Ia - I0*( exp((Vc+Ia*Rs)/Vt_Ta) -1))/(-1 - (I0*( exp((Vc+Ia*Rs)/Vt_Ta) -1))*Rs/Vt_Ta);
    }
    
   

   float I=Ia*4095/3.3;
   float V=Va*4095/17.5;
   
  
  Serial.println("Corriente");
  Serial.println(Ia);
    
if (Ia>0){
  analogWrite(DAC0,I);
  analogWrite(DAC1,V);
  }
  r=!r;
  digitalWrite(52,r);
  vTaskDelayUntil(&xLastWakeTime, (10/portTICK_RATE_MS));
  } 
  
 }

void loop() {
  
//  TaC=25;
//  Suns=1;
//  //Va=analogRead(A0)*3.3/4095;
//  
//  TaK = 273 + TaC;
//  IL_T1 = Isc_T1 * Suns; 
//  IL = IL_T1 + K0*(TaK - T1); 
//  I0= I0_T1*pow((TaK/T1),(3/n))*exp(-q*Vg/(n*k)*((1/TaK)-(1/T1)));
//  Vt_Ta = A * k * TaK / q;
//  Vc = Va/Ns;
//
//  for (int j=1;j<=5;j++){
//    Ia=Ia- (IL - Ia - I0*( exp((Vc+Ia*Rs)/Vt_Ta) -1))/(-1 - (I0*( exp((Vc+Ia*Rs)/Vt_Ta) -1))*Rs/Vt_Ta);
//    }
//   Va=Va+0.01;
//if (Ia>0){
//  analogWrite(DAC0,4095);
//  }
//
//if (Va>25){
//  Va=0;
//  }
//  Serial.println(Ia);
  // put your main code here, to run repeatedly:

}
