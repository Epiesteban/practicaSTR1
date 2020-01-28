#include <mbed.h>
#include <PID.h>
#include <DCMotor.h>

// DISTANCIALINEAL = 2 * PI * RADI(DIÀMETRE/2) / FORATS PER VOLTA
const double DISTANCIALINEAL = (2 * 3.1416 * 0.033) / 80;

//Inicialitzacio de variables
Serial teclat(USBTX, USBRX);
double kp=1.0;
double ki=5.0;
double kd=0.0;
double input=0.0;
double output=0.0;
double setPoint=0.0;

//Inicialitzem el motor
Motor motor = Motor(0);

//Cada cop que el sensor detecta un forat de lencoder
InterruptIn rsi(PG_0);

//Inicialitzem el timer i les variables dinterrupcio
int contInterrupcions=0;
float tempsInici=0.0, tempsFinal=0.0;
Timer t;

//Inicialitzem el pid
PID pid = PID(&input, &output, &setPoint, kp, ki, kd, 1, 0);

//Cada vegada que salta la rsi augmentem el contador en una unitat
void augmentarContador(){
  contInterrupcions++;
}

//Llegim la velocitat escrita per lusuari, aquesta ha destar entre 0 i 100 unitats
void llegirVelocitat(){
  teclat.printf("\nLlegint velocitat, entre 0 i 100");
  teclat.scanf("%lf", &setPoint);
  if (setPoint < 0) setPoint = 0;
  else if (setPoint > 100) setPoint = 100;

  teclat.printf("\n%f", setPoint);
  teclat.getc();
}

//Per calcular la velocitat dividirem el contador dinterrupcions i la distancia lineal que es recorreguda per interrupcio per el temps transcorregut
void calculVelocitat(){
  tempsFinal = tempsInici;  
  tempsInici = t.read();

  // velocitat lineal
  input = ((contInterrupcions * DISTANCIALINEAL) / ((tempsInici-tempsFinal)));
  contInterrupcions = 0;
}

 //Establim el mode(!=0 vol dir mode automàtic), l'escala de valors que pot retornar l'output i la direcció del motor (Endavant/endarrere)
void establimModeIVelocitats() {
  pid.SetMode(1);
  pid.SetOutputLimits(0, 100);
  motor.Direction(0);
}

int main() {

  establimModeIVelocitats();

  //Engeguem les interrupcions amb la funció d'interrupcio
  rsi.rise(&augmentarContador);

  //Engeguem timer
  tempsInici=0;
  tempsFinal=0;
  t.start();

  while(1) {
    //Detectem si escrivim per teclat per detectar la velocitat a la que volem anar
    if(teclat.readable()){
      llegirVelocitat();
    }

    wait(1);

    //Fem el calcul de velocitat instantania
    calculVelocitat();
  
    //Fem compute per aplicar la nova velocitat que ens retorna el pid
    if(pid.Compute()){
      teclat.printf("\nInput: %f Output: %f", input, output);
      motor.setSpeed(output, 0, 0, 0);
    } 
  }
}