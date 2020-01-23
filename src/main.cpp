#include <mbed.h>
#include <PID.h>
#include <DCMotor.h>

// DISTANCIALINEAL = 2 * PI * RADI / FORATS PER VOLTA
const double DISTANCIALINEAL = (2 * 3.1416 * 0.033) / 80;

//Inicialitzacio de variables
int i;
Serial teclat(USBTX, USBRX);
double kp=1.0;
double ki=5.0;
double kd=0.0;
double input=0.0;
double output=0.0;
double velocitatAAsolir=0.0;
double velocitatEscollida=0.0;

//Motor
Motor motor = Motor(0);

//Cada cop que el sensor detecta 
InterruptIn rsi(PG_0);

//Timer
int contInterrupcions=0;
float tempsIniciVolta=0.0, tempsFinalVolta=0.0;
Timer t;

// Inicialitzem el pid
PID pid = PID(&input, &output, &velocitatEscollida, kp, ki, kd, 1, 0);

void augmentarContador(){
  contInterrupcions++;
}

void llegirVelocitat(){
  teclat.printf("\nLlegint velocitat, entre 0 i 100");
  teclat.scanf("%lf", &velocitatEscollida);
  if (velocitatEscollida < 0) velocitatEscollida = 0;
  else if (velocitatEscollida > 100) velocitatEscollida = 100;

  teclat.printf("\n%f", velocitatEscollida);
  teclat.getc();
}

void calculVelocitat(){
  tempsFinalVolta = tempsIniciVolta;  
  tempsIniciVolta = t.read();
  input = ((contInterrupcions * DISTANCIALINEAL) / ((tempsIniciVolta-tempsFinalVolta)));   // velocitat lineal
  contInterrupcions = 0;
}

void establimModeIVelocitats() {
  pid.SetMode(1);
  pid.SetOutputLimits(0, 100);
  motor.Direction(0);
}

int main() {

  //Establim el mode de gir la direccio i la velocitat minima i maxima
  establimModeIVelocitats();

  //Engeguem les interrupcions amb la funció d'interrupcio
  rsi.rise(&augmentarContador);

  //Timer
  tempsIniciVolta=0;
  tempsFinalVolta=0;
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