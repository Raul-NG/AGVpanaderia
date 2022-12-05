#define frec 1000.0 // Hz
#define dt 0.001 // 1/frec
#define diamRueda 0.0325 //m
#define ejeLlantas 0.11 //m
#define saturacion 1500.0
#define rpmMax 90.0
#define pulsos 680.0 // 715.0 pulsos/rev

const int pinA[] = {4, 2};
const int pinB[] = {7, 8};
const int pinMotorDir1[] = {11, 13};
const int pinMotorDir2[] = {10, 12};
const int pinPWM[] = {6, 3};

volatile int dir[] = {0, 0};
volatile int counter[] = {0, 0};
volatile float vel_act[] = {0.0, 0.0};

int vel_ref[] = {0.0, 0.0}; // Cambiar a float
float vel_act_prom[] = {0.0, 0.0};
float vel_ctrl[] = {0.0, 0.0};
float error[] = {0.0, 0.0};
float error1[] = {0.0, 0.0};
float error2[] = {0.0, 0.0};

const float kp[] = {450.0, 450.0}; //15
const float ki[] = {20.0, 20.0}; //2
const float kd[] = {0.005, 0.005}; //1.5

float lects[2][10] = {0};
int contLect[] = {0, 0};
float sumaLects[] = {0.0, 0.0};

//float cero = 40.0;
//float kd = 0.017;
//float kp = 2*cero*kd;
//float ki = cero*cero*kd;
float intError[] = {0.0, 0.0};
float diffError[] = {0.0, 0.0};
//float lecturaActual = 0.0;
//float lectFiltradaAnt = 0.0;
//float a1 = 0.8;
//float b0 = 0.2;

unsigned long millis_ant = 0;
unsigned long millis_vel[] = {0, 0};
unsigned long t_ant[] = {0, 0};
void encoderInt();
void encoderInt2();

void setup() {
  for (int motor = 0; motor < 2; motor++) {
    pinMode(pinA[motor], INPUT);
    pinMode(pinB[motor], INPUT);
    pinMode(pinMotorDir1[motor], OUTPUT);
    pinMode(pinMotorDir2[motor], OUTPUT);
    pinMode(pinPWM[motor], OUTPUT);
    digitalWrite(pinMotorDir1[motor], 0);
    digitalWrite(pinMotorDir2[motor], 1);
    digitalWrite(pinPWM[motor], 0);
  }
  Serial.begin(115200);
  Serial1.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pinB[0]), encoderInt, RISING);
  attachInterrupt(digitalPinToInterrupt(pinA[1]), encoderInt2, RISING);
}

void loop() {
  if (Serial1.available()) { // Serial1.available()
    char keys = Serial1.read(); //Serial1.read()
//    if (keys >= 48 && keys <= 58){ //
//    keys = keys - 48; //
    int velLineal = (keys & B0001) - ((keys>>2) & B0001); //w - s
    int velAngular = ((keys>>1) & B0001) - ((keys>>3) & B0001); //a - d
    vel_ref[0] = (velLineal + velAngular*0.5)*rpmMax;
    vel_ref[1] = (velLineal - velAngular*0.5)*rpmMax;
//    Serial.print("keys: "+ String(keys));
//    Serial.print(" vL: " + String(velLineal));
//    Serial.print(" vA: " + String(velAngular));
//    Serial.print(" v0: " + String(vel_ref[0]));
//    Serial.println(" v1: " + String(vel_ref[1]));
    for (int motor = 0; motor < 2; motor++){
      if (vel_ref[motor] >= 0){
        vel_ref[motor] = vel_ref[motor] > rpmMax ? rpmMax : vel_ref[motor];
        digitalWrite(pinMotorDir1[motor], 0);
        digitalWrite(pinMotorDir2[motor], 1);
        
      }else if (vel_ref[motor] < 0){
        Serial.print("vel_menor");
        vel_ref[motor] = vel_ref[motor] < -rpmMax ?  rpmMax : -vel_ref[motor];
        digitalWrite(pinMotorDir1[motor], 1);
        digitalWrite(pinMotorDir2[motor], 0);
      }
    }
//    }//
  }

  unsigned long temp = millis();
  for (int motor = 0; motor < 2; motor++) {
    if (temp - millis_vel[motor] >= 500) {
      vel_act[motor] = 0;
    }
  }
  if (temp - millis_ant >= dt * 1000) {
    millis_ant = temp;
    for (int motor = 0; motor < 2; motor++) {
      sumaLects[motor] = sumaLects[motor] - lects[motor][contLect[motor]];
      lects[motor][contLect[motor]] = vel_act[motor]; //counter*frec/pulsos * 60; // [rev/min] dir*
      sumaLects[motor] = sumaLects[motor] + lects[motor][contLect[motor]++];
      contLect[motor] = contLect[motor] % 10;
      vel_act_prom[motor] = sumaLects[motor] / 10.0;
      error[motor] = vel_ref[motor] - vel_act_prom[motor];
      intError[motor] = intError[motor] + error[motor] * dt;
      diffError[motor] = (error[motor] - error1[motor]) * frec;
      vel_ctrl[motor] = kp[motor] * error[motor] + ki[motor] * intError[motor] + kd[motor] * diffError[motor];
      error2[motor] = error1[motor];
      error1[motor] = error[motor];
      vel_ctrl[motor] = (vel_ctrl[motor] >= 0.0 && vel_ctrl[motor] <= saturacion) ? vel_ctrl[motor] : (vel_ctrl[motor] > 0.0 ? saturacion : 0.0);
      analogWrite(pinPWM[motor], (int)(vel_ctrl[motor] * 255.0 / saturacion)); //
    }
    Serial.print("ref0: ");
    Serial.print(vel_ref[0]);
    Serial.print(" rpm0: ");
    Serial.print(vel_act[0]);
    Serial.print(" pwm0: ");
    Serial.print((vel_ctrl[0] / saturacion) * 50.0);
    Serial.print(" ref1: ");
    Serial.print(vel_ref[1]);
    Serial.print(" rpm1: ");
    Serial.print(vel_act[1]);
    Serial.print(" pwm1: ");
    Serial.println((vel_ctrl[1] / saturacion) * 50.0);
  }
}

void encoderInt() {
  millis_vel[0] = millis();
  unsigned long t = micros();
  vel_act[0] = 60000000.0 / ((float)(t - t_ant[0]) * pulsos);
  t_ant[0] = t;
}

void encoderInt2() {
  millis_vel[1] = millis();
  unsigned long t = micros();
  vel_act[1] = 60000000.0 / ((float)(t - t_ant[1]) * pulsos);
  t_ant[1] = t;
}
