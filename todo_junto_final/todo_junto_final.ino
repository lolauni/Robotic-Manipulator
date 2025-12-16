
#include <Servo.h> 

// ==========================================
// 1. POTENCIOMETER PARAMETERS
// ==========================================
const int LIMITE_MIN = 60;   
const int LIMITE_MAX = 960;  

// MOTOR 1 (Base)
float kp1 = 1.4; float kd1 = 0.8; int minPWM1 = 70; int maxPWM1 = 200;    

// --- MOTOR 2 (Shoulder) - 
// whith big kp2 and minPWM2 in order to have a more agressive reaction
float kp2 = 6.0;      
float kd2 = 1.5;      
int minPWM2 = 150;    
int maxPWM2 = 255;    

// MOTOR 3 (elbow)
float kp3 = 2.0; float kd3 = 0.9; int minPWM3 = 80; int maxPWM3 = 255;    

// ==========================================
// 2. PINES 
// ==========================================
const int PIN_BTN_SELECT = 8;   
const int PIN_BTN_IZQ    = A3;  
const int PIN_BTN_DER    = A5;  

int motorSeleccionado = 1; 
int lastSelectState = HIGH;
unsigned long lastSelectTime = 0;

// MOTOR 1
const int m1_Sensor = A2; const byte m1_PWM = 3; const byte m1_Dir1 = 2; const byte m1_Dir2 = 4;
int m1_Theta, m1_Theta_d; float m1_last_error = 0; unsigned long m1_t_prev = 0;

// MOTOR 2
const int m2_Sensor = A0; const byte m2_PWM = 5; const byte m2_Dir1 = 6; const byte m2_Dir2 = 7;
int m2_Theta, m2_Theta_d; float m2_last_error = 0; unsigned long m2_t_prev = 0;

// MOTOR 3
const int m3_Sensor = A1; const byte m3_PWM = 10; const byte m3_Dir1 = 12; const byte m3_Dir2 = 13;
int m3_Theta, m3_Theta_d; float m3_last_error = 0; unsigned long m3_t_prev = 0;

// GRIPPER
Servo miGripper; const int pinServo = 9; const int pinBotonServo = 11; 
int anguloCerrado = 70; int anguloAbierto = 20; bool estaAbierto = false; 
int ultimoEstadoBotonServo = HIGH; unsigned long ultimoReboteServo = 0; 

// ==========================================
// 3. SETUP
// ==========================================
int leerSensorSuavizado(int pin) {
  long suma = 0;
  for(int i=0; i<20; i++) { suma += analogRead(pin); delayMicroseconds(50); }
  return (int)(suma / 20);
}

void setup() {
  Serial.begin(9600);
  Serial.println("--- MODO MANUAL V5 (FUERZA M2) ---");

  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);
  pinMode(PIN_BTN_IZQ, INPUT_PULLUP);
  pinMode(PIN_BTN_DER, INPUT_PULLUP);

  miGripper.attach(pinServo); pinMode(pinBotonServo, INPUT_PULLUP); miGripper.write(anguloCerrado); 

  pinMode(m1_Dir1, OUTPUT); pinMode(m1_Dir2, OUTPUT); pinMode(m1_PWM, OUTPUT);
  pinMode(m2_Dir1, OUTPUT); pinMode(m2_Dir2, OUTPUT); pinMode(m2_PWM, OUTPUT);
  pinMode(m3_Dir1, OUTPUT); pinMode(m3_Dir2, OUTPUT); pinMode(m3_PWM, OUTPUT);
  
  delay(100);
  m1_Theta = leerSensorSuavizado(m1_Sensor); m1_Theta_d = m1_Theta;
  m2_Theta = leerSensorSuavizado(m2_Sensor); m2_Theta_d = m2_Theta;
  m3_Theta = leerSensorSuavizado(m3_Sensor); m3_Theta_d = m3_Theta;
}

// ==========================================
// 4. LOOP
// ==========================================
void loop() {
  
  // SELECTOR MODE
  int lecturaSelect = digitalRead(PIN_BTN_SELECT);
  if (lecturaSelect == LOW && lastSelectState == HIGH) {
    if (millis() - lastSelectTime > 250) { 
       motorSeleccionado++;
       if (motorSeleccionado > 3) motorSeleccionado = 1;
       Serial.print("MOTOR ACTIVO: "); Serial.println(motorSeleccionado);
       lastSelectTime = millis();
    }
  }
  lastSelectState = lecturaSelect;

  // BUTTONS
  bool btnIzq = (digitalRead(PIN_BTN_IZQ) == LOW);
  bool btnDer = (digitalRead(PIN_BTN_DER) == LOW);

  // CONTROL
  controlarMotorEstricto(m1_Sensor, (motorSeleccionado == 1), btnIzq, btnDer, 
                 m1_PWM, m1_Dir1, m1_Dir2, 
                 m1_Theta, m1_Theta_d, m1_last_error, m1_t_prev,
                 kp1, kd1, minPWM1, maxPWM1);

  controlarMotorEstricto(m2_Sensor, (motorSeleccionado == 2), btnIzq, btnDer, 
                 m2_PWM, m2_Dir1, m2_Dir2, 
                 m2_Theta, m2_Theta_d, m2_last_error, m2_t_prev,
                 kp2, kd2, minPWM2, maxPWM2);

  controlarMotorEstricto(m3_Sensor, (motorSeleccionado == 3), btnIzq, btnDer, 
                 m3_PWM, m3_Dir1, m3_Dir2, 
                 m3_Theta, m3_Theta_d, m3_last_error, m3_t_prev,
                 kp3, kd3, minPWM3, maxPWM3);

  // GRIPPER
  int lecturaServo = digitalRead(pinBotonServo);
  if (lecturaServo == LOW && ultimoEstadoBotonServo == HIGH) {
    if (millis() - ultimoReboteServo > 200) {
      estaAbierto = !estaAbierto; 
      miGripper.write(estaAbierto ? anguloAbierto : anguloCerrado);
      ultimoReboteServo = millis();
    }
  }
  ultimoEstadoBotonServo = lecturaServo;

  delay(5); 
}

// ==========================================
// 5. MOVEMENT FUNCTION
// ==========================================
void controlarMotorEstricto(int pinSensor, bool esElSeleccionado, bool pulsaIzq, bool pulsaDer,
                    byte pinPWM, byte pinDir1, byte pinDir2,
                    int &Theta_Actual, int &Theta_Target, 
                    float &LastError, unsigned long &TimePrev,
                    float k_p, float k_d, int min_pwm, int max_pwm) {
  
  Theta_Actual = leerSensorSuavizado(pinSensor);
  
  bool movimientoActivo = esElSeleccionado && (pulsaIzq || pulsaDer);

  if (movimientoActivo) {
    if (pulsaIzq) {
      if (Theta_Target < LIMITE_MAX) Theta_Target += 2; 
      else Theta_Target = LIMITE_MAX;
    }
    if (pulsaDer) {
      if (Theta_Target > LIMITE_MIN) Theta_Target -= 2; 
      else Theta_Target = LIMITE_MIN;
    }

    unsigned long t = millis();
    int dt = t - TimePrev;
    if (dt <= 0) dt = 1;

    float error = Theta_Target - Theta_Actual;
    float derivada = (error - LastError) / (float)dt;
    float salida = (k_p * error) + (k_d * derivada);
    
    int pwmSalida = min_pwm + abs((int)salida);
    if (pwmSalida > max_pwm) pwmSalida = max_pwm;
       
    if (salida > 0) {
       digitalWrite(pinDir1, HIGH); digitalWrite(pinDir2, LOW);
    } else {
       digitalWrite(pinDir1, LOW); digitalWrite(pinDir2, HIGH);
    }
    analogWrite(pinPWM, pwmSalida);

    LastError = error;
    TimePrev = t;

  } else {
    // STOP AND RESET
    analogWrite(pinPWM, 0); 
    Theta_Target = Theta_Actual; 
    LastError = 0;
  }
}