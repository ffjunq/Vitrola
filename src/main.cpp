#include <Arduino.h>

#define SETPOINT_RPM(rpm) ((rpm) >= 40 ? 45.0 : 33.3)
#define HYSTERESIS 1.0
#define ERROR_TOLERANCE 0.3
#define PWM_MOTOR_INICIAL 115
// Configurações
#define SENSOR_PIN 34
#define DEBOUNCE_TIME 100000        // 100ms
#define RPM_RESET_TIMEOUT 3000000
#define PULSES_PER_REVOLUTION 4

#define IN3_PIN 25
#define IN4_PIN 26
#define ENB_PIN 27

// Variáveis voláteis
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool pulseDetected = false;

// Constantes PID
float Kp = 4.0;    
float Ki = 0.05;    
float Kd = 0.2;    

// Variáveis de tempo
unsigned long lastPIDTime = 0;
const unsigned long PIDInterval = 10; // 10ms para loop de controle

void IRAM_ATTR countPulse() {
  unsigned long currentTime = micros();
  if ((currentTime - lastPulseTime) >= DEBOUNCE_TIME) {
    pulseInterval = currentTime - lastPulseTime;
    pulseDetected = true;
    lastPulseTime = currentTime;
  }
}

float calculateRPM() {
  static float rpm = 0;
  static float rpmBuffer[5] = {0}; // Buffer para média móvel
  static int index = 0;

  if (pulseDetected) {
    noInterrupts();
    unsigned long interval = pulseInterval;
    pulseDetected = false;
    interrupts();

    if (interval > 0) {
      float newRPM = (60000000.0 / (interval * PULSES_PER_REVOLUTION));
      rpmBuffer[index] = newRPM;
      index = (index + 1) % 5; // Atualiza buffer
      float sum = 0;
      for (int i = 0; i < 5; i++) sum += rpmBuffer[i];
      rpm = sum / 5; // Média dos últimos 5 valores
    }
  }

  if ((micros() - lastPulseTime) > RPM_RESET_TIMEOUT) {
    rpm = 0;
  }

  return rpm;
}

float computePID(float rpm, float setpoint, bool resetPID) {
  static float integral = 0;
  static float lastRPM = 0;
  const float dt = max(pulseInterval / 1000000.0, 0.001); // Valor mínimo de 1 ms
  float error = setpoint - rpm;
  
  if (resetPID || abs(error) > 5.0) { // Reinicia integral apenas em grandes desvios
    integral = 0;
    lastRPM = rpm;
    return PWM_MOTOR_INICIAL;
  }
  
  // Termo Proporcional
  float P = Kp * error;
  
  // Termo Integral com clamping dinâmico
  integral += Ki * error * dt;
  integral = constrain(integral, -5, 5); // Limites mais conservadores
  
  // Termo Derivativo baseado na variação do RPM
  float D = -Kd * (rpm - lastRPM) / dt; // Sinal negativo para ação correta
  float filteredD = 0.8 * filteredD + 0.2 * D; // Filtro exponencial
  lastRPM = rpm;
  
  float output = PWM_MOTOR_INICIAL + P + integral + filteredD;
  output = constrain(output, 0, 255);

  Serial.print("P:");
  Serial.print(P);
  Serial.print(" I:");
  Serial.print(integral);
  Serial.print(" D:");
  Serial.print(D);
  
  return output;
}

float activateMotor(float pidOutput) 
{
  static float output = PWM_MOTOR_INICIAL;
  const float smoothingFactor = 0.01; // Resposta mais rápida
  
  output += smoothingFactor * (pidOutput - output);
  output = constrain(output, 0, 255);
  
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  ledcWrite(0, (int)output);

  return output;
}

void printDebug(float rpm, float setpoint, float output) {
  static unsigned long lastPrint = 0;
//  if(millis() - lastPrint > 100) {
    Serial.print(" TEMP:");
    Serial.print(millis());
    Serial.print(" RPM:");
    Serial.print(rpm, 4);
    Serial.print(" SET:");
    Serial.print(setpoint);
    Serial.print(" OUT:");
    Serial.println(output);
    lastPrint = millis();
//   }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(SENSOR_PIN, INPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  ledcAttachPin(ENB_PIN, 0);
  ledcSetup(0, 5000, 8);
  
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);
}

void loop() 
{
  static float setpoint = 0;
  static bool pidActive = false;
  
  float rpm = calculateRPM();
  
  // Lógica de ativação
  if(rpm == 0) {
    pidActive = false;
    ledcWrite(0, PWM_MOTOR_INICIAL);
  } else if(rpm > 25 && !pidActive) {
    pidActive = true;
  }

  if(rpm > (SETPOINT_RPM(rpm) + HYSTERESIS)) 
  {
    setpoint = SETPOINT_RPM(rpm) - HYSTERESIS/2;
  } 
  else if(rpm < (SETPOINT_RPM(rpm) - HYSTERESIS)) 
  {
    setpoint = SETPOINT_RPM(rpm) + HYSTERESIS/2;
  } 
  else 
  {
    setpoint = SETPOINT_RPM(rpm);
  }
  
  if(pidActive) 
  {
    static float lastRpm = 0;
    static float smoothingOut = 0;
    float rpmError = abs(rpm - setpoint);
    //if(rpmError >= ERROR_TOLERANCE) 
    if(lastRpm != rpm)
    {
      float output = computePID(rpm, setpoint, !pidActive);
      smoothingOut = activateMotor(output);
      printDebug(rpm, setpoint, smoothingOut);
      lastRpm = rpm;
    }
  }
}