#include <Arduino.h>

#define SETPOINT_RPM(rpm) ((rpm) >= 40 ? 45.0 : 33.3)

// Configurações
#define SENSOR_PIN 34
#define DEBOUNCE_TIME 100000        // 100ms
#define RPM_RESET_TIMEOUT 3000000

#define IN3_PIN 25
#define IN4_PIN 26
#define ENB_PIN 27

// Variáveis voláteis
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool pulseDetected = false;

// Constantes do PID ajustadas
float Kp = 0.5;    // Aumentado para resposta mais rápida
float Ki = 0.1;     // Reduzido
float Kd = 0.05;    // Aumentado

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
  
  if (pulseDetected) {
    noInterrupts();
    unsigned long interval = pulseInterval;
    pulseDetected = false;
    interrupts();

    if (interval > 0) {
      rpm = (60000000.0 / interval); // Cálculo direto sem filtro
    }
  }

  if ((micros() - lastPulseTime) > RPM_RESET_TIMEOUT) {
    rpm = 0;
  }

  return rpm;
}

float computePID(float rpm, float setpoint, bool resetPID) {
  static float integral = 0;
  static float lastError = 0;
  const float dt = PIDInterval / 1000.0; // 0.01 segundos
  
  if (resetPID) {
    integral = 0;
    lastError = 0;
    return 150;
  }

  float error = setpoint - rpm;
  
  // Termo Proporcional
  float P = Kp * error;
  
  // Termo Integral com clamping
  integral += error * dt;
  if(error > 10) integral = constrain(integral, -1000, 1000);
  
  // Termo Derivativo com filtro
  float D = Kd * ((error - lastError) / dt);
  D = 0.6 * D + 0.4 * lastError; // Filtro suave
  
  lastError = error;
  
  float output = 150 + P + (Ki * integral) + D;
  return constrain(output, 130, 170);
}

void activateMotor(float pidOutput) {
  static float output = 150;
  const float smoothingFactor = 0.3; // Suavização mínima
  
  output += smoothingFactor * (pidOutput - output);
  output = constrain(output, 130, 170);
  
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  ledcWrite(0, (int)output);
}

void printDebug(float rpm, float setpoint, float output) {
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint > 100) {
    Serial.print("TEMP: ");
    Serial.print(millis());
    Serial.print(" RPM:");
    Serial.print(rpm, 4);
    Serial.print(" SET:");
    Serial.print(setpoint);
    Serial.print(" OUT:");
    Serial.println(output);
    lastPrint = millis();
  }
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
  ledcWrite(0, 150);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);
}

void loop() {
  static float setpoint = 0;
  static bool pidActive = false;
  
  // Controle de timing preciso
  if(micros() - lastPIDTime >= PIDInterval * 1000) {
    float rpm = calculateRPM();
    
    // Lógica de ativação
    if(rpm == 0) {
      pidActive = false;
      ledcWrite(0, 130);
    } else if(rpm > 25 && !pidActive) {
      pidActive = true;
    }
    
    setpoint = SETPOINT_RPM(rpm);
    
    if(pidActive) {
      float output = computePID(rpm, setpoint, !pidActive);
      activateMotor(output);
      printDebug(rpm, setpoint, output);
    }
    
    lastPIDTime = micros();
  }
}