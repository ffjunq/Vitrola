#include <Arduino.h>

#define SETPOINT_RPM(rpm) ((rpm) >= 38 ? 45.0 : 33.3)
#define HYSTERESIS 2
#define ERROR_TOLERANCE 0.3
#define PWM_MOTOR_INICIAL 130//95
// Configurações
#define SENSOR_PIN 34
#define DEBOUNCE_TIME 100000        // 200ms
#define RPM_RESET_TIMEOUT 3000
#define PULSES_PER_REVOLUTION 4
#define MIN_INTERVAL 200000  // 200ms (equivale a 75 RPM)
#define MAX_INTERVAL 600000  // 600ms (equivale a 25 RPM)

#define IN3_PIN 25
#define IN4_PIN 26
#define ENB_PIN 27

// Variáveis voláteis
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool pulseDetected = false;
volatile bool newPulse = false; // Declare no início do código (junto com pulseDetected)

// Constantes PID
float Kp = 1.8;    
float Ki = 0.25;    
float Kd = 0.8;    

// Variáveis de tempo
unsigned long lastPIDTime = 0;

void IRAM_ATTR countPulse() {
  unsigned long currentTime = micros();
  if ((currentTime - lastPulseTime) >= DEBOUNCE_TIME) {
    pulseInterval = currentTime - lastPulseTime;
    // Verifica se o intervalo é plausível
    if (pulseInterval < MIN_INTERVAL || pulseInterval > MAX_INTERVAL) 
    {
        lastPulseTime = currentTime;
        return; // Mantém o último valor válido
    }
    pulseDetected = true;
    newPulse = true;
    lastPulseTime = currentTime;
  }
}

float calculateRPM() {
    static float rpm = 0;
    static float rpmBuffer[3] = {0}; // Buffer de 10 amostras
    static int index = 0;
    static long lastUpdateTime = 0;

    lastUpdateTime = millis();
    // Aguarda o primeiro pulso válido
    while(!pulseDetected)
    {
      if(millis() - lastUpdateTime > RPM_RESET_TIMEOUT) // Se passaram 1 segundo sem novo pulso
      {
        return 0;
      }
    }

    // Atualiza RPM usando buffer circular e média móvel
    if (pulseDetected) 
    {
        noInterrupts();
        unsigned long interval = pulseInterval;
        pulseDetected = false;
        interrupts();

        if (interval > 100) { // Filtro de intervalos inválidos
            float newRPM = 60000000.0 / (interval * PULSES_PER_REVOLUTION);
            rpmBuffer[index] = newRPM; // Atualiza buffer
            index = (index + 1) % 10;

            // Média móvel
            float sum = 0;
            for (int i = 0; i < 3; i++) sum += rpmBuffer[i];
            float avgRPM = sum / 3;

            // Filtro exponencial para suavização adicional
            rpm = 0.8 * avgRPM + 0.2 * rpm;
        }
    }

    // Reset se não houver pulsos
    //if ((micros() - lastPulseTime) > RPM_RESET_TIMEOUT) rpm = 0;

    return rpm;
}

float computePID(float rpm, float setpoint, bool resetPID) {
  static float integral = 0;

  // Usa o intervalo do pulso atual para dt
  const float dt = fmaxf(pulseInterval / 1000000.0f, 0.001f);

  static float lastError = 0;
  float error = setpoint - rpm;
  
  if (resetPID) { // Reinicia integral apenas em grandes desvios
    integral = 0;
    return PWM_MOTOR_INICIAL;
  }

  if (abs(error) < ERROR_TOLERANCE) {
    integral = 0; // Anti-windup em regime permanente
  }
  
  // Termo Proporcional
  float P = Kp * error;
  
  // Termo Integral com clamping dinâmico
  integral += Ki * error * dt;
  integral = constrain(integral, -3, 3); // Limites mais amplos

  // Termo Derivativo baseado na variação do RPM
  static float lastRpm = 0;
  float D = -Kd * (rpm - lastRpm) / dt; // Derivativo baseado na variação do RPM
  lastRpm = rpm;
  static float filteredD = 0; // Variável estática para manter o estado
  filteredD = 0.85 * filteredD + 0.15 * D;  
  lastError = error;
  
  float output = PWM_MOTOR_INICIAL + P + integral + filteredD;
  output = constrain(output, 0, 255);

  Serial.print("P:");
  Serial.print(P);
  Serial.print(" I:");
  Serial.print(integral);
  Serial.print(" D:");
  Serial.print(D);
  Serial.print(" Error:");
  Serial.print(error, 4);
  
  return output;
}

float activateMotor(float pidOutput) 
{
  static float output = PWM_MOTOR_INICIAL;
  const float smoothingFactor = 0.3; // Resposta mais rápida
  
  output += smoothingFactor * (pidOutput - output);
  output = constrain(output, 0, 255);
  
  // Converter para inteiro com arredondamento correto
  output = lround(output); // Usa arredondamento matemático

  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  ledcWrite(0, output);

  return output;
}

void printDebug(float rpm, float setpoint, float output) {
  static unsigned long lastPrint = 0;
    Serial.print(" TEMP:");
    Serial.print(millis());
    Serial.print(" RPM:");
    Serial.print(rpm, 4);
    Serial.print(" SET:");
    Serial.print(setpoint);
    Serial.print(" OUT:");
    Serial.println(output);
    lastPrint = millis();
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
  ledcWrite(0, PWM_MOTOR_INICIAL);

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
    static float smoothingOut = 0;
    //if(rpmError >= ERROR_TOLERANCE) 
    if(newPulse)
    {
      float output = computePID(rpm, setpoint, !pidActive);
      smoothingOut = activateMotor(output);
      printDebug(rpm, setpoint, smoothingOut);
      newPulse = false;
    }
  }
  else
  {
    Serial.print("PID OFF!");
  }
}