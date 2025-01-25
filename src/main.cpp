#include <Arduino.h>

#define SETPOINT_RPM(rpm) ((rpm) >= 40 ? 45.0 : 33.3)

// Configurações
#define SENSOR_PIN 34               // Pino digital onde o TCRT5000 está conectado
#define DEBOUNCE_TIME 500000        // Tempo de debounce em microssegundos (500ms)
#define RPM_RESET_TIMEOUT 3000000   // Tempo limite para zerar o RPM (3 segundos em microssegundos)

#define IN3_PIN 25 // Pino digital do ESP32 para IN3
#define IN4_PIN 26 // Pino digital do ESP32 para IN4
#define ENB_PIN 27 // Pino PWM do ESP32 para ENB

// Variáveis voláteis para uso na interrupção
volatile unsigned long lastPulseTime = 0; // Tempo do último pulso válido
volatile unsigned long pulseInterval = 0; // Intervalo entre os últimos dois pulsos
volatile bool pulseDetected = false;      // Flag para indicar um novo pulso detectado

// Constantes do PID
float Kp = 1.0; // Proporcional
float Ki = 0.01; // Integral
float Kd = 0.05;  // Derivativo

// Função de interrupção
void IRAM_ATTR countPulse() 
{
  unsigned long currentTime = micros(); // Tempo atual em microssegundos

  // Verifica se o intervalo desde o último pulso é maior que o tempo de debounce
  if ((currentTime - lastPulseTime) >= DEBOUNCE_TIME) 
  {
    pulseInterval = currentTime - lastPulseTime; // Calcula o intervalo entre pulsos
    pulseDetected = true;                        // Marca que um novo pulso foi detectado
    lastPulseTime = currentTime;                 // Atualiza o tempo do último pulso
  }
}

// Função para calcular o RPM
float calculateRPM() 
{
  static float rpm = 0; // Valor atual de RPM

  // Calcula o RPM quando um novo pulso é detectado
  if (pulseDetected) 
  {
    noInterrupts(); // Desativa as interrupções para acessar variáveis voláteis com segurança
    unsigned long interval = pulseInterval;
    pulseDetected = false; // Reseta a flag após leitura
    interrupts(); // Reativa as interrupções

    // Evita divisões por zero
    if (interval > 0) 
    {
      rpm = (1.0 / (interval / 1000000.0)) * 60.0; // Calcula o RPM
    }
  }

  // Zera o RPM se o último pulso foi há mais de 3 segundos
  if ((micros() - lastPulseTime) > RPM_RESET_TIMEOUT) 
  {
    rpm = 0;
  }

  return rpm;
}

// Função para controlar o PID
// Função para controlar o PID
float computePID(float rpm, float setpoint, bool resetPID) 
{
  static float error = 0;             // Erro atual
  static float lastError = 0;         // Erro na última iteração
  static float integral = 0;          // Acumulador do erro
  static float derivative = 0;        // Taxa de variação do erro
  static float output = 0;            // Saída do controlador PID

  // Se o PID precisar ser resetado (RPM == 0), zerar as variáveis
  if (resetPID) {
    error = 0;
    lastError = 0;
    integral = 0;
    derivative = 0;
    output = 0;
  }

  // Controle PID
  error = setpoint - rpm;
  integral += error;
  derivative = error - lastError;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  return output;
}

// Função para imprimir os dados no monitor serial
void printData(float rpm, float setpoint, float smoothedOutput) 
{
  Serial.print("RPM: ");
  Serial.print(rpm, 2);
  Serial.print(" | Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" | Output: ");
  Serial.println(smoothedOutput);
}

// Função para ativar o motor com a saída suavizada
float activateMotor(float rpm, float pidOutput) 
{
  // Variáveis para suavização do output
  float smoothingFactor = 0.005; // Fator de suavização (quanto menor, mais suave)
  static float smoothedOutput = 0;

  // Condição para usar smoothedOutput apenas entre RPMs específicos
  if ((rpm >= 32 && rpm <= 34) || (rpm >= 44 && rpm <= 46)) 
  {
    // Suavização da transição do output
    smoothingFactor = 0.05;
  } else 
  {
    // Se não estiver nos intervalos, usa a saída bruta
    smoothingFactor = 0.5;
  }

  smoothedOutput = smoothedOutput + smoothingFactor * (pidOutput - smoothedOutput);

  // Limita a saída entre 150 e 180 (valores válidos para PWM)
  smoothedOutput = constrain(smoothedOutput, 150, 170);

  // Ajusta a direção e velocidade do motor com a saída suavizada
  if (smoothedOutput > 0) 
  {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    ledcWrite(0, smoothedOutput); // Define a velocidade suavizada
  }
  
  return smoothedOutput;
}

void setup() 
{
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  // Configura PWM no ENB
  ledcAttachPin(ENB_PIN, 0);
  ledcSetup(0, 5000, 8); // Frequência de 5kHz, resolução de 8 bits

  // Inicializa motor 150 de PWM
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  ledcWrite(0, 150); // Define a velocidade suavizada

  // Configura a interrupção para disparar na borda de descida (fita branca detectada)
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);
}

void loop() 
{
  static float setpoint = 0;          // Valor desejado de RPM
  static unsigned long lastPrintTime = 0;  // Última vez que os dados foram impressos
  unsigned long currentTime = micros();   // Tempo atual em microssegundos
  static bool pidActive = false; // Flag para verificar se o PID está ativado
  static bool resetPID = false; // Flag para controlar o reset do PID

  // Calcula o RPM
  float rpm = calculateRPM();

  // Desativa o PID e reseta os parâmetros se o RPM zerar
  if (rpm == 0) 
  {
    pidActive = false; // Desativa o PID quando o RPM for 0
    resetPID = true;   // Ativa o reset do PID
  } 
  else 
  {
    resetPID = false;  // Desativa o reset do PID quando o RPM for diferente de 0
  }

  // Ativa o PID apenas quando o RPM estiver em torno de 30 RPM
  if (rpm >= 30 && !pidActive) 
  {
    pidActive = true; // Ativa o PID quando o RPM atingir o valor mínimo
  }


  // Define o setpoint com base no RPM atual
  setpoint = SETPOINT_RPM(rpm);

  // Se o PID está ativo, calcula a saída do PID
  float smoothedPidOutput = 0;
  if (pidActive) 
  {
    // Calcula a saída do PID
    float pidOutput = computePID(rpm, setpoint, resetPID);

    // Ajusta a direção e velocidade do motor com a saída suavizada
    smoothedPidOutput = activateMotor(rpm, pidOutput);
  }

  // Verifica se já passou 100ms para exibir os dados no monitor serial
  if (currentTime - lastPrintTime >= 100) 
  {
    // Exibe informações no monitor serial
    printData(rpm, setpoint, smoothedPidOutput);

    // Atualiza o tempo da última impressão
    lastPrintTime = currentTime;
  }
}
