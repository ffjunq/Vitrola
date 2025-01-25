#include <Arduino.h>

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

// Variáveis para suavização do output
float smoothedOutput = 0; // Saída suavizada
float smoothingFactor = 0.005; // Fator de suavização (quanto menor, mais suave)


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

  // Inicializa motor parado
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(0, 0);

  // Configura a interrupção para disparar na borda de descida (fita branca detectada)
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);
}

void loop() 
{
  static float rpm = 0;               // Valor atual de RPM
  static float setpoint = 0;          // Valor desejado de RPM
  static float error = 0;             // Erro atual
  static float lastError = 0;         // Erro na última iteração
  static float integral = 0;          // Acumulador do erro
  static float derivative = 0;        // Taxa de variação do erro
  static float output = 0;            // Saída do controlador PID

  static unsigned long lastPrintTime = 0;  // Última vez que os dados foram impressos
  unsigned long currentTime = micros();   // Tempo atual em microssegundos

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
  if ((currentTime - lastPulseTime) > RPM_RESET_TIMEOUT) 
  {
    rpm = 0;
  }

  // Define o setpoint com base no RPM atual
  setpoint = 33.3;
  if (rpm >= 40) 
  {
    setpoint = 45.0;
  }

  // Controle PID
  error = setpoint - rpm;
  integral += error;
  derivative = error - lastError;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Limita a saída entre 0 e 255 (valores válidos para PWM)
  output = constrain(output, 150, 180);

  // Condição para usar smoothedOutput apenas entre RPMs específicos
  if ((rpm >= 32 && rpm <= 34) || (rpm >= 44 && rpm <= 46)) {
    // Suavização da transição do output
    smoothedOutput = smoothedOutput + smoothingFactor * (output - smoothedOutput);
  } else {
    // Se não estiver nos intervalos, usa a saída bruta
    smoothedOutput = output;
  }

  // Ajusta a direção e velocidade do motor com a saída suavizada
  if (smoothedOutput > 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    ledcWrite(0, smoothedOutput); // Define a velocidade suavizada
  }

  // Verifica se já passou 100ms para exibir os dados no monitor serial
  if (currentTime - lastPrintTime >= 100) {
    // Exibe informações no monitor serial
    Serial.print("RPM: ");
    Serial.print(rpm, 2);
    Serial.print(" | Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Output: ");
    Serial.println(smoothedOutput);

    // Atualiza o tempo da última impressão
    lastPrintTime = currentTime;
  }
}
