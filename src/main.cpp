#include <Arduino.h>

// Configurações
#define SENSOR_PIN 34               // Pino digital onde o TCRT5000 está conectado
#define DEBOUNCE_TIME 100000        // Tempo de debounce em microssegundos (100ms)
#define RPM_RESET_TIMEOUT 3000000   // Tempo limite para zerar o RPM (3 segundos em microssegundos)

// Variáveis voláteis para uso na interrupção
volatile unsigned long lastPulseTime = 0; // Tempo do último pulso válido
volatile unsigned long pulseInterval = 0; // Intervalo entre os últimos dois pulsos
volatile bool pulseDetected = false;      // Flag para indicar um novo pulso detectado

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

  pinMode(SENSOR_PIN, INPUT); // Configura o pino do sensor como entrada

  // Configura a interrupção para disparar na borda de descida (fita branca detectada)
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);

  Serial.println("Inicializando medição de RPM...");
}

void loop() 
{
  static float rpm = 0;                   // Variável para armazenar o valor calculado de RPM
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

  // Exibe o valor de RPM
  Serial.print("RPM: ");
  Serial.println(rpm, 2); // Exibe com 2 casas decimais

  delay(100); // Atraso para evitar poluição no monitor serial
}
