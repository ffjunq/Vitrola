#include <Arduino.h>

#define SENSOR_PIN 34              // Pino digital onde o TCRT5000 está conectado
#define DEBOUNCE_TIME 100000         // Tempo de debounce em microssegundos (5ms)
#define RPM_RESET_TIMEOUT 3000000  // Tempo limite para zerar o RPM (3 segundos em microssegundos)

volatile unsigned long lastPulseTime = 0; // Tempo do último pulso válido
volatile unsigned long pulseInterval = 0; // Intervalo entre os últimos dois pulsos
volatile bool pulseDetected = false;      // Flag para indicar um novo pulso

void IRAM_ATTR countPulse() 
{
  unsigned long currentTime = micros(); // Tempo atual em microssegundos

  // Verifica se o intervalo desde o último pulso é maior que o tempo de debounce
  if ((currentTime - lastPulseTime) >= DEBOUNCE_TIME) 
  {
    pulseInterval = currentTime - lastPulseTime; // Calcula o intervalo desde o último pulso válido
    pulseDetected = true; // Marca que um novo pulso foi registrado
    lastPulseTime = currentTime; // Atualiza o tempo do último pulso válido
  }
}

void setup() 
{
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT); // Configura o pino do sensor como entrada

  // Configura a interrupção para disparar na borda de descida (fita branca detectada)
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);
}

void loop() 
{
  static float rpm = 0;                   // Variável para armazenar o valor calculado de RPM
  unsigned long currentTime = micros();   // Tempo atual em microssegundos

  // Calcula o RPM somente quando um novo pulso é detectado
  if (pulseDetected) 
  {
    noInterrupts(); // Desativa as interrupções para ler `pulseInterval` com segurança
    unsigned long interval = pulseInterval;
    pulseDetected = false; // Reseta a flag
    interrupts(); // Reativa as interrupções

    // Calcula o RPM: RPM = (1 / intervalo em segundos) * 60
    rpm = (1.0 / (interval / 1000000.0)) * 60.0;
  }

  // Zera o RPM se o último pulso foi há mais de 3 segundos
  if ((currentTime - lastPulseTime) > RPM_RESET_TIMEOUT) 
  {
    rpm = 0;
  }

    Serial.print("RPM: ");
    Serial.println(rpm, 4); // Exibe com 4 casas decimais

  delay(100); // Pequeno atraso para evitar poluição no monitor serial
}
