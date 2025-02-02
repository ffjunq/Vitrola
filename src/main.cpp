#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

// ========== Configurações ========== //
#define SENSOR_PIN          34
#define IN3_PIN             25
#define IN4_PIN             26
#define ENB_PIN             27

#define PWM_INIT_33         235
#define PWM_INIT_45         200
#define PWM_FREQ            5000
#define PWM_RESOLUTION      8

#define PULSES_PER_REV      4
#define DEBOUNCE_US         100000UL     
#define RPM_TIMEOUT_US      5000000UL      // 3 segundos
#define MIN_INTERVAL_US     100000UL    // 100ms (proteção contra leituras inválidas)

// ========== Parâmetros PID ========== //
typedef struct {
    float Kp, Ki, Kd;
    float integral_limit;
} PIDParams;

static const PIDParams PID_33 = {2.5f, 0.5f, 1.0f, 25.0f};
static const PIDParams PID_45 = {3.2f, 0.18f, 1.5f, 1.5f};

// ========== Estruturas ========== //
typedef struct {
    volatile uint32_t last_pulse;
    volatile uint32_t interval;
    float filtered_rpm;
    float alpha;
} RPMSensor;

typedef struct {
    const PIDParams* params;
    float integral;
    float prev_measurement;
    float prev_filtered_derivative;
} PIDController;

typedef struct {
    uint8_t current_pwm;
    float filter_alpha;
} MotorDriver;

typedef enum { STATE_IDLE, STATE_RUNNING, DO_NOTHING } SystemState;

typedef struct {
    RPMSensor sensor;
    PIDController pid;
    MotorDriver motor;
    SystemState state;
    float setpoint;
    volatile bool pid_update;
    volatile uint32_t last_pulse_time;  // Novo: Controle de timeout
} SystemController;

static SystemController sys;

// ========== ISR do Sensor ========== //
void IRAM_ATTR pulse_isr(void* arg) {
    RPMSensor* sensor = (RPMSensor*)arg;
    const uint32_t now = micros();
    // Cálculo seguro de delta (considera overflow)
    uint32_t delta = (now >= sensor->last_pulse) ? (now - sensor->last_pulse) : (0xFFFFFFFF - sensor->last_pulse + now);
    
    if (delta > DEBOUNCE_US) { // Debounce de 50ms
        //Serial.printf("[ISR] Pulso! Delta: %lu us | Last Pulse: %lu | Now: %lu\n", delta, sensor->last_pulse, now);
        noInterrupts();
        sensor->interval = delta;
        sensor->last_pulse = now;
        sys.pid_update = true;
        sys.last_pulse_time = now;
        interrupts();
        
        // Intervalo mínimo coerente com RPM máxima (50 RPM)
        if (sensor->interval < MIN_INTERVAL_US) {
            sensor->interval = MIN_INTERVAL_US;
        }
    }
}

// ========== RPM ========== //
void rpm_init(RPMSensor* sensor) {
    sensor->last_pulse = 0;
    sensor->interval = 0;
    sensor->filtered_rpm = 0.0f;
    sensor->alpha = 0.6f;
    pinMode(SENSOR_PIN, INPUT);
    attachInterruptArg(SENSOR_PIN, pulse_isr, sensor, FALLING);
}

float rpm_get(RPMSensor* sensor) {
    while (!sys.pid_update) {
        uint32_t now, last_pulse;
        // Leitura atômica de last_pulse
        noInterrupts();
        last_pulse = sensor->last_pulse;
        interrupts();
        
        now = micros();
        uint32_t delta = (now >= last_pulse) ? (now - last_pulse) : (0xFFFFFFFF - last_pulse + now);

        if (delta > RPM_TIMEOUT_US) {
            //Serial.printf("[RPM GET] Sensor timeout! Last pulse: %lu us ago | NOW = %lu us\n", delta, now);
            return 0.0f;
        }
    }

    // Leitura atômica de interval e last_pulse
    uint32_t interval;
    noInterrupts();
    interval = sensor->interval;
    interrupts();

    if (interval == 0 || PULSES_PER_REV == 0) {
        Serial.printf("Sensor interval = %lu \n", interval);
        return 0.0f;
    }

    const float raw_rpm = 60000000.0f / (interval * PULSES_PER_REV);
    sensor->filtered_rpm = sensor->alpha * raw_rpm + (1 - sensor->alpha) * sensor->filtered_rpm;
    return sensor->filtered_rpm;
}

// ========== PID ========== //
void pid_init(PIDController* pid) {
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
}

float pid_compute(PIDController* pid, float setpoint, float measurement) {
    const float dt = fmaxf(sys.sensor.interval / 1000000.0f, 0.001f); // Garanta que dt nunca seja zero
    const float error = setpoint - measurement;
    const float base_pwm = (sys.setpoint >= 45.0f) ? PWM_INIT_45 : PWM_INIT_33;
    
    // Proporcional
    const float P = pid->params->Kp * error;

    // Integral (sem duplicação)
    pid->integral += pid->params->Ki * error * dt;
    pid->integral = fmaxf(fminf(pid->integral, pid->params->integral_limit), 
                         -pid->params->integral_limit);
    
    // Derivativo
    float D = 0.0f;
    float derivative = (measurement - pid->prev_measurement) / dt;
    float filtered_derivative = 0.2 * derivative + 0.8 * pid->prev_filtered_derivative;
    if (dt > 0.001f) {
        D = -pid->params->Kd * filtered_derivative;
        pid->prev_filtered_derivative = filtered_derivative;
    }

    // Saída do PID com limitação
    const float pid_out = P + pid->integral + D;
    const float clamped_pid = constrain(pid_out, -30, 30); // Limite seguro

    // Anti-windup: se a saída estiver limitada, não acumule integral
    if (pid_out != clamped_pid) {
        pid->integral -= (pid_out - clamped_pid) * pid->params->Ki * dt * 0.5f; 
    }

    pid->prev_measurement = measurement;

    Serial.printf("[PID] Set: %.1f | Med: %.1f | Err: %.2f | P: %.2f | I: %.2f | D: %.2f \n",
                setpoint, measurement, error, P, pid->integral, D);

    Serial.printf("[PID] Clamped: %.1f | Integral Limit: %.1f\n", clamped_pid, pid->params->integral_limit);
    Serial.printf("[PID] Raw Out: %.1f | Clamped: %.1f\n", pid_out, clamped_pid);
    
    return clamped_pid; // Retorna valor limitado
}

// ========== Motor ========== //
void motor_init(MotorDriver* motor) {
    motor->current_pwm = 0;
    motor->filter_alpha = 0.1f;
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    ledcAttachPin(ENB_PIN, 0);
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    ledcWrite(0, PWM_INIT_33);
}


void motor_set_pwm(MotorDriver* motor, uint8_t pwm) {
    // Limita valores inválidos
    if (pwm > 255) pwm = 255;
    motor->current_pwm = motor->filter_alpha * pwm + 
                        (1 - motor->filter_alpha) * motor->current_pwm;
    ledcWrite(0, (uint32_t)motor->current_pwm);
}

// ========== Sistema ========== //
void system_init() {
    rpm_init(&sys.sensor);
    motor_init(&sys.motor);
    sys.state = STATE_IDLE;
    sys.setpoint = 0.0f;
    sys.pid_update = false;
    sys.last_pulse_time = 0;
}

void system_run() {
    const float rpm = rpm_get(&sys.sensor);

    // Failsafe: se RPM = 0 por timeout, mantém último PWM válido
    if (rpm == 0.0f && sys.state == STATE_RUNNING) {
        Serial.println("[Failsafe] Timeout do sensor! Mantendo PWM atual.");
        sys.state = STATE_IDLE;
        return; // Não atualiza o PWM
    }

    // Verifica RPM inválido
    if (isnan(rpm) || isinf(rpm)) {
        Serial.println("[ERRO] RPM inválido! Reiniciando...");
        sys.state = STATE_IDLE;
        pid_init(&sys.pid);
        motor_set_pwm(&sys.motor, 0);
        return;
    }

    switch(sys.state) {
        case STATE_IDLE:
            if(rpm > 30.0f) {
                Serial.println("[TRANSIÇÃO] IDLE -> RUNNING");
                sys.setpoint = (rpm >= 38.0f) ? 45.0f : 33.3f;
                sys.pid.params = (sys.setpoint >= 45.0f) ? &PID_45 : &PID_33;
                pid_init(&sys.pid);
                sys.state = STATE_RUNNING;
            } else {
                const uint8_t pwm = (sys.setpoint >= 45.0f) ? PWM_INIT_45 : PWM_INIT_33;
                motor_set_pwm(&sys.motor, pwm);
            }
            break;
            
        case STATE_RUNNING:
            if(sys.pid_update) {
                const float pid_out = pid_compute(&sys.pid, sys.setpoint, rpm);
                const uint8_t base_pwm = (sys.setpoint >= 45.0f) ? PWM_INIT_45 : PWM_INIT_33;
                motor_set_pwm(&sys.motor, (uint8_t)constrain(base_pwm + pid_out, 0, 255));
                sys.pid_update = false;
                Serial.printf("[MOTOR] PWM Base: %d | PID Out: %.1f | PWM Final: %d\n",
                              base_pwm, pid_out, (uint8_t)constrain(base_pwm + pid_out, 0, 255));
            }
            
            if(rpm == 0.0f) {
                Serial.println("[TRANSIÇÃO] RUNNING -> IDLE");
                sys.state = STATE_IDLE;
                pid_init(&sys.pid);
            }
            break;
    }
}

// ========== Principal ========== //
void setup() {
    Serial.begin(115200);
    Serial.println("\n[SISTEMA] Inicializando...");
    system_init();
    Serial.println("[SISTEMA] Pronto");
}

void loop() {
    system_run();
}