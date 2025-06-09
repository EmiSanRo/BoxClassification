#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// Pines del encoder
const int encoderPinA = 34;
const int encoderPinB = 35;

// Pines del IBT-2
const int RPWM = 27;
const int LPWM = 14;
const int REN  = 25;
const int LEN  = 26;

// PWM config
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;
const int PWM_CH_R = 0;
const int PWM_CH_L = 1;

// Encoder + reductora
volatile long encoderCount = 0;
int lastEncoded = 0;
const int  PULSOS_POR_VUELTA_MOTOR = 42;
const float GEAR_RATIO              = 210.0;
const float PULSOS_POR_VUELTA_EJE   = PULSOS_POR_VUELTA_MOTOR * GEAR_RATIO;
const float GRADOS_POR_PULSO        = 360.0 / PULSOS_POR_VUELTA_EJE;

unsigned long last_control_time = 0;
unsigned long last_time = 0;
const unsigned long CONTROL_INTERVAL_MS = 20;

float gradosAnterior = 0.0;

// PID
float setpointDestino = 90.0;
float Kp = 4.0, Ki = 1.5, Kd = 1.0;
float integral = 0.0, lastError = 0.0;

// Control por estado
bool movimientoActivo   = false;
bool destinoAlcanzado   = false;
bool regresando         = false;
bool fueHaciaAdelante   = true;
bool nuevaMision        = false;
unsigned long tiempoLlegada = 0;
const unsigned long tiempoEspera = 6000;
long encoderInicio = 0;

// Micro-ROS
rcl_publisher_t actuador_publisher;
rcl_publisher_t status_publisher;
rcl_subscription_t motor_control_subscriber;
std_msgs__msg__Int32     motor_control_msg;
std_msgs__msg__Int32     actuador_msg;
std_msgs__msg__String    status_msg;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

static char status_buffer[128];
void publish_status(const char * fmt, ...) {
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(status_buffer, sizeof(status_buffer), fmt, args);
  va_end(args);
  if (len <= 0) return;
  size_t needed = len + 1;
  if (status_msg.data.capacity < needed) {
    status_msg.data.data = (char*)realloc(status_msg.data.data, needed);
    status_msg.data.capacity = needed;
  }
  memcpy(status_msg.data.data, status_buffer, needed);
  status_msg.data.size = len;
  rcl_publish(&status_publisher, &status_msg, NULL);
}

void motor_control_cb(const void* msgin) {
  auto msg = (const std_msgs__msg__Int32*)msgin;
  publish_status("Recibido: %d", msg->data);
  if (msg->data == 1 && !movimientoActivo) {
    movimientoActivo = true;
    destinoAlcanzado = false;
    regresando = false;
    nuevaMision = true;
    encoderInicio = encoderCount;
    publish_status("Iniciando movimiento hacia destino...");
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  delay(2000);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(&actuador_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "actuador");
  actuador_msg.data = 3;

  rclc_publisher_init_default(&status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status");
  std_msgs__msg__String__init(&status_msg);

  rclc_subscription_init_default(&motor_control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_control");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &motor_control_subscriber, &motor_control_msg, motor_control_cb, ON_NEW_DATA);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, PWM_CH_R);
  ledcAttachPin(LPWM, PWM_CH_L);

  publish_status("Listo. Esperando comando en 'motor_control'...");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (!movimientoActivo) return;

  unsigned long now = millis();
  if (now - last_control_time < CONTROL_INTERVAL_MS) return;
  last_control_time = now;

  float dt = (now - last_time) / 1000.0;
  last_time = now;

  float grados = encoderCount * GRADOS_POR_PULSO;
  grados = fmod(grados, 360.0);
  if (grados < 0) grados += 360;

  const long MAX_PULSOS_SEGURIDAD = PULSOS_POR_VUELTA_EJE / 4;
  long diferencia = encoderCount - encoderInicio;

  if (abs(diferencia) > MAX_PULSOS_SEGURIDAD) {
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
    movimientoActivo = false;
    regresando = false;
    publish_status("\u26a0\ufe0f Movimiento detenido: l\u00edmite de 180\u00b0 superado.");
    return;
  }

  if (regresando) {
    bool cruzoCero = false;
    if (fueHaciaAdelante && gradosAnterior > 10.0 && grados < 10.0) cruzoCero = true;
    if (!fueHaciaAdelante && gradosAnterior < 350.0 && grados > 350.0) cruzoCero = true;

    if (cruzoCero) {
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, 0);
      movimientoActivo = false;
      regresando = false;
      publish_status("Regreso completado EXACTO en sentido contrario.");
      rcl_publish(&actuador_publisher, &actuador_msg, NULL);
    } else {
      if (fueHaciaAdelante) {
        ledcWrite(PWM_CH_R, 160);
        ledcWrite(PWM_CH_L, 0);
        publish_status("Regresando (adelante \u2192 atr\u00e1s)");
      } else {
        ledcWrite(PWM_CH_R, 0);
        ledcWrite(PWM_CH_L, 160);
        publish_status("Regresando (atr\u00e1s \u2192 adelante)");
      }
      publish_status("\u00c1ngulo actual: %.2f | \u00c1ngulo previo: %.2f", grados, gradosAnterior);
    }

    gradosAnterior = grados;
    return;
  }

  if (!destinoAlcanzado) {
    float error = setpointDestino - grados;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (nuevaMision) {
      fueHaciaAdelante = (error > 0);
      nuevaMision = false;
    }

    const float DEAD_ZONE = 1.0;
    if (abs(error) < DEAD_ZONE) {
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, 0);
      integral = 0;
      lastError = 0;
      destinoAlcanzado = true;
      tiempoLlegada = now;
      publish_status("Destino alcanzado. Esperando para regresar...");
      return;
    }

    if (error * lastError < 0 || abs(error) > 180) {
      integral = 0;
    }

    float proportional = Kp * error;
    float derivative = (error - lastError) / dt;
    float controlSignal = proportional + Ki * integral + Kd * derivative;
    lastError = error;

    int pwmOutput = abs((int)controlSignal);
    if (pwmOutput < 255) integral += error * dt;

    const int PWM_MIN = 25;
    if (pwmOutput > 0 && pwmOutput < PWM_MIN) pwmOutput = PWM_MIN;
    if (pwmOutput > 255) pwmOutput = 255;

    if (controlSignal > 0) {
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, pwmOutput);
    } else {
      ledcWrite(PWM_CH_R, pwmOutput);
      ledcWrite(PWM_CH_L, 0);
    }
    publish_status("\u00c1ngulo: %.2f | Error: %.2f | PWM: %d", grados, error, pwmOutput);
  }

  if (destinoAlcanzado && (now - tiempoLlegada >= tiempoEspera)) {
    regresando = true;
    publish_status("Iniciando regreso autom\u00e1tico al origen...");
  }
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount++;
  lastEncoded = encoded;
}
#include <string.h>
#include <stdarg.h>

// Pines del encoder
const int encoderPinA = 34;
const int encoderPinB = 35;

// Pines del IBT-2
const int RPWM = 27;
const int LPWM = 14;
const int REN  = 25;
const int LEN  = 26;

// PWM config
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;
const int PWM_CH_R = 0;
const int PWM_CH_L = 1;

// Encoder + reductora
volatile long encoderCount = 0;
int lastEncoded = 0;
const int  PULSOS_POR_VUELTA_MOTOR = 42;
const float GEAR_RATIO              = 210.0;
const float PULSOS_POR_VUELTA_EJE   = PULSOS_POR_VUELTA_MOTOR * GEAR_RATIO;
const float GRADOS_POR_PULSO        = 360.0 / PULSOS_POR_VUELTA_EJE;

unsigned long last_control_time = 0;
unsigned long last_time = 0;
const unsigned long CONTROL_INTERVAL_MS = 20;

float gradosAnterior = 0.0;

// PID
float setpointDestino = 90.0;
float Kp = 4.0, Ki = 1.5, Kd = 1.0;
float integral = 0.0, lastError = 0.0;

// Control por estado
bool movimientoActivo   = false;
bool destinoAlcanzado   = false;
bool regresando         = false;
bool fueHaciaAdelante   = true;
bool nuevaMision        = false;
unsigned long tiempoLlegada = 0;
const unsigned long tiempoEspera = 6000;
long encoderInicio = 0;

// Micro-ROS
rcl_publisher_t actuador_publisher;
rcl_publisher_t status_publisher;
rcl_subscription_t motor_control_subscriber;
std_msgs__msg__Int32     motor_control_msg;
std_msgs__msg__Int32     actuador_msg;
std_msgs__msg__String    status_msg;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

static char status_buffer[128];
void publish_status(const char * fmt, ...) {
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(status_buffer, sizeof(status_buffer), fmt, args);
  va_end(args);
  if (len <= 0) return;
  size_t needed = len + 1;
  if (status_msg.data.capacity < needed) {
    status_msg.data.data = (char*)realloc(status_msg.data.data, needed);
    status_msg.data.capacity = needed;
  }
  memcpy(status_msg.data.data, status_buffer, needed);
  status_msg.data.size = len;
  rcl_publish(&status_publisher, &status_msg, NULL);
}

void motor_control_cb(const void* msgin) {
  auto msg = (const std_msgs__msg__Int32*)msgin;
  publish_status("Recibido: %d", msg->data);
  if (msg->data == 1 && !movimientoActivo) {
    movimientoActivo = true;
    destinoAlcanzado = false;
    regresando = false;
    nuevaMision = true;
    encoderInicio = encoderCount;
    publish_status("Iniciando movimiento hacia destino...");
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  delay(2000);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(&actuador_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "actuador");
  actuador_msg.data = 3;

  rclc_publisher_init_default(&status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status");
  std_msgs__msg__String__init(&status_msg);

  rclc_subscription_init_default(&motor_control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_control");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &motor_control_subscriber, &motor_control_msg, motor_control_cb, ON_NEW_DATA);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, PWM_CH_R);
  ledcAttachPin(LPWM, PWM_CH_L);

  publish_status("Listo. Esperando comando en 'motor_control'...");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (!movimientoActivo) return;

  unsigned long now = millis();
  if (now - last_control_time < CONTROL_INTERVAL_MS) return;
  last_control_time = now;

  float dt = (now - last_time) / 1000.0;
  last_time = now;

  float grados = encoderCount * GRADOS_POR_PULSO;
  grados = fmod(grados, 360.0);
  if (grados < 0) grados += 360;

  const long MAX_PULSOS_SEGURIDAD = PULSOS_POR_VUELTA_EJE / 4;
  long diferencia = encoderCount - encoderInicio;

  if (abs(diferencia) > MAX_PULSOS_SEGURIDAD) {
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
    movimientoActivo = false;
    regresando = false;
    publish_status("\u26a0\ufe0f Movimiento detenido: l\u00edmite de 180\u00b0 superado.");
    return;
  }

  if (regresando) {
    bool cruzoCero = false;
    if (fueHaciaAdelante && gradosAnterior > 10.0 && grados < 10.0) cruzoCero = true;
    if (!fueHaciaAdelante && gradosAnterior < 350.0 && grados > 350.0) cruzoCero = true;

    if (cruzoCero) {
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, 0);
      movimientoActivo = false;
      regresando = false;
      publish_status("Regreso completado EXACTO en sentido contrario.");
      rcl_publish(&actuador_publisher, &actuador_msg, NULL);
    } else {
      if (fueHaciaAdelante) {
        ledcWrite(PWM_CH_R, 160);
        ledcWrite(PWM_CH_L, 0);
        publish_status("Regresando (adelante \u2192 atr\u00e1s)");
      } else {
        ledcWrite(PWM_CH_R, 0);
        ledcWrite(PWM_CH_L, 160);
        publish_status("Regresando (atr\u00e1s \u2192 adelante)");
      }
      publish_status("\u00c1ngulo actual: %.2f | \u00c1ngulo previo: %.2f", grados, gradosAnterior);
    }

    gradosAnterior = grados;
    return;
  }

  if (!destinoAlcanzado) {
    float error = setpointDestino - grados;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (nuevaMision) {
      fueHaciaAdelante = (error > 0);
      nuevaMision = false;
    }

    const float DEAD_ZONE = 1.0;
    if (abs(error) < DEAD_ZONE) {
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, 0);
      integral = 0;
      lastError = 0;
      destinoAlcanzado = true;
      tiempoLlegada = now;
      publish_status("Destino alcanzado. Esperando para regresar...");
      return;
    }

    if (error * lastError < 0 || abs(error) > 180) {
      integral = 0;
    }

    float proportional = Kp * error;
    float derivative = (error - lastError) / dt;
    float controlSignal = proportional + Ki * integral + Kd * derivative;
    lastError = error;

    int pwmOutput = abs((int)controlSignal);
    if (pwmOutput < 255) integral += error * dt;

    const int PWM_MIN = 25;
    if (pwmOutput > 0 && pwmOutput < PWM_MIN) pwmOutput = PWM_MIN;
    if (pwmOutput > 255) pwmOutput = 255;

    if (controlSignal > 0) {
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, pwmOutput);
    } else {
      ledcWrite(PWM_CH_R, pwmOutput);
      ledcWrite(PWM_CH_L, 0);
    }
    publish_status("\u00c1ngulo: %.2f | Error: %.2f | PWM: %d", grados, error, pwmOutput);
  }

  if (destinoAlcanzado && (now - tiempoLlegada >= tiempoEspera)) {
    regresando = true;
    publish_status("Iniciando regreso autom\u00e1tico al origen...");
  }
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount++;
  lastEncoded = encoded;
}