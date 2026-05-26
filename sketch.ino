#include <Servo.h>

// Робот с обходом препятствий для Wokwi.
// HC-SR04 сканирует пространство через сервопривод, два A4988 управляют левым и правым моторами.

// -------------------- Пины схемы --------------------
// Названия пинов синхронизированы с diagram.json, чтобы схему было проще ревьюить.
const byte PIN_LEFT_DIR = 2;
const byte PIN_RIGHT_DIR = 3;
const byte PIN_LEFT_ENABLE = 4;
const byte PIN_LEFT_STEP = 5;
const byte PIN_RIGHT_STEP = 6;
const byte PIN_TRIG = 7;
const byte PIN_ECHO = 8;
const byte PIN_SERVO = 9;
const byte PIN_LED_GREEN = 10;
const byte PIN_LED_RED = 11;
const byte PIN_RIGHT_ENABLE = 12;
const byte PIN_BUZZER = 13;

// -------------------- Настройки алгоритма --------------------
// Два порога образуют гистерезис: вход в обход при 25 см, возврат к норме при 35 см.
const unsigned int DISTANCE_OBSTACLE_CM = 25;
const unsigned int DISTANCE_CLEAR_CM = 35;
const unsigned int MAX_DISTANCE_CM = 400;
const unsigned long ECHO_TIMEOUT_US = 25000UL;

const byte SERVO_CENTER_DEG = 90;
const byte SERVO_LEFT_DEG = 150;
const byte SERVO_RIGHT_DEG = 30;

// Все периодические действия выполняются через millis()/micros(), без delay() в loop().
const unsigned long SENSOR_PERIOD_MS = 120;
const unsigned long SERIAL_PERIOD_MS = 500;
const unsigned long SERVO_SETTLE_MS = 350;
const unsigned long BACK_TIME_MS = 350;
const unsigned long TURN_TIME_MS = 650;
const unsigned long LED_BLINK_PERIOD_MS = 180;

const unsigned int STEP_INTERVAL_FORWARD_US = 1400;
const unsigned int STEP_INTERVAL_TURN_US = 1700;
const unsigned int BUZZER_TONE_HZ = 1200;

// -------------------- Состояние программы --------------------
Servo scannerServo;

// Конечный автомат: каждый режим описывает один этап поведения робота.
enum RobotMode {
  MODE_FORWARD,
  MODE_SCAN_LEFT,
  MODE_SCAN_RIGHT,
  MODE_BACKWARD,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT
};

RobotMode currentMode = MODE_FORWARD;

// obstacleLatched фиксирует обнаружение препятствия, чтобы робот не дёргался около порога.
bool obstacleLatched = false;
bool redLedState = false;
bool leftStepState = false;
bool rightStepState = false;

unsigned int distanceFrontCm = MAX_DISTANCE_CM;
unsigned int distanceLeftCm = MAX_DISTANCE_CM;
unsigned int distanceRightCm = MAX_DISTANCE_CM;

unsigned long lastSensorMs = 0;
unsigned long lastSerialMs = 0;
unsigned long lastBlinkMs = 0;
unsigned long modeStartedMs = 0;

unsigned long lastLeftStepUs = 0;
unsigned long lastRightStepUs = 0;

void setup() {
  // Настраиваем управляющие пины A4988, датчика и индикаторов.
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_ENABLE, OUTPUT);
  pinMode(PIN_RIGHT_ENABLE, OUTPUT);
  pinMode(PIN_LEFT_STEP, OUTPUT);
  pinMode(PIN_RIGHT_STEP, OUTPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  Serial.begin(115200);

  scannerServo.attach(PIN_SERVO);
  scannerServo.write(SERVO_CENTER_DEG);

  // Безопасное начальное состояние: моторы выключены, индикация сброшена.
  stopMotors();
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  noTone(PIN_BUZZER);

  modeStartedMs = millis();

  Serial.println("Robot obstacle avoidance with two A4988 drivers: start");
}

void loop() {
  // Время считается один раз за цикл и передаётся в функции, чтобы не плодить разные срезы времени.
  const unsigned long nowMs = millis();
  const unsigned long nowUs = micros();

  updateSensorsAndLogic(nowMs);
  updateActuators(nowMs, nowUs);
  printStatus(nowMs);
}

void updateSensorsAndLogic(unsigned long nowMs) {
  // В нормальном режиме регулярно смотрим вперёд и проверяем порог препятствия.
  if (currentMode == MODE_FORWARD && nowMs - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = nowMs;
    distanceFrontCm = measureDistanceCm();

    // Гистерезис предотвращает частые переключения при расстоянии около границы.
    if (!obstacleLatched && distanceFrontCm <= DISTANCE_OBSTACLE_CM) {
      obstacleLatched = true;
      enterMode(MODE_SCAN_LEFT);
      scannerServo.write(SERVO_LEFT_DEG);
    } else if (obstacleLatched && distanceFrontCm >= DISTANCE_CLEAR_CM) {
      obstacleLatched = false;
    }
  }

  // После поворота серво ждём стабилизацию, затем измеряем левую сторону.
  if (currentMode == MODE_SCAN_LEFT && nowMs - modeStartedMs >= SERVO_SETTLE_MS) {
    distanceLeftCm = measureDistanceCm();
    enterMode(MODE_SCAN_RIGHT);
    scannerServo.write(SERVO_RIGHT_DEG);
  }

  // Аналогично измеряем правую сторону и возвращаем датчик в центральное положение.
  if (currentMode == MODE_SCAN_RIGHT && nowMs - modeStartedMs >= SERVO_SETTLE_MS) {
    distanceRightCm = measureDistanceCm();
    scannerServo.write(SERVO_CENTER_DEG);
    enterMode(MODE_BACKWARD);
  }

  // Перед поворотом робот немного отъезжает назад, чтобы освободить место для манёвра.
  if (currentMode == MODE_BACKWARD && nowMs - modeStartedMs >= BACK_TIME_MS) {
    if (distanceLeftCm >= distanceRightCm) {
      enterMode(MODE_TURN_LEFT);
    } else {
      enterMode(MODE_TURN_RIGHT);
    }
  }

  // Поворот ограничен временем: для учебной модели этого достаточно и легко проверяется в Wokwi.
  if ((currentMode == MODE_TURN_LEFT || currentMode == MODE_TURN_RIGHT) &&
      nowMs - modeStartedMs >= TURN_TIME_MS) {
    obstacleLatched = false;
    enterMode(MODE_FORWARD);
  }
}

void updateActuators(unsigned long nowMs, unsigned long nowUs) {
  // Исполнительная часть отделена от логики: здесь только моторы, светодиоды и зуммер.
  switch (currentMode) {
    case MODE_FORWARD:
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);
      noTone(PIN_BUZZER);
      moveForward(nowUs);
      break;

    case MODE_SCAN_LEFT:
    case MODE_SCAN_RIGHT:
      stopMotors();
      digitalWrite(PIN_LED_GREEN, LOW);
      blinkRedLed(nowMs);
      tone(PIN_BUZZER, BUZZER_TONE_HZ);
      break;

    case MODE_BACKWARD:
      digitalWrite(PIN_LED_GREEN, LOW);
      blinkRedLed(nowMs);
      tone(PIN_BUZZER, BUZZER_TONE_HZ);
      moveBackward(nowUs);
      break;

    case MODE_TURN_LEFT:
      digitalWrite(PIN_LED_GREEN, LOW);
      blinkRedLed(nowMs);
      tone(PIN_BUZZER, BUZZER_TONE_HZ);
      turnLeft(nowUs);
      break;

    case MODE_TURN_RIGHT:
      digitalWrite(PIN_LED_GREEN, LOW);
      blinkRedLed(nowMs);
      tone(PIN_BUZZER, BUZZER_TONE_HZ);
      turnRight(nowUs);
      break;
  }
}

unsigned int measureDistanceCm() {
  // HC-SR04 требует короткий TRIG-импульс; delayMicroseconds здесь допустим и очень короткий.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  const unsigned long durationUs = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

  // Если эхо не пришло за таймаут, считаем, что препятствие вне рабочего диапазона.
  if (durationUs == 0) {
    return MAX_DISTANCE_CM;
  }

  // Для HC-SR04 расстояние в сантиметрах приблизительно равно длительности импульса / 58.
  const unsigned int distanceCm = durationUs / 58;
  return constrain(distanceCm, 2, MAX_DISTANCE_CM);
}

void moveForward(unsigned long nowUs) {
  setMotorDirections(true, true);
  enableMotors();
  makeSteps(nowUs, STEP_INTERVAL_FORWARD_US);
}

void moveBackward(unsigned long nowUs) {
  setMotorDirections(false, false);
  enableMotors();
  makeSteps(nowUs, STEP_INTERVAL_TURN_US);
}

void turnLeft(unsigned long nowUs) {
  setMotorDirections(false, true);
  enableMotors();
  makeSteps(nowUs, STEP_INTERVAL_TURN_US);
}

void turnRight(unsigned long nowUs) {
  setMotorDirections(true, false);
  enableMotors();
  makeSteps(nowUs, STEP_INTERVAL_TURN_US);
}

void setMotorDirections(bool leftForward, bool rightForward) {
  digitalWrite(PIN_LEFT_DIR, leftForward ? HIGH : LOW);

  // Правый мотор стоит зеркально левому, поэтому направление намеренно инвертировано.
  digitalWrite(PIN_RIGHT_DIR, rightForward ? LOW : HIGH);
}

void enableMotors() {
  // У A4988 вход ENABLE активен низким уровнем.
  digitalWrite(PIN_LEFT_ENABLE, LOW);
  digitalWrite(PIN_RIGHT_ENABLE, LOW);
}

void stopMotors() {
  // Остановка делается через ENABLE, а STEP дополнительно сбрасывается в LOW для понятной осциллограммы.
  digitalWrite(PIN_LEFT_ENABLE, HIGH);
  digitalWrite(PIN_RIGHT_ENABLE, HIGH);
  digitalWrite(PIN_LEFT_STEP, LOW);
  digitalWrite(PIN_RIGHT_STEP, LOW);
}

void makeSteps(unsigned long nowUs, unsigned int stepIntervalUs) {
  // Неблокирующая генерация STEP: моторы продолжают вращаться, пока loop обслуживает датчики и Serial.
  if (nowUs - lastLeftStepUs >= stepIntervalUs) {
    lastLeftStepUs = nowUs;
    leftStepState = !leftStepState;
    digitalWrite(PIN_LEFT_STEP, leftStepState ? HIGH : LOW);
  }

  if (nowUs - lastRightStepUs >= stepIntervalUs) {
    lastRightStepUs = nowUs;
    rightStepState = !rightStepState;
    digitalWrite(PIN_RIGHT_STEP, rightStepState ? HIGH : LOW);
  }
}

void enterMode(RobotMode newMode) {
  // Единая точка смены режима сбрасывает таймер текущего этапа.
  currentMode = newMode;
  modeStartedMs = millis();
}

void blinkRedLed(unsigned long nowMs) {
  // Мигание сделано без delay(), чтобы не блокировать сканирование и управление моторами.
  if (nowMs - lastBlinkMs >= LED_BLINK_PERIOD_MS) {
    lastBlinkMs = nowMs;
    redLedState = !redLedState;
    digitalWrite(PIN_LED_RED, redLedState ? HIGH : LOW);
  }
}

void printStatus(unsigned long nowMs) {
  // Serial выводится раз в 500 мс: достаточно информативно и без спама в монитор.
  if (nowMs - lastSerialMs < SERIAL_PERIOD_MS) {
    return;
  }

  lastSerialMs = nowMs;

  Serial.print("Front: ");
  Serial.print(distanceFrontCm);
  Serial.print(" cm | Left: ");
  Serial.print(distanceLeftCm);
  Serial.print(" cm | Right: ");
  Serial.print(distanceRightCm);
  Serial.print(" cm | Status: ");
  Serial.println(modeToText(currentMode));
}

const char* modeToText(RobotMode mode) {
  switch (mode) {
    case MODE_FORWARD:
      return "движение вперёд";
    case MODE_SCAN_LEFT:
      return "сканирование слева";
    case MODE_SCAN_RIGHT:
      return "сканирование справа";
    case MODE_BACKWARD:
      return "отъезд назад";
    case MODE_TURN_LEFT:
      return "поворот налево";
    case MODE_TURN_RIGHT:
      return "поворот направо";
    default:
      return "неизвестно";
  }
}
