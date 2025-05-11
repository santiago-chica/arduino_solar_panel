/* Integrantes del grupo
- Fulanito de tal...
...
*/

// Importaciones
#include <Servo.h>

// Definición de fotoresistencias
const uint8_t ldrBottomRightPin = A0;
const uint8_t ldrBottomLeftPin = A1;
const uint8_t ldrTopRightPin = A2;
const uint8_t ldrTopLeftPin = A3;
// Definición de servos
const uint8_t servoLeftRightPin = 2;
const uint8_t servoTopBottomPin = 3;
// Definición de LED
const uint8_t calibrationLed = 4;

const Servo servoLeftRight;
const Servo servoTopBottom;

// Constantes modificables
const uint8_t maxDegrees = 180; // Máximo de grados de los servomotores

const float calibrationSeconds = 10.0;
const unsigned int calibrationMilliseconds = calibrationSeconds * 1000;

const uint8_t minLdr = 0; // Valor mínimo predeterminado
const uint8_t maxLdr = 1023; // Valor máximo predeterminado

const uint8_t delayMs = 20; // Espera para reposicionar

const uint8_t threshold = 5; // Que tanto debe variar como mínimo para moverse
const uint8_t step = 2; // Paso de los motores para moverse

// Clase para LDR
class LDR {
  public:
    uint8_t pin;
    uint8_t minValue;
    uint8_t maxValue;

    LDR(uint8_t inputPin) {
      pin = inputPin;
      minValue = maxLdr;
      maxValue = minValue;
    }

    void initialize() {
      pinMode(pin, INPUT);
    }

    void calibrate() {
      const uint8_t readValue = analogRead(pin);
      minValue = min(minValue, readValue);
      maxValue = max(maxValue, readValue);
    }

    int readNormalized() {
      const uint8_t readValue = analogRead(pin);
      return map(readValue, minValue, maxValue, 0, 100);
    }
};

LDR ldrBottomRight = LDR(ldrBottomRightPin);
LDR ldrBottomLeft = LDR(ldrBottomLeftPin);
LDR ldrTopRight = LDR(ldrTopRightPin);
LDR ldrTopLeft = LDR(ldrTopLeftPin);

void setup() {
  // Colocar pines en su modo correspondiente
  ldrBottomRight.initialize();
  ldrBottomLeft.initialize();
  ldrTopRight.initialize();
  ldrTopLeft.initialize();
  pinMode(calibrationLed, OUTPUT);
  // Colocar servomotores según su pin
  servoLeftRight.attach(servoLeftRightPin, 0, maxDegrees);
  servoTopBottom.attach(servoTopBottomPin, 0, maxDegrees);
  // Escribir servomotores en el punto medio
  servoLeftRight.write(maxDegrees / 2);
  servoTopBottom.write(maxDegrees / 2);
}

void loop() {
  // Calibración
  if (millis() < calibrationMilliseconds) {
    calibration();
    return;
  }

  digitalWrite(calibrationLed, LOW);

  trackLight();

  delay(delayMs);
}

void calibration() {
  // Colocar led
  digitalWrite(calibrationLed, HIGH);
  // Calibrar
  ldrBottomRight.calibrate();
  ldrBottomLeft.calibrate();
  ldrTopRight.calibrate();
  ldrTopLeft.calibrate();
}

void trackLight() {
  // Leer valores normalizados
  uint8_t bottomRight = ldrBottomRight.readNormalized();
  uint8_t bottomLeft = ldrBottomLeft.readNormalized();
  uint8_t topRight = ldrTopRight.readNormalized();
  uint8_t topLeft = ldrTopLeft.readNormalized();
  // Promediar
  int topAverage = (topRight + topLeft) / 2;
  int rightAverage = (topRight + bottomRight) / 2;
  int bottomAverage = (bottomRight + bottomLeft) / 2;
  int leftAverage = (topLeft + bottomLeft) / 2;
  // Obtener diferencia
  int horizontalDiff = leftAverage - rightAverage;
  int verticalDiff = topAverage - bottomAverage;
  // Posición servomotores
  int leftRightVal = servoLeftRight.read();
  int topBottomVal = servoTopBottom.read();

  // Ajustarlos acordemente
  if (abs(horizontalDiff) > threshold) {
    leftRightVal += (horizontalDiff > 0) ? -step: step;
    leftRightVal = constrain(leftRightVal, 0, maxDegrees);
    servoLeftRight.write(leftRightVal);
  }

  if (abs(verticalDiff) > threshold) {
    topBottomVal += (verticalDiff > 0) ? -step: step;
    topBottomVal = constrain(topBottomVal, 0, maxDegrees);
    servoTopBottom.write(topBottomVal);
  }
}
