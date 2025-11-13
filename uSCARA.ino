#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_TinyUSB.h"
#include <TMCStepper.h>
#include <math.h>

#define PROXIMAL_DIR 0
#define PROXIMAL_STEP 1
#define PROXIMAL_CS 2
#define PROXIMAL_EN 3

#define DISTAL_STEP 4
#define DISTAL_DIR 5
#define DISTAL_EN 6
#define DISTAL_CS 7

#define SCK_PIN 8
#define MISO_PIN 9
#define MOSI_PIN 10

#define R_SENSE 0.075f

#define N_MICROSTEPS 128
#define N_FULL_STEPS 200
#define BELT_REDUCTION 3.125

static const double STEPS_PER_DEG = BELT_REDUCTION * (double)N_MICROSTEPS * (double)N_FULL_STEPS / 360.0;

static const double L_PROX = 73.5;
static const double L_DIST = 47.0;

static const double DEFAULT_PULSE_US = 15;

// state
static double phi_actual_deg = 0.0;
static double theta_actual_deg = 0.0;

TMC5160Stepper pM(PROXIMAL_CS, R_SENSE);
TMC5160Stepper dM(DISTAL_CS, R_SENSE);

static inline double rad2deg(double r) {
  return r * 180.0 / M_PI;
}
static inline double deg2rad(double d) {
  return d * M_PI / 180.0;
}
static inline double sqr(double v) {
  return v * v;
}

bool ik(double x, double y, double* phi_deg, double* theta_deg) {
  // Workspace check
  if (x < 0) {
    Serial.println("Warning! x < 0");
    return false;
  }

  const double r = hypot(x, y);
  const double r_max = L_PROX + L_DIST;
  const double r_min = fabs(L_PROX - L_DIST);

  if (r < r_min || r > r_max) {
    // // Clamp to nearest reachable point
    // double scale = (r < r_min) ? (r_min / (r + 1e-9)) : (r_max / (r + 1e-9));
    // x *= scale;
    // y *= scale;
    Serial.println("Warning! point is unreachable");
    return false;
  }

  double c = (sqr(x) + sqr(y) - sqr(L_PROX) - sqr(L_DIST)) / (2 * L_PROX * L_DIST);
  if (c > 1.0) Serial.println("WTF");
  if (c < -1.0) Serial.println("WTF");
  const double theta = acos(c);

  const double k1 = L_PROX + L_DIST * cos(theta);
  const double k2 = L_DIST * sin(theta);
  const double phi = atan2(y, x) - atan2(k2, k1);

  *phi_deg = rad2deg(phi);
  *theta_deg = rad2deg(theta);

  // Normalize to [-180, 180) for consistency
  auto norm = [](double a) {
    while (a >= 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
  };

  *phi_deg = norm(*phi_deg);
  *theta_deg = norm(*theta_deg);

  return true;
}

void stepper_pulse(uint8_t step_pin, uint16_t pulse = DEFAULT_PULSE_US) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(pulse);
}

void move_joint_to(uint8_t dir_pin, uint8_t step_pin, double* actual_deg, double* target_deg) {
  const double delta = *target_deg - *actual_deg;
  // if (fabs(delta) < 1e-3) {
  //   Serial.println("Delta is too small");
  //   return;
  // }

  digitalWrite(dir_pin, (delta > 0.0) ? HIGH : LOW);
  unsigned long steps = lround(fabs(delta) * STEPS_PER_DEG);

  for (long i = 0; i < steps; ++i) {
    stepper_pulse(step_pin);
  }

  *actual_deg += (delta > 0.0) ? (steps / STEPS_PER_DEG) : -(steps / STEPS_PER_DEG);
}

// Line-reader, expects "x y\n" in mm
bool readXY(double& x, double& y) {
  char buf[64];
  size_t len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      len = 0;

      char* p = buf;
      char* end = nullptr;

      x = strtod(p, &end);
      if (end == p) {
        Serial.println(F("Parse error: x"));
        return false;
      }

      p = end;
      y = strtod(p, &end);
      if (end == p) {
        Serial.println(F("Parse error: y"));
        return false;
      }

      return true;
    }
    if (len < sizeof(buf) - 1) buf[len++] = c;
  }
  return false;
}

void parse_serial() {
  char buf[128];
  size_t len = 0;

  while(Serial.available()){
    c = (char)Serial.read();

    
  }
  return false;
}

void setup() {
  pinMode(PROXIMAL_EN, OUTPUT);
  pinMode(DISTAL_EN, OUTPUT);
  pinMode(PROXIMAL_DIR, OUTPUT);
  pinMode(DISTAL_DIR, OUTPUT);
  pinMode(PROXIMAL_STEP, OUTPUT);
  pinMode(DISTAL_STEP, OUTPUT);

  digitalWrite(PROXIMAL_EN, HIGH);
  digitalWrite(DISTAL_EN, HIGH);

  SPI.begin();

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\nuSCARA control ready. Send: \"x y\" (mm)"));

  TMC5160Stepper* motors[] = { &pM, &dM };
  for (auto* d : motors) {
    d->begin();
    d->toff(4);
    d->blank_time(24);  // ???

    d->en_pwm_mode(true);
    d->pwm_autoscale(true);
    d->pwm_autograd(true);

    d->microsteps(N_MICROSTEPS);
    d->rms_current(450);

    d->GSTAT(0b111);
  }

  digitalWrite(PROXIMAL_EN, LOW);
  digitalWrite(DISTAL_EN, LOW);
  delay(10);
}

void loop() {
  double x, y;
  if (readXY(x, y)) {
    double phi_t, relative_theta_t;
    if (!ik(x, y, &phi_t, &relative_theta_t)) {
      Serial.println(F("IK failed"));
      return;
    }

    double abs_theta_t = phi_t + relative_theta_t;

    // Move joints (absolute to target)
    move_joint_to(PROXIMAL_DIR, PROXIMAL_STEP, &phi_actual_deg, &phi_t);
    move_joint_to(DISTAL_DIR, DISTAL_STEP, &theta_actual_deg, &abs_theta_t);

    Serial.print(F("Moved to (deg): phi="));
    Serial.print(phi_actual_deg, 3);
    Serial.print(F(", theta="));
    Serial.println(theta_actual_deg, 3);
  }
}