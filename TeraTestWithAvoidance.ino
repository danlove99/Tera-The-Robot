#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <NewPing.h>

#define SONAR_NUM     1 // No of sensors - change this if you want to add more sensors.
#define MAX_DISTANCE 500 // Maxi distance (in cm) to ping
#define PING_INTERVAL 250 // Ms ping-ping

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM];
uint8_t currentSensor = 0;

NewPing sonar[SONAR_NUM] = {
  NewPing(8, 9, MAX_DISTANCE), // Trigger pin, echo pin, and max. Copy & paste this line (changing trig & echo pins) for each sensor you physically use.
};

int IN1 = 5;
int IN2 = 4;
int IN3 = 6;
int IN4 = 7;

int Base = 0;
int Head = 3;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the start time for each sensor
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);



#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

  uint8_t servonum = 0;


  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}
int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}




void loop() {

  headSpan();

  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all sensors
    if (millis() >= pingTimer[i]) {         // Check each sensor for time to ping
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set time for next ping.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = MAX_DISTANCE;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
 // for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.

    if (cm[0] < 20)  // something seen by right US
    {
      Backward();
      pwm.setPWM(Base, 0, pulseWidth(110));
      delay(1000);
      Right();
      pwm.setPWM(Base, 0, pulseWidth(90));
      delay(2000);


    }


    else; // if there's nothing in front of us
    {
      Forward(); // drive forward
    }



  }

  void echoCheck() {
    if (sonar[currentSensor].check_timer())
      cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }

  void oneSensorCycle() {
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      Serial.print(i);
      Serial.print("=");
      Serial.print(cm[i]);
      Serial.print("cm ");
    }
    Serial.println();
  }

  void headSpan() {
    for (int x = 0; x < 180; x++) {
      pwm.setPWM(Head, 0, pulseWidth(x));
      delay(4);
    }
    for (int x = 180; x > 0; x--) {
      pwm.setPWM(Head, 0, pulseWidth(x));
      delay(4);
    }
    for (int x = 0; x < 90; x++) {
      pwm.setPWM(Head, 0, pulseWidth(x));
      delay(4);
    }
    delay(3000);
  }






  void Left()
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);


    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);


  }

  void Right()
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);


    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

  }

  void Stop()
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);


    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);


  }

  void Forward()
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);


    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

  }

  void Backward()
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);


    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

  }
