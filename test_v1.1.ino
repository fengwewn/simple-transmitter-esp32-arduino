// Include Wire Library for I2C
#include <Wire.h>

// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// Define servo motor connections (expand as required)
#define SER0  7   //Servo Motor 0 on connector 0
#define SER1  11   //Servo Motor 1 on connector 12


//320
#define SERVO_0_PULSE_MIN 0.001005 //500
#define SERVO_0_PULSE_MAX 0.002278 //2500
#define SERVO_0_PULSE_MID 0.0016 //2500

#define SERVO_FREQ 330 // Analog servos run at ~50 Hz updates
//#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


// Variables for Servo Motor positions (expand as required)
void arm_up();
void arm_down();
void servo_set_servo_pulse(uint8_t n, double pulse);

float master_map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {

  // Serial monitor setup
  Serial.begin(115200);

  // Print to monitor
  Serial.println("PCA9685 Servo Test");

  // Initialize PCA9685
  pca9685.begin();

  pca9685.setOscillatorFrequency(25000000);//27000000

  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(SERVO_FREQ);

}

void loop() {
  if (Serial.available()>0)
  {
    char receivedChar = Serial.read();

    if(receivedChar == '1'){
      arm_up();
    }
    else if(receivedChar == '2'){
      arm_down();
    }
  }
}

void arm_up(){
  Serial.println("Up!");
  
  float posDegrees= 0;
  float pwm0 = master_map_float(posDegrees, -90,90, SERVO_0_PULSE_MIN, SERVO_0_PULSE_MAX);
  servo_set_servo_pulse(SER0, pwm0);

  float pwm1 = master_map_float(posDegrees, -90,90, SERVO_0_PULSE_MIN, SERVO_0_PULSE_MAX);
  servo_set_servo_pulse(SER1, pwm1);
}

void arm_down(){
  Serial.println("Down!");

  float posDegrees0 = 0;
  float posDegrees1 = 0;
  
  for (int i = 0; i <= 90; i++) {    
    if (posDegrees0 <= 90 && posDegrees1 >= -90){
      posDegrees0+=2;
      posDegrees1-=2;
      float pwm0 = master_map_float(posDegrees0, -90,90, SERVO_0_PULSE_MIN, SERVO_0_PULSE_MAX);
      servo_set_servo_pulse(SER0, pwm0);
    
      //float posDegrees1= -90;
      float pwm1 = master_map_float(posDegrees1, -90,90, SERVO_0_PULSE_MIN, SERVO_0_PULSE_MAX);
      servo_set_servo_pulse(SER1, pwm1); 
      delay(50);    
    }    
  }  
}

void servo_set_servo_pulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
//  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
//  Serial.println(pulse);
  pca9685.setPWM(n, 0, pulse);
}
