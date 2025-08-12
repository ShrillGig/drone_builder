#include <Wire.h>
#include <Servo.h>

float  RatePitch;
float  RateCalibrationPitch;
int RateCalibrationNumber;

float  PrevErrorRatePitch;
float  PrevItermRatePitch;
float  DesiredRatePitch;
float ErrorRatePitch;

float InputPitch;
float AccX, AccY, AccZ;

float AnglePitch;
uint32_t LoopTimer;

float KalmanAnglePitch=0, 
 KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
float PIDReturn[]={0, 0, 0};

//Internal PID coefficients
 float PRatePitch=0.6; 
 float IRatePitch=1.0;
 float DRatePitch=0.03;

//External PID coefficients
 float PAnglePitch=0.15;
 float IAnglePitch=0.0;
 float DAnglePitch=0.0;

 int print_div = 0;
 uint32_t start_ms = 0;

float DesiredAnglePitch;
float ErrorAnglePitch;
float PrevErrorAnglePitch;
float PrevItermAnglePitch;

 int MIN = 1000;
 int MAX = 2000;
 int CURRENT = 1400;
 int LEFT_THROTTLE, RIGHT_THROTTLE;

Servo left_motor, right_motor;

void kalman_filter(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
 KalmanState=KalmanState+0.004*KalmanInput;
 KalmanUncertainty=KalmanUncertainty + 0.004 
 * 0.004 * 4 * 4;
 float KalmanGain=KalmanUncertainty * 1/
 (1*KalmanUncertainty + 2 * 2);
 KalmanState=KalmanState+KalmanGain * (
 KalmanMeasurement-KalmanState);
 KalmanUncertainty=(1-KalmanGain) * 
 KalmanUncertainty;
 Kalman1DOutput[0]=KalmanState; 
 Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_acc_data(void) {
 Wire.beginTransmission(0x68);
 Wire.write(0x1A);
 Wire.write(0x05);
 Wire.endTransmission();
 Wire.beginTransmission(0x68);
 Wire.write(0x1C);
 Wire.write(0x10);
 Wire.endTransmission();
 Wire.beginTransmission(0x68);
 Wire.write(0x3B);
 Wire.endTransmission(); 
 Wire.requestFrom(0x68,6);

 int16_t AccXLSB = Wire.read() << 8 | 
 Wire.read();
 int16_t AccYLSB = Wire.read() << 8 |
 Wire.read();
 int16_t AccZLSB = Wire.read() << 8 | 

 Wire.read();
 Wire.beginTransmission(0x68);
 Wire.write(0x1B); 
 Wire.write(0x8);
 Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
 Wire.write(0x43);
 Wire.endTransmission();
 Wire.requestFrom(0x68,6);

 int16_t GyroX=Wire.read()<<8 | Wire.read();
 int16_t GyroY=Wire.read()<<8 | Wire.read();
 int16_t GyroZ=Wire.read()<<8 | Wire.read();

 RatePitch=-(float)GyroY/65.5; // Y rate is inverted

 AccX=(float)AccXLSB/4096-0.02;
 AccY=-(float)AccYLSB/4096+0.02; //Y accelearation is inverted
 AccZ=-(float)AccZLSB/4096-0.02; //Z accelearation is inverted


 AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*
 AccZ))*1/(3.142/180);
}

void pid_regulator(float Error, float P , float I, float D,   
    float PrevError, float PrevIterm) {
 float Pterm=P*Error;
 float Iterm=PrevIterm+I*(Error+
 PrevError)*0.004/2;
 if (Iterm > 400) Iterm=400;
 else if (Iterm <-400) Iterm=-400;
 float Dterm=D*(Error-PrevError)/0.004;
 float PIDOutput= Pterm+Iterm+Dterm;
 if (PIDOutput>400) PIDOutput=400;
 else if (PIDOutput <-400) PIDOutput=-400;
 PIDReturn[0]=PIDOutput;
 PIDReturn[1]=Error;
 PIDReturn[2]=Iterm;
 }

 void reset_pid(void) {
 PrevErrorRatePitch=0; 
 PrevItermRatePitch=0; 
 PrevErrorAnglePitch=0; 
 PrevItermAnglePitch=0;

 }


void setup() {

 Serial.begin(57600);
 pinMode(13, OUTPUT);
 digitalWrite(13, HIGH);
 Wire.setClock(400000);
  Wire.begin();
 delay(250);
 Wire.beginTransmission(0x68); 
 Wire.write(0x6B);
 Wire.write(0x00);
 Wire.endTransmission();

//Gyroscope calibration
 for (RateCalibrationNumber=0; 
 RateCalibrationNumber<2000;
 RateCalibrationNumber ++) {
 gyro_acc_data();
 RateCalibrationPitch+=RatePitch;
 delay(1);
 }
 RateCalibrationPitch/=2000;
Serial.println("\nGyro Calibration is done!");


left_motor.attach(9);
right_motor.attach(10);


//System launch and ESCs calibration
Serial.print("\nPress 1 if you want to start");
  while(true) {

    char one = Serial.read();

    if(one == '1') {
      Serial.println("\nStarting the calibration");
       left_motor.writeMicroseconds(MAX);
       right_motor.writeMicroseconds(MAX);
       Serial.println("MAX is done");
       delay(7000);

      left_motor.writeMicroseconds(MIN);
      right_motor.writeMicroseconds(MIN);
      Serial.println("MIN is done");
      delay(5000);

      //Checking if motors spin correctly
      for(int current_value = MIN + 100; current_value < 1150; current_value ++) {

        left_motor.writeMicroseconds(current_value);
        right_motor.writeMicroseconds(current_value);
        Serial.print("Throttle");
        Serial.println(current_value);
        delay(50);

  }

        left_motor.writeMicroseconds(MIN);
        right_motor.writeMicroseconds(MIN);
        Serial.println("Calibration done!");

        delay(3000);
        break;
    }  
  }

 start_ms = millis();
 LoopTimer=micros();
}
void loop() {
 gyro_acc_data();

 RatePitch-=RateCalibrationPitch;

 kalman_filter(KalmanAnglePitch, 
   KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
 KalmanAnglePitch=Kalman1DOutput[0]; 
   KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

//External PID regulator (Kalmanangle)
ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch; 
 pid_regulator(ErrorAnglePitch, PAnglePitch, 
 IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
 DesiredRatePitch=PIDReturn[0]; 
 PrevErrorAnglePitch=PIDReturn[1];
 PrevItermAnglePitch=PIDReturn[2];

//Internal PID regulator (rate)
ErrorRatePitch=DesiredRatePitch-RatePitch;
pid_regulator(ErrorRatePitch, PRatePitch,
 IRatePitch, DRatePitch, PrevErrorRatePitch,
 PrevItermRatePitch);
 InputPitch=PIDReturn[0]; 
 PrevErrorRatePitch=PIDReturn[1]; 
 PrevItermRatePitch=PIDReturn[2];


//Stop the system 
 if (Serial.read() == '0') {

    left_motor.writeMicroseconds(MIN);
    right_motor.writeMicroseconds(MIN);
    Serial.println("Motor has been stopped");
    while(1);
  }


 LEFT_THROTTLE = CURRENT + InputPitch;
 RIGHT_THROTTLE = CURRENT - InputPitch;

//Left and right motor max&min control 
 if (LEFT_THROTTLE > 1800) LEFT_THROTTLE = 1800;
 if(LEFT_THROTTLE < 1100) LEFT_THROTTLE = 1100;
 if(RIGHT_THROTTLE > 1800) RIGHT_THROTTLE = 1800;
 if(RIGHT_THROTTLE < 1100) RIGHT_THROTTLE = 1100;

 left_motor.writeMicroseconds(LEFT_THROTTLE);
 right_motor.writeMicroseconds(RIGHT_THROTTLE);


//Getting data on Serial every 10HZ
if (++print_div >= 25) { 
    print_div = 0;

  uint32_t t_ms = millis() - start_ms;
  Serial.print(t_ms); Serial.print(',');
  Serial.print(KalmanAnglePitch, 3); Serial.print(',');
  Serial.print(RatePitch, 3); Serial.print(',');
  Serial.print(DesiredRatePitch, 3); Serial.print(',');
  Serial.print(InputPitch, 1); Serial.print(',');
  Serial.print(LEFT_THROTTLE); Serial.print(',');
  Serial.println(RIGHT_THROTTLE);

}

 while (micros() - LoopTimer < 4000);
 LoopTimer=micros();
}
