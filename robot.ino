#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 accelgyro;
MPU6050 initialize;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gry_offset 0  //The offset of the gyro
#define Gyr_Gain 131
#define Angle_offset 3  // The offset of the accelerator
#define RMotor_offset 0  // The offset of the Motor
#define LMotor_offset 0  // The offset of the Motor
#define pi 3.14159

float Angle_Delta, Angle_Recursive, Angle_Confidence;

float Angle_Raw, Angle_Filtered, omega, dt;
float Turn_Speed = 0, Run_Speed = 0;
float LOutput, ROutput, Input, Output;

unsigned long preTime, lastTime;
float errSum, dErr, error, lastErr;
int timeChange;

float Sum_Right, Sum_Right_Temp, Sum_Left, Sum_Left_Temp, Distance, Distance_Right, Distance_Left, Speed;

int TN1 = 23;
int TN2 = 22;
int ENA = 5;
int TN3 = 24;
int TN4 = 25;
int ENB = 4;

void setup()
{
    Serial.begin(115200); // begin serial correspondence at highest frequency
    Wire.begin(); // starts I2C communication

    // 3rd timer setup, necessary
    TCCR3A = _BV(COM3A1) | _BV(WGM31) | _BV(WGM30); // TIMER_3 @1K Hz, fast pwm
    TCCR3B = _BV(CS31);
    TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // TIMER_0 @1K Hz, fast pwm
    TCCR0B = _BV(CS01) | _BV(CS00);

    /* If the robot was turned on with the angle over 45(-45) degrees,the wheels
     will not spin until the robot is in right position. */
    accelgyro.initialize();
    for (int i = 0; i < 200; i++)  // Looping 200 times to get the real gesture when starting
    {
        GetState();
    }
    if (abs(Angle_Filtered) < 45)  // Start to work after cleaning data
    {
        omega = Angle_Raw = Angle_Filtered = 0;
        Output = error = errSum = dErr = 0;
        GetState();
        myPID();
    }
    pinMode(TN1, OUTPUT);
    pinMode(TN2, OUTPUT);
    pinMode(TN3, OUTPUT);
    pinMode(TN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(18, INPUT);
    pinMode(2, INPUT);

    // initialize interrupts to check what they do
    attachInterrupt(4, State_A, FALLING);
    attachInterrupt(1, State_B, FALLING);
}

void loop()
{
    while (1)
    {
        GetState();
        if ((micros() - lastTime) > 10000)
        {
            // If angle > 45 or < -45 then stop the robot
            if (abs(Angle_Filtered) > 45)
            {
                digitalWrite(TN1, HIGH);
                digitalWrite(TN2, HIGH);
                digitalWrite(TN3, HIGH);
                digitalWrite(TN4, HIGH);
                Serial.println(Angle_Filtered, 4);
                Serial.println(omega, 4);
            }
            else
            {
                getAction();
                PWMControl();
            }
            lastTime = micros();
        }
    }
}

void GetState()
{
    // Raw datas
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Angle_Raw = (atan2(ay, az) * 180 / pi + Angle_offset);
    omega = gx / Gyr_Gain + Gry_offset;
    // Filter datas to get the real gesture
    unsigned long now = micros();
    timeChange = now - preTime;
    preTime = now;
    dt = timeChange * 0.000001;
    Angle_Delta = (Angle_Raw - Angle_Filtered) * 0.64;
    Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
    Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + omega;
    Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
    Serial.println(Angle_Filtered);
}

void getAction()
{

    Serial.println(Angle_Filtered, 4);
    Serial.println(omega, 4);


    // read data only when you receive data:
    if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
    }



}


void myPID()
{
    // Calculating the output values using the gesture values and the PID values.
    error = Angle_Filtered;
    errSum += error;
    dErr = error - lastErr;
    Output = kp * error + ki * errSum + kd * omega;
    lastErr = error;
    noInterrupts();
    Sum_Right = (Sum_Right + Sum_Right_Temp) / 2;
    Sum_Left = (Sum_Left + Sum_Left_Temp) / 2;
    Speed = (Sum_Right + Sum_Left) / 2;
    Distance += Speed + Run_Speed;
    Distance = constrain(Distance, -300, 300);
    Output += Speed * 70 + Distance * 0.6;
    Sum_Right_Temp = Sum_Right;
    Sum_Left_Temp = Sum_Right;
    Sum_Right = 0;
    Sum_Left = 0;
    ROutput = Output;// + Turn_Speed;
    LOutput = Output;// - Turn_Speed;
    interrupts();
}

void PWMControl()
{
    if (LOutput < 0)
    {
        digitalWrite(TN1, HIGH);
        digitalWrite(TN2, LOW);
    }
    else if (LOutput > 0)
    {
        digitalWrite(TN1, LOW);
        digitalWrite(TN2, HIGH);
    }
    else
    {
        OCR3A = 0;
    }
    if (ROutput < 0)
    {
        digitalWrite(TN3, HIGH);
        digitalWrite(TN4, LOW);
    }
    else if (ROutput > 0)
    {
        digitalWrite(TN3, LOW);
        digitalWrite(TN4, HIGH);
    }
    else
    {
        OCR0B = 0;
    }
    OCR3A = min(1023, (abs(LOutput * 4) + LMotor_offset * 4)); // Timer/Counter3 is a general purpose 16-bit Timer/Counter module
    OCR0B = min(255, (abs(ROutput) + RMotor_offset)); // Timer/Counter0 is a general purpose 8-bit Timer/Counter module
}

void State_A()
{
    if (digitalRead(18))
    {
        Sum_Right ++;
    }
    else
    {
        Sum_Right --;
    }
}

void State_B()
{
    if (!digitalRead(2))
    {
        Sum_Left ++;
    }
    else
    {
        Sum_Left --;
    }
}