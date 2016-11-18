#include<Wire.h>
#include "Arduino.h"
#include "myMPU6050.h"

volatile unsigned char myMPU6050::flag_MPU6050_INTpin = 0; //initialise the class static variable to zero

void myMPU6050::sensorProcess(float Ts)
{
  if (((accY_raw) != 0) && ((accZ_raw) != 0))
  {
    pitch_acc = atan2(accY_raw, accZ_raw) * rad_to_deg;
  }
  roll_acc = atan2(-accX_raw, sqrt((float)accY_raw * accY_raw + (float)accZ_raw * accZ_raw))*rad_to_deg;
//  roll_acc = atan2(-accX_raw, sqrt(accY_raw * accY_raw + accZ_raw * accZ_raw))*rad_to_deg;



  pitch_gyro -= ((gyroX_raw - gyroX_0Rate) / GYRO_CONVER_FACTOR) * Ts;
  roll_gyro += ((gyroY_raw - gyroY_0Rate) / GYRO_CONVER_FACTOR) * Ts;
  yaw_gyro += ((gyroZ_raw - gyroZ_0Rate) / GYRO_CONVER_FACTOR) * Ts; //+ IMU.yaw_ofset

  pitch = 0.99 * (pitch + ((gyroX_raw - gyroX_0Rate) / GYRO_CONVER_FACTOR) * Ts) + 0.01 * pitch_acc;
  roll = 0.99 * (roll + ((gyroY_raw - gyroY_0Rate) / GYRO_CONVER_FACTOR) * Ts) + 0.01 * roll_acc;

  cos_roll=cos(roll*deg_to_rad);
  cos_pitch=cos(pitch*deg_to_rad);
  
  sin_roll=sin(roll*deg_to_rad);
  sin_pitch=sin(pitch*deg_to_rad);
  
  accX=(float)accX_raw/ACCLEROMETER_SCALE_FACTOR;
  accY=(float)accY_raw/ACCLEROMETER_SCALE_FACTOR;
  accZ=(float)accZ_raw/ACCLEROMETER_SCALE_FACTOR;
  
  acc_amplitude=sqrt((float)accX*accX+(float)accY*accY+(float)accZ*accZ);
  accX_linear=accX+acc_amplitude_offset*sin_roll;
  accY_linear=accY-acc_amplitude_offset*cos_roll*sin_pitch;
  accZ_linear=accZ-acc_amplitude_offset*cos_roll*cos_pitch;
}
myMPU6050::myMPU6050()
{
    flag_MPU6050_INTpin = 0;
  attachInterrupt(digitalPinToInterrupt(2), flagINTpin, RISING);
}

void myMPU6050::flagINTpin()
{
  flag_MPU6050_INTpin = 1;
}


void myMPU6050::sensorUpdate()
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true); // request a total of 14 registers
  accX_raw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY_raw  = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ_raw  = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX_raw = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY_raw = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ_raw = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


void myMPU6050::MPU6050DataGyroGetRaw (int16_t *pui16X, int16_t *pui16Y, int16_t *pui16Z)
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_GYRO_XOUT_H);  // starting with register 0x43 (MPU6050_O_GYRO_XOUT_HH)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true); // request a total of 6 registers

  *pui16X = Wire.read() << 8 | Wire.read();
  *pui16Y = Wire.read() << 8 | Wire.read();
  *pui16Z = Wire.read() << 8 | Wire.read();
}

void myMPU6050::MPU6050DataAccelGetRaw (int16_t *pui16X, int16_t *pui16Y, int16_t *pui16Z)
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_ACCEL_XOUT_H);  // starting with register 0x3B (MPU6050_O_ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true); // request a total of 6 registers
  *pui16X = Wire.read() << 8 | Wire.read();
  *pui16Y = Wire.read() << 8 | Wire.read();
  *pui16Z = Wire.read() << 8 | Wire.read();
}

void myMPU6050::init_module()
{
  Wire.begin();
  Wire.setClock(400000L);
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_PWR_MGMT_1);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_CONFIG);  // CONFIG low pass filter
  Wire.write(MPU6050_CONFIG_DLPF_CFG_44_42);     //
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_SMPLRT_DIV);  // Sample Rate Divider
  Wire.write(0x07);  // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz => MPU6050_O_SMPLRT_DIV=7
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_GYRO_CONFIG);  //
  Wire.write(MPU6050_GYRO_CONFIG_FS_SEL_250);  //
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_ACCEL_CONFIG);  //
  Wire.write(MPU6050_ACCEL_CONFIG_AFS_SEL_2G);  //
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_O_INT_ENABLE);  // INT source, data ready register
  Wire.write(MPU6050_INT_ENABLE_DATA_RDY_EN);
  Wire.endTransmission(true);
  delay(10);
}



uint8_t myMPU6050::Calib_Gyro()
{
    int32_t sum[3] = {0, 0, 0};
    uint16_t numOfsamples = 200;
  
    int16_t buffer[3];
    int16_t max_value[3];
    int16_t min_value[3];
    uint8_t i, j;
  
    while (flag_MPU6050_INTpin == 0);
    flag_MPU6050_INTpin = 0;
    MPU6050DataGyroGetRaw (&buffer[0], &buffer[1], &buffer[2]);
  
    for (i = 0; i <= 2; i++)
    {
      max_value[i] = buffer[i];
      min_value[i] = buffer[i];
    }
  
    for (i = 1; i <= numOfsamples; i++)
    {
      while (flag_MPU6050_INTpin == 0) {};
      flag_MPU6050_INTpin = 0;
      MPU6050DataGyroGetRaw (&buffer[0], &buffer[1], &buffer[2]);
  
      for (j = 0; j <= 2; j++)
      {
        if (buffer[j] > max_value[j]) max_value[j] = buffer[j];
        else if (buffer[j] < min_value[j])  min_value[j] = buffer[j];
        if ((max_value[j] - min_value[j]) >= GYRO_NOISE_RANGE)
        {
          Serial.print(max_value[j] - min_value[j]);
          Serial.println(" :error gyro noise!");
          return 1;
        }
        // code sum
        sum[j] += buffer[j];
      }
  
    }//end of for
  
    gyroX_0Rate = (float)sum[0] / numOfsamples;
    gyroY_0Rate = (float)sum[1] / numOfsamples;
    gyroZ_0Rate = (float)sum[2] / numOfsamples;
      Serial.print(" Gyro offset: ");
      Serial.print(gyroX_0Rate);
      Serial.print(" : ");
      Serial.print(gyroY_0Rate);
      Serial.print(" : ");
      Serial.println(gyroZ_0Rate);
    return 0;
}

uint8_t myMPU6050::Calib_Accelerometer_Amplitude()
{

  char uartBuffer[10];
  float sum=0;
  uint16_t numOfsamples=200;

  float max_value;
  float min_value;
  uint8_t i;

  while(flag_MPU6050_INTpin==0);
  flag_MPU6050_INTpin=0;

  MPU6050DataAccelGetRaw(&accX_raw,&accY_raw,&accZ_raw);

  accX=(float)accX_raw/ACCLEROMETER_SCALE_FACTOR;
  accY=(float)accY_raw/ACCLEROMETER_SCALE_FACTOR;
  accZ=(float)accZ_raw/ACCLEROMETER_SCALE_FACTOR;
  acc_amplitude=sqrt((float)accX*accX+(float)accY*accY+(float)accZ*accZ);


  max_value=acc_amplitude;
  min_value=acc_amplitude;


  for (i=1; i<=numOfsamples;i++)
  {
    while(flag_MPU6050_INTpin ==  0);
    flag_MPU6050_INTpin=0;
    MPU6050DataAccelGetRaw (&accX_raw,&accY_raw,&accZ_raw);
    accX=(float)accX_raw/ACCLEROMETER_SCALE_FACTOR;
  accY=(float)accY_raw/ACCLEROMETER_SCALE_FACTOR;
  accZ=(float)accZ_raw/ACCLEROMETER_SCALE_FACTOR;
  acc_amplitude=sqrt((float)accX*accX+(float)accY*accY+(float)accZ*accZ);
  
    if(acc_amplitude>max_value) max_value=acc_amplitude;
    else if(acc_amplitude<min_value) min_value=acc_amplitude;

    if((max_value-min_value)>=ACC_NOISE_RANGE_)
    {
      Serial.print(max_value - min_value);
      Serial.println(" :error accelerometer noise!");
      return 1;
    }
    sum+=acc_amplitude;
  }//end of for

  acc_amplitude_offset=sum/numOfsamples;

  Serial.print("accleration amplitude offset: ");
  Serial.println(acc_amplitude_offset);


  return 0;
}
