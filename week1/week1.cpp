#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>

#define ACC_CONV 6.0/65535
#define GYR_CONV 2000.0/65535

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm


int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void calculate_roll();
void calculate_pitch();

//global variables
int accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz, 
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;

 
int main (int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();    
    sleep(3);
    while(1)
    {
      read_imu();    
      printf("%10.5f %10.5f %10.5f %10.5f %10.5f\n\r",imu_data[3],imu_data[4],imu_data[5],roll_angle,pitch_angle);
      usleep(5000);
    }
  
}

void calibrate_imu()
{
 float x_gyro_cal=0;
 float y_gyro_cal=0;
 float z_gyro_cal=0;
 float roll_cal=0;
 float pitch_cal=0;
 float accel_z_cal=0;

 for (int i = 0; i<1000; i++) {
  read_imu();
  x_gyro_cal+=imu_data[3];
  y_gyro_cal+=imu_data[4];
  z_gyro_cal+=imu_data[5];
  roll_cal+=roll_angle;
  pitch_cal+=pitch_angle;
  accel_z_cal+=imu_data[2];
 }

  x_gyro_cal/=1000;
  y_gyro_cal/=1000;
  z_gyro_cal/=1000;
  roll_cal/=1000;
  pitch_cal/=1000;
  accel_z_cal/=1000;

  x_gyro_calibration = x_gyro_cal;
  y_gyro_calibration = y_gyro_cal;
  z_gyro_calibration = z_gyro_cal;
  roll_calibration = roll_cal;
  pitch_calibration = pitch_cal;
  accel_z_calibration = accel_z_cal;

  printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  uint8_t address=0;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh=0;
  int vl=0;
  int vw=0;


  //accel reads

  address=0x12;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);    
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]= vw * ACC_CONV;//convert to g's  
  
  address=0x14;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]=vw * ACC_CONV;//convert to g's  
  
  address=0x16;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=vw * ACC_CONV;//convert to g's  
  
  
     

  //gyro reads

  address=0x02;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=(vw * GYR_CONV) - x_gyro_calibration;//convert to degrees/sec
  
  address=0x04;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);    
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=(vw * GYR_CONV) - y_gyro_calibration;//convert to degrees/sec
  
  address=0x06;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=((vw * GYR_CONV) - z_gyro_calibration) * -1;//convert to degrees/sec  

  calculate_pitch();
  calculate_roll();
}


int setup_imu()
{
  wiringPiSetup ();
  
  //setup imu on I2C
  accel_address=wiringPiI2CSetup (0x19) ; 
  
  
  gyro_address=wiringPiI2CSetup (0x69) ; 
  
  if(accel_address==-1)
  {
    printf("-----cant connect to accel I2C device %d --------\n",accel_address);
    return -1;
  }
  else if(gyro_address==-1)
  {
    printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
    return -1;
  }
  else
  {
    printf("all i2c devices detected\n");
    sleep(1);
    wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel    
    wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g    
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel
    
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0f, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith
    
    
    sleep(1);
  }
  return 0;
}

void calculate_roll()
{
  float az = imu_data[2];
  float ax = imu_data[0];
  roll_angle = (atan2(az, ax) * 180.0/M_PI) - roll_calibration; 
}

void calculate_pitch()
{
  float ay = imu_data[1];
  float ax = imu_data[0];
  pitch_angle = (atan2(ay, ax) * 180.0/M_PI) - pitch_calibration; 
}  
