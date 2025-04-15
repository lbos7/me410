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

#define A 0.02

#define MAX_ROT_SPEED    300 // deg/sec
#define MAX_PITCH_ANGLE  45 // deg
#define MAX_ROLL_ANGLE   45 //deg

#define JOYSTICK_TER_BUTTON key1 // B?
//gcc -o week1 week_1_student.cpp -lwiringPi  -lm

struct Joystick
{
  int key0;
  int key1;
  int key2;
  int key3;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};

int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_joystick();
void calculate_roll_acc();
void calculate_pitch_acc();
void safety_check();
void trap(int signal);


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
float time_now = 0;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float roll_acc = 0;
float pitch_acc = 0;

float roll_gyro = 0;
float pitch_gyro = 0;

bool run_program = true;

Joystick* shared_memory;
 
int main (int argc, char *argv[])
{

    setup_imu();
    setup_joystick();
    signal(SIGINT, &trap);
    calibrate_imu();    
    sleep(3);
    while(run_program)
    {
      read_imu();
      safety_check();
      // printf("%10.5f %10.5f %10.5f %10.5f %10.5f\n\r",imu_data[3],imu_data[4],imu_data[5],roll_angle,pitch_angle);
    }
  
}

void trap(int signal)

{

   printf("ending program\n\r");

   run_program=false;
}

void safety_check()
{
  // check if any gyro rate > 300 degrees/sec

  for (int i = 3; i <6; i++)
  {
    if (imu_data[i] > MAX_ROT_SPEED || imu_data[i] < -MAX_ROT_SPEED)
    {
      printf("Gyro exceeds max value!\n\r");
      trap(0);
      return;
    }
  }

  //check roll angle

  if (roll_angle > MAX_ROLL_ANGLE || roll_angle < -MAX_ROLL_ANGLE)
  {
    printf("Roll exceeds max value!\n\r");
    trap(0);
    return;
  }

  if (pitch_angle > MAX_PITCH_ANGLE || pitch_angle < -MAX_PITCH_ANGLE)
  {
    printf("Pitch exceeds max value!\n\r");
    trap(0); 
    return;
  }

  Joystick joystick_data=*shared_memory;
  if (joystick_data.JOYSTICK_TER_BUTTON)
  {
    printf("Termination requested by user\n\r");
    trap(0);
    return;
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
  roll_cal+=roll_acc;
  pitch_cal+=pitch_acc;
  accel_z_cal+=imu_data[2];
 }

  x_gyro_cal/=1000;
  y_gyro_cal/=1000;
  z_gyro_cal/=1000;
  roll_cal/=1000;
  pitch_cal/=1000;
  accel_z_cal/=1000;

  // x_gyro_calibration = x_gyro_cal;
  // y_gyro_calibration = y_gyro_cal;
  // z_gyro_calibration = z_gyro_cal;
  roll_calibration = roll_cal;
  pitch_calibration = pitch_cal;
  accel_z_calibration = accel_z_cal;

  roll_gyro = 0;
  pitch_gyro = 0;
  time_now = 0;
  
  printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void setup_joystick()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Joystick*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

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
  imu_data[5]=((vw * GYR_CONV) - z_gyro_calibration) * -1; //convert to degrees/sec

  calculate_pitch_acc();
  calculate_roll_acc();
  update_filter();
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
    wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +-3g    
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel
    
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0f, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith
    
    
    sleep(1);
  }
  return 0;
}


void calculate_roll_acc()
{
  float az = imu_data[2];
  float ax = imu_data[0];
  roll_acc = (atan2(az, ax) * 180.0/M_PI) - roll_calibration; 
  // printf("non cal = %f , cal value = %f\n\r",atan2(az, ax) * 180.0/M_PI,roll_calibration);
}

void calculate_pitch_acc()
{
  float ay = imu_data[1];
  float ax = imu_data[0];
  pitch_acc = (atan2(ay, ax) * 180.0/M_PI) - pitch_calibration; 
}  

// calculate roll and pitch with comp filter
void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev; // delta t    
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000; // delta t
  time_prev=time_curr;
  time_now += imu_diff;
  
  //comp. filter for roll, pitch here: 

  float roll_gyro_delta = imu_data[4] * imu_diff;
  float pitch_gyro_delta = imu_data[5] * imu_diff;


  roll_angle = roll_acc * A + (1-A)*(roll_gyro_delta + roll_angle);
  pitch_angle = pitch_acc * A + (1-A)*(pitch_gyro_delta + pitch_angle);

  roll_gyro = roll_gyro_delta + roll_gyro;
  pitch_gyro = pitch_gyro_delta + pitch_gyro;

  printf("%f,%f,%f,%f,%f,%f,%f\n", time_now,roll_acc,roll_gyro,roll_angle,pitch_acc,pitch_gyro,pitch_angle); // blue red yellow
}
