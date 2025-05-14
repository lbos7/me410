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

#include <sched.h>

#define ACC_CONV 6.0/65535
#define GYR_CONV 2000.0/65535

#define A 0.02

#define MAX_ROT_SPEED    300 // deg/sec
#define MAX_PITCH_ANGLE  45 // deg
#define MAX_ROLL_ANGLE   45 //deg

#define JOYSTICK_TER_BUTTON key1 // B
#define JOYSTICK_PAUSE_BUTTON key2
#define JOYSTICK_RESUME_BUTTON key0
//gcc -o week1 week_1_student.cpp -lwiringPi -lm

// Pitch PID 

#define P_PITCH 10.0
#define D_PITCH 2.0
#define I_PITCH 0.1

#define PITCH_ERR_CLIP 1.0

// Roll PID 

#define P_ROLL 12.0
#define D_ROLL 2.5
#define I_ROLL 0.1

// Yaw P

#define P_YAW 5.5

#define I_SAT 100.0

#define THRUST_NEUTRAL 1450.0
#define THRUST_AMP     250.0

#define PITCH_AMP 8.0
#define ROLL_AMP 8.0
#define YAW_AMP 70.0

// 0 = motor 4
// 1 = motor 3
// 2 = motor 2
// 3 = motor 1

#define MOTOR_1 motor_commands[0]
#define MOTOR_2 motor_commands[1]
#define MOTOR_3 motor_commands[2]
#define MOTOR_4 motor_commands[3]

#define MOTOR_LIMIT 2000

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
void pitchController();
void rollController();
void PIDcontroller();
void set_motors(int motor0, int motor1, int motor2, int motor3);
void motor_enable();
void checkMotorPause();



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
long joystickWatchdogTimer = 0;
float time_now = 0;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float roll_acc = 0;
float pitch_acc = 0;
float desired_pitch = 0.0;
float desired_roll = 0.0;
float desired_yaw_speed = 0.0;

int motor_address=wiringPiI2CSetup (0x56); 

bool run_program = true;

Joystick* shared_memory;

int sequence_num = 0;


int motor_commands[4];

float int_pitch = 0;
float int_roll = 0;

float runMotor = 0.0;

int main (int argc, char *argv[])
{

    setup_imu();
    setup_joystick();
    signal(SIGINT, &trap);
    calibrate_imu();    
    sleep(3);
    timespec_get(&te,TIME_UTC);
    joystickWatchdogTimer=te.tv_nsec;

    motor_enable();

    while(run_program)
    {
        read_imu();
        safety_check();
        PIDcontroller();
        checkMotorPause();
        printf("%f,%f,%f,%f,%f,%d,%d,%d,%d\n", time_now, pitch_angle, desired_pitch, roll_angle, desired_roll, motor_commands[0],motor_commands[1],motor_commands[2],motor_commands[3]); // blue red yellow
        // printf("%f\n",roll_angle);

    }
  
}

void trap(int signal)

{

   printf("ending program\n\r");

   // turn off motors

   set_motors(0,0,0,0);

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
      set_motors(0,0,0,0);
      return;
    }
  }

  //check roll angle

  if (roll_angle > MAX_ROLL_ANGLE || roll_angle < -MAX_ROLL_ANGLE)
  {
    printf("Roll exceeds max value!\n\r");
    trap(0);
    set_motors(0,0,0,0);
    return;
  }

  if (pitch_angle > MAX_PITCH_ANGLE || pitch_angle < -MAX_PITCH_ANGLE)
  {
    printf("Pitch exceeds max value!\n\r");
    trap(0); 
    set_motors(0,0,0,0);
    return;
  }

  Joystick joystick_data=*shared_memory;
  if (joystick_data.JOYSTICK_TER_BUTTON)
  {
    printf("Termination requested by user\n\r");
    trap(0);
    set_motors(0,0,0,0);
    return;
  }

  // check for sequence_num timeout
  time_curr = te.tv_nsec;

  // Check if the sequence number is unchanged
  if (joystick_data.sequence_num == sequence_num)
  {
      float joystick_diff = time_curr - joystickWatchdogTimer;
  
      // Handle nanosecond rollover at 1 second
      if (joystick_diff <= 0)
      {
          joystick_diff += 1000000000;
      }
  
      if (joystick_diff > 500000000)
      {
          printf("Joystick timeout!\n\r");
          trap(0);
          set_motors(0,0,0,0);
          return;
      }
  }
  else
  {
      // New joystick message received, so reset timer and sequence_num
      joystickWatchdogTimer = time_curr;
      sequence_num = joystick_data.sequence_num;
  }
}

void checkMotorPause()
{
  if (runMotor == 0.0)
  {
    set_motors(2,2,2,2);
  }
  Joystick joystick_data=*shared_memory;
  if (joystick_data.JOYSTICK_PAUSE_BUTTON)
  {
    printf("Pause requested by user\n\r");
    runMotor = 0.0;
    set_motors(2,2,2,2);
    return;
  }
  if (joystick_data.JOYSTICK_RESUME_BUTTON)
  {
    printf("Resume requested by user\n\r");
    runMotor = 1.0;
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

  // Gyro calibration commented out for plotting
  x_gyro_calibration = x_gyro_cal;
  y_gyro_calibration = y_gyro_cal;
  z_gyro_calibration = z_gyro_cal;
  roll_calibration = roll_cal;
  pitch_calibration = pitch_cal;
  accel_z_calibration = accel_z_cal;

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
  accel_address=wiringPiI2CSetup(0x19); 
  
  gyro_address=wiringPiI2CSetup(0x69);
  
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

  // Calculating pitch and roll based on gyro data for plot
  float roll_gyro_delta = imu_data[4] * imu_diff;
  float pitch_gyro_delta = imu_data[5] * imu_diff;

  // Complementary filter
  roll_angle = roll_acc * A + (1-A)*(roll_gyro_delta + roll_angle);
  pitch_angle = pitch_acc * A + (1-A)*(pitch_gyro_delta + pitch_angle);

}

void pitchController()
{
    float pitch_mul = (128 - shared_memory->pitch)/127.0;
    float thrust_mul = (128 - shared_memory->thrust)/127.0;

    float desired_thrust = THRUST_NEUTRAL + thrust_mul*THRUST_AMP;
    desired_pitch = -pitch_mul*PITCH_AMP;

    // printf("%f\n",desired_pitch);

    float pitch_err = desired_pitch - pitch_angle; 

    int_pitch += I_PITCH*pitch_err;

    if (int_pitch > I_SAT)
    {
      int_pitch = I_SAT;
    }
    if (int_pitch < -I_SAT)
    {
      int_pitch = -I_SAT;
    }

    motor_commands[0] = desired_thrust + P_PITCH*pitch_err - D_PITCH*imu_data[5] + int_pitch;
    motor_commands[2] = desired_thrust + P_PITCH*pitch_err - D_PITCH*imu_data[5] + int_pitch;

    motor_commands[1] = desired_thrust - P_PITCH*pitch_err + D_PITCH*imu_data[5] - int_pitch;
    motor_commands[3] = desired_thrust - P_PITCH*pitch_err + D_PITCH*imu_data[5] - int_pitch;

    for (int i =0; i<4; i++)
    {
      if (motor_commands[i] < 0)
      {
        motor_commands[i] = 0;
        continue;
      }

      if (motor_commands[i] > MOTOR_LIMIT)
      {
        motor_commands[i] = MOTOR_LIMIT;
        continue;
      }
    }

    // print for plots
    
    // printf("%f,%f,%f,%f,%d,%d\n",time_now , pitch_angle*10,desired_pitch*10,desired_thrust,motor_commands[0],motor_commands[1]);
    // printf("pitch mul : %f , thrust_mul: %f\n",pitch_mul,thrust_mul);
    // 0 = motor 4
    // 1 = motor 3
    // 2 = motor 2
    // 3 = motor 1
    set_motors(MOTOR_4,MOTOR_3,MOTOR_2,MOTOR_1);

}

void rollController()
{
    float roll_mul = (128 - shared_memory->roll)/127.0;
    float thrust_mul = (128 - shared_memory->thrust)/127.0;

    float desired_thrust = THRUST_NEUTRAL + thrust_mul*THRUST_AMP;
    desired_roll= -roll_mul*ROLL_AMP;

    float roll_err = desired_roll - roll_angle; 

    int_roll += I_ROLL*roll_err;

    if (int_roll > I_SAT)
    {
      int_roll = I_SAT;
    }
    if (int_roll < -I_SAT)
    {
      int_roll = -I_SAT;
    }

    motor_commands[0] = desired_thrust + P_ROLL*roll_err - D_ROLL*imu_data[4] + int_roll;
    motor_commands[1] = desired_thrust + P_ROLL*roll_err - D_ROLL*imu_data[4] + int_roll;

    motor_commands[2] = desired_thrust - P_ROLL*roll_err + D_ROLL*imu_data[4] - int_roll;
    motor_commands[3] = desired_thrust - P_ROLL*roll_err + D_ROLL*imu_data[4] - int_roll;

    for (int i =0; i<4; i++)
    {
      if (motor_commands[i] < 0)
      {
        motor_commands[i] = 0;
        continue;
      }

      if (motor_commands[i] > MOTOR_LIMIT)
      {
        motor_commands[i] = MOTOR_LIMIT;
        continue;
      }

    }

    // print for plots
    
    // printf("%f,%f,%f,%f,%d,%d\n",time_now , pitch_angle*10,desired_pitch*10,desired_thrust,motor_commands[0],motor_commands[1]);
    // printf("pitch mul : %f , thrust_mul: %f\n",pitch_mul,thrust_mul);
    // 0 = motor 4
    set_motors(MOTOR_4,MOTOR_3,MOTOR_2,MOTOR_1);

}

void PIDcontroller()
{
    if (runMotor == 0.0)
    {
      return;
    }
    float pitch_mul = (128 - shared_memory->pitch)/127.0;
    float roll_mul = (128 - shared_memory->roll)/127.0;
    float yaw_mul = (128 - shared_memory->yaw)/127.0;
    float thrust_mul = (128 - shared_memory->thrust)/127.0;

    float desired_thrust = THRUST_NEUTRAL + thrust_mul*THRUST_AMP;

    desired_pitch = -pitch_mul*PITCH_AMP;

    // printf("%f\n",desired_pitch);

    float pitch_err = desired_pitch - pitch_angle; 

    if (pitch_err <= PITCH_ERR_CLIP && pitch_err >= -PITCH_ERR_CLIP)
    {
      pitch_err = 0.0;
    }

    int_pitch += I_PITCH*pitch_err;

    if (int_pitch > I_SAT)
    {
      int_pitch = I_SAT;
    }
    if (int_pitch < -I_SAT)
    {
      int_pitch = -I_SAT;
    }

    desired_roll= -roll_mul*ROLL_AMP;

    float roll_err = desired_roll - roll_angle; 

    int_roll += I_ROLL*roll_err;

    if (int_roll > I_SAT)
    {
      int_roll = I_SAT;
    }
    if (int_roll < -I_SAT)
    {
      int_roll = -I_SAT;
    }

    desired_yaw_speed =  yaw_mul*YAW_AMP;
    float yaw_err = desired_yaw_speed- imu_data[3];

    motor_commands[0] = desired_thrust + P_PITCH*pitch_err - D_PITCH*imu_data[5] + int_pitch  + P_ROLL*roll_err - D_ROLL*imu_data[4] + int_roll -P_YAW*yaw_err;
    motor_commands[1] = desired_thrust - P_PITCH*pitch_err + D_PITCH*imu_data[5] - int_pitch  + P_ROLL*roll_err - D_ROLL*imu_data[4] + int_roll+P_YAW*yaw_err;
    motor_commands[2] = desired_thrust + P_PITCH*pitch_err - D_PITCH*imu_data[5] + int_pitch - P_ROLL*roll_err + D_ROLL*imu_data[4] - int_roll+P_YAW*yaw_err;
    motor_commands[3] = desired_thrust - P_PITCH*pitch_err + D_PITCH*imu_data[5] - int_pitch - P_ROLL*roll_err + D_ROLL*imu_data[4] - int_roll-P_YAW*yaw_err;

    for (int i =0; i<4; i++)
    {
      if (motor_commands[i] < 0)
      {
        motor_commands[i] = 0;
        continue;
      }

      if (motor_commands[i] > MOTOR_LIMIT)
      {
        motor_commands[i] = MOTOR_LIMIT;
        continue;
      }
    }

    set_motors(MOTOR_4,MOTOR_3,MOTOR_2,MOTOR_1);
    // set_motors(2,2,2,2);

}
 

void motor_enable()
{
  
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
    int cal_delay=50;
    
    for(int i=0;i<1000;i++)
    {
    
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }
     
    for(int i=0;i<2000;i++)
    {
    
      motor_id=0;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }
    
     
    for(int i=0;i<500;i++)
    {
    
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }

}


void set_motors(int motor0, int motor1, int motor2, int motor3)
{

    if(motor0<0)
      motor0=0;
    if(motor0>2000)
      motor0=2000;
    if(motor1<0)
      motor1=0;
    if(motor1>2000)
      motor1=2000;
    if(motor2<0)
      motor2=0;
    if(motor2>2000)
      motor2=2000;
    if(motor3<0)
      motor3=0;
    if(motor3>2000)
      motor3=2000;
      
    
    
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
   // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
    //wiringPiI2CWrite (motor_address,data[0]) ;
    int com_delay=500;
   
    motor_id=0;
    commanded_speed=motor0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);  
 
    
    usleep(com_delay);   
    motor_id=1;
    commanded_speed=motor1;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);      
  
    usleep(com_delay); 
    motor_id=2;
    commanded_speed=motor2;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);   

    
    usleep(com_delay);   
    motor_id=3;
    commanded_speed=motor3;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);    
    usleep(com_delay);


}


