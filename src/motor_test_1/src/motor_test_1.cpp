#include "motor_test_1.h"
//g++ motor_test_1.cpp -I ../../include/motor_test -std=c++11
//need sudo to run exec
//#define MOTOR_PATH "/dev/i2c-10"  //may have to do this instead of passing in!

#define MOTORS_OFF 854
#define MOTORS_FULL 1550

motor::motor(uint8_t motor_id, uint8_t i2c_handle, uint8_t i2c_address)
{
    printf("Motor object is being created, motor = %i \n", motor_id);
    motorId_ = motor_id;
    i2cHandle_ = i2c_handle;
    i2cAddress_ = i2c_address;
    send_force_i2c(0,MOTORS_OFF);
    
}

void motor::write8(uint8_t addr, uint8_t d, uint8_t handle)
{

          uint8_t data[2];
          data[0] = addr;
          data[1] = d;
          if(write(handle,data,2)!= 2)     printf("error using write8 \n");
          else                             printf("success using write8 \n");

}

uint8_t motor::read8(uint8_t addr, uint8_t handle)
{
          uint8_t buf[2];
          buf[1] = addr;
          if(write(handle,buf,1) != 1) printf("error when trying to set read register");

          if(read(handle,buf,1) != 1)  printf("failed to read from i2c bus");
          return buf[1];
}

uint8_t motor::open_motors_i2c(std::string PATH2MOTOR, uint8_t addr, float freq)
{
        printf("opening i2c port...");
        uint8_t handle = open(PATH2MOTOR.c_str(),O_RDWR);

        if (handle > 0) printf("Done!: file descriptor: %i \n",handle);
        else            printf("Fail to open port \n");
	ioctl(handle, I2C_SLAVE, addr); // set slave address    

        //write8(PCA9685_MODE1, 0x0); // this is reset in motordriver.cpp

        uint8_t data[2];
        data[0] = PCA9685_MODE1;
        data[1] = 0x0;
        if(write(handle,data,2)!= 2) printf("er:or in resetting chip \n");
        else                                printf("successi in ressetting chip \n");

        freq *= 0.9;  // Correct overshoot in the frequency setting (see issue #11).
        float prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        uint8_t prescale = floor(prescaleval + 0.5);

        uint8_t oldmode = read8(PCA9685_MODE1, handle);
        uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
        motor::write8(PCA9685_MODE1,     newmode,     handle); // go to sleep
        motor::write8(PCA9685_PRESCALE, prescale,    handle); // set the prescaler
        motor::write8(PCA9685_MODE1,    oldmode,     handle);
        usleep(5000);
        motor::write8(PCA9685_MODE1, oldmode | 0xa1, handle);

        return handle;
}
void motor::send_force_i2c(uint16_t on, uint16_t off) {

        //NEED TO USE IDENTIFIER FOR THE MOTOR: channel id's the motor 0-3
        uint8_t buf[5];
        buf[0] = LED0_ON_L + 4*motorId_;
        buf[1] = on;
        buf[2] = on >> 8;
        buf[3] = off;
        buf[4] = off >> 8;
        write(i2cHandle_,buf,5);
	if(first_call)
    	{
        	first_call = false;
        	clock_gettime(CLOCK_REALTIME,&oldT);
   	}
	calcDt(oldT,newT);
}

int motor::send_motor_data(uint16_t on, uint16_t off)
{
    // select returns the number of fd's ready
    FD_ZERO(&write_fds);
    FD_SET(i2cHandle_, &write_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;

   int num_fds = select(i2cHandle_+1, NULL, &write_fds, NULL, &no_timeout);
   int returnval = 0;

   //printf("returnval in select: %i \n\n\n", num_fds);

 //No data ready to read 
        if(num_fds == 0)
        {
                returnval= -1;
		printf("No File_Descriptor Available to write! \n");
                return -1;
        }
        //select returned an error 
        else if(num_fds ==-1)
        {
                returnval= -2;
                return -2;
        }
        else if(num_fds ==1)
        {
                //channel/motor_id tells us which motor should get signal. On/Off is the pwm frequency. Need to map this from origianl (0-255) to new (700-???)
	  // printf("file descriptor ready to write! \n");
		send_force_i2c(on,off);//on, off);
                returnval = num_fds;
        }


        return returnval;
}
void motor::set_force( int force_in, bool controller_run )
{
        //when setting force, check that ...
        //the motors are allowed to run (controllER_RUN flag is true)
        //the force is within acceptable bounds

  if(controller_run) force_ = force_in; //ensure_valid_force(force_in) ;
        else force_ = 0;
}
int motor::get_force( void )
{
	return force_;
}

/*int motor::ensure_valid_force(int force_in)
{
        //check if requested force of this motor is in the acceptable bounds
        // if not cap it at the max/min
        if(force_in > max_force) {return max_force;}
        if(force_in < min_force) {return min_force;}
        return force_in;
	}*/


//timer functions 
float motor::calcDt(timespec& oldT, timespec& newT)
{
                //track dt between reads
                clock_gettime(CLOCK_REALTIME, &newT);
                float dt = UTILITY::timespec2float(UTILITY::time_diff(oldT, newT));
                clock_gettime(CLOCK_REALTIME ,&oldT);
		calc_dt = dt;
                return dt;
}

float motor::getDt(void)
{
                return this->calc_dt;
}

float motor::timeSinceLastRead(void)
{
		
		timespec currentTime;
                clock_gettime(CLOCK_REALTIME,&currentTime);
                //printf("xbee time %f \n",UTILITY::timespec2float(UTILITY::time_diff(this->oldT, currentTime)));
                return UTILITY::timespec2float(UTILITY::time_diff(this->oldT, currentTime));

}

//map values from 0-255 to 900-1550 returns a uint16_t to be passed to sned_motor_data parameter "on"

uint16_t motor::map_values(uint16_t force){
  return (887 + ((1550-887)*force)/255);
}


int main(void)
{
  std::string PATH2MOTOR = "/dev/i2c-1";
  uint8_t handle;
  handle =  motor::open_motors_i2c(PATH2MOTOR, 0x40, 200);
  motor motor0 = motor(0,handle,0x40);
  motor motor1 = motor(1,handle,0x40);
  motor motor2 = motor(2,handle,0x40);
  motor motor3 = motor(3,handle,0x40);
  int speed0;

  timespec oldT, newT;
  float loop_dt;
  while(1)
{
	  //cout << "enter speed for motor2" << endl;
	  //cin >> speed0;
	  //motor1.send_motor_data();
	  //motor1.send_force_i2c(0,700); //speed0);
	  //cout << "frequency: " << 1/motor1.getDt()  << "time since last read: " << motor1.timeSinceLastRead( )<< endl;
	  clock_gettime(CLOCK_REALTIME,&oldT);
	  int returnVal;
	  uint16_t m0Val = motor::map_values(0);

	  returnVal =  motor0.send_motor_data(0,m0Val);
	  returnVal =  motor1.send_motor_data(0,705);
	  returnVal =  motor2.send_motor_data(0,705);
	  returnVal =  motor3.send_motor_data(0,705);
	  // motor1.send_force_i2c(0,700);
	  // motor2.send_force_i2c(0,700);
	  // motor3.send_force_i2c(0,700);
	  
	  
	  clock_gettime(CLOCK_REALTIME,&newT);
	  //int returnVal = 1;
	  loop_dt = UTILITY::calcDt(oldT, newT);

	  //printf("returnVal: %i, loop frequency: %f, read frequency: %f, time since last read: %f  \n\n", returnVal, 1/loop_dt,1/motor1.getDt(), motor1.timeSinceLastRead());
	  if(returnVal == 0){
	    printf("not able to write");
	    break;
	  }
	

 }
 
  
  

}
