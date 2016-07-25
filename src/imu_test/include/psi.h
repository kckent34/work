//=================================
// include guard
#ifndef PSI
#define PSI

using namespace std;

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <string.h>

class Psi {
        bool first_loop;
        int num_psi_rotations;
        float old_raw_psi;
        float new_raw_psi;
        float dt;
  public:


Psi(void)
{
        first_loop = true;
        num_psi_rotations = 0;
        old_raw_psi = 0.0;
        new_raw_psi = 0.0;
}

Psi(float dt)
{
        first_loop = true;
        num_psi_rotations = 0;
        old_raw_psi = 0.0;
        new_raw_psi = 0.0;
        (this -> dt) = dt;
}
float integ_gyro(float psi_dot)
{
        this->old_raw_psi = (old_raw_psi + psi_dot * dt);
        return old_raw_psi;
}
//new_raw_psi is the raw psi out of the imu before calibration and with no continuous
float make_contin(float new_raw_psi)
{
        //if this is the first loop, store this value for next time
        if(first_loop==true)
                {
                        old_raw_psi = new_raw_psi;
                        first_loop = false;
                        //printf("first loop psi %f !!!!!!!!!!!!!!!!!!!\n\n\n\n", new_raw_psi);
                }

        //Check if psi is decreasing toward 0
        if( (old_raw_psi <= 10.0) && (old_raw_psi >= 0.0)  && (new_raw_psi <= 360.0) && (new_raw_psi >= 350.0))
    {
            num_psi_rotations--;
            //printf("num_psi_rotations--");
    }

        //Else Check if psi is increasing toward 360
        else if( (new_raw_psi <= 10.0) && (new_raw_psi >= 0.0) && (old_raw_psi <= 360.0) && (old_raw_psi >= 350.0))
    {
            num_psi_rotations++;
            //printf("num_psi_rotations++");
    }


        float psi_to_return  = num_psi_rotations*360 + new_raw_psi;

        old_raw_psi = new_raw_psi;

        return psi_to_return;
}
float getDt(void)
{
return this->dt;
}
void setDt(float dt)
{
 (this->dt) = dt;
}
int getIter(void)
{
return this->num_psi_rotations;
}

};

#endif
// __PSI_INCLUDED__
