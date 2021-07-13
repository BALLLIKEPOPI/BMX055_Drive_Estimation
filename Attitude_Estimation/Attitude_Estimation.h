#ifndef __ATTITUDE_ESTIMATION_H__
#define __ATTITUDE_ESTIMATION_H__

#define dt 0.01//»ý·Ö¼ä¸ô Avg. time step
#define pi 3.1415926
#define alf 0.1

struct output_data
{
    float phi_out;
    float theta_out;
};

struct output_data Att_Est(void);

#endif
