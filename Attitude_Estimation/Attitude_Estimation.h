#ifndef __ATTITUDE_ESTIMATION_H__
#define __ATTITUDE_ESTIMATION_H__

#define dt 0.0185//»ý·Ö¼ä¸ô Avg. time step

struct output_data
{
    float phi_out;
    float theta_out;
};

struct output_data Att_Est(void);

#endif
