#include "Attitude_Estimation.h"
#include "bmx055.h"
#include <math.h>

struct output_data Att_Est(void)
{
  struct bma2x2_accel_data_temp acc_data_xyzt;
  struct bmm050_mag_data_s16_t mag_data;
  struct bmg160_data_t gyro_data_xyzi;
  struct output_data output_data;
  
  float Gx_rad,Gy_rad,Gz_rad,Ax,Ay,Az = 0.0;
  float phi_hat_acc,theta_hat_acc = 0.0;
  float phi_hat_gyr,theta_hat_gyr = 0.0;
  float phi_hat1,theta_hat1,phi_hat2,theta_hat2;
  float phi_hat_complimentary,theta_hat_complimentary = 0;
  float phi_hat_gyr_comp,theta_hat_gyr_comp = 0;
  static int time = 1;

  bma2x2_data_readout(&acc_data_xyzt);
  bmg160_data_readout(&gyro_data_xyzi);
  bmm050_data_readout(&mag_data);

  //Convert gyroscope measurements to radians
  Gx_rad = (float)(gyro_data_xyzi.datax) * pi / 180.0;
  Gy_rad = (float)(gyro_data_xyzi.datay) * pi / 180.0;
  Gz_rad = (float)(gyro_data_xyzi.dataz) * pi / 180.0;

  Ax = (float)(acc_data_xyzt.x);
  Ay = (float)(acc_data_xyzt.y);
  Az = (float)(acc_data_xyzt.z);

  //Accelerometer only
  phi_hat_acc = atan2(Ay,sqrtf(Ax * Ax + Az * Az));
  theta_hat_acc = atan2(-Ax,sqrtf(Ay * Ay + Az * Az));

  //Gyroscope only
  if(time > 1)
  {
    phi_hat_gyr = phi_hat1 + dt * (Gx_rad + sinf(phi_hat1) * tanf(theta_hat1) * Gy_rad * cosf(phi_hat1) * tanf(theta_hat1) * Gz_rad);
    theta_hat_gyr = theta_hat1 + dt * (cosf(phi_hat1) * Gy_rad - sinf(phi_hat1) * Gz_rad);

    //Complimentary Filter
    phi_hat_gyr_comp = phi_hat2 + dt * (Gx_rad + sin(phi_hat2) * tan(theta_hat2) * Gy_rad + cos(phi_hat2) * tan(theta_hat2) * Gz_rad);
    theta_hat_gyr_comp = theta_hat2 + dt * (cos(phi_hat2) * Gy_rad - sin(phi_hat2) * Gz_rad);

    phi_hat_complimentary = (1 - alf) * phi_hat_gyr_comp   + alf * phi_hat_acc;
    theta_hat_complimentary = (1 - alf) * theta_hat_gyr_comp + alf * theta_hat_acc;
  }

  //last state
  phi_hat1 = phi_hat_gyr;
  theta_hat1 = theta_hat_gyr;
  phi_hat2 = phi_hat_complimentary;
  theta_hat2 = theta_hat_complimentary;
  time ++;
  if(time == 1000)
      time = 2;

  //Convert all estimates to degrees
  output_data.phi_out = phi_hat_complimentary * 180.0 / pi; 
  output_data.theta_out = theta_hat_complimentary * 180.0 / pi;

  return output_data;
}
