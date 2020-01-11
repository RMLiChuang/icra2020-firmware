/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "external_gyro.h"
#include "errno.h"
#include "drv_can.h"
#include "ahrs.h"
	struct rm_imu_data_t external_gyro;
void quat_to_euler(struct rm_imu_data_t *rm_imu_data,float q0,float q1,float q2,float q3)
{
	float recipNorm;
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
	rm_imu_data->euler_angle_fp32[0]=atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; //yaw
	rm_imu_data->euler_angle_fp32[1]=asin(-2*q1*q3 + 2*q0*q2)* 57.3; 												 //pit
	rm_imu_data->euler_angle_fp32[2]=atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3; //rol
}

int32_t external_gyro_update(struct rm_imu_data_t *rm_imu_data, CAN_RxHeaderTypeDef *header,uint8_t *can_rx_data)
{
	//struct can_std_msg rx_message;
//	struct rm_imu_data_t rm_imu_data;
//	rm_imu_data = *gyro;
	
  if(rm_imu_data == NULL)
  {
    return -RM_INVAL;
  }
	switch(header->StdId)
	{     
		case RM_IMU_PARAM_ID:     
		{         
			rm_imu_data->accel_rangle = can_rx_data[0] &0x0F;         
			rm_imu_data->gyro_rangle = (can_rx_data[0] &0xF0) >> 4;         
			rm_imu_data->sensor_control_temperature = can_rx_data[2];         
			rm_imu_data->imu_sensor_rotation = can_rx_data[3] & 0x1F;         
			rm_imu_data->ahrs_rotation_sequence = (can_rx_data[3] & 0xE0) >> 5;         
			rm_imu_data->quat_euler = can_rx_data[4] & 0x01;         
			switch(rm_imu_data->gyro_rangle)         
				{ 
          case 0: rm_imu_data->gyro_sen = GYRO_2000_SEN; break;             
					case 1: rm_imu_data->gyro_sen = GYRO_1000_SEN; break;             
					case 2: rm_imu_data->gyro_sen = GYRO_500_SEN; break;             
					case 3: rm_imu_data->gyro_sen = GYRO_250_SEN; break;             
					case 4: rm_imu_data->gyro_sen = GYRO_125_SEN; break;         
				} 
 
        switch(rm_imu_data->accel_rangle)         
					{             
						case 0: rm_imu_data->accel_sen = ACCEL_3G_SEN; break;             
						case 1: rm_imu_data->accel_sen = ACCEL_6G_SEN; break;             
						case 2: rm_imu_data->accel_sen = ACCEL_12G_SEN; break;
						case 3: rm_imu_data->accel_sen = ACCEL_24G_SEN; break;         
					}         
				break;
			}
			case RM_IMU_QUAT_ID:     
			{         
				if(rm_imu_data->quat_euler && header->DLC == 6)         
					{             
						memcpy(rm_imu_data->euler_angle, can_rx_data, header->DLC);             
						rm_imu_data->euler_angle_fp32[0] = rm_imu_data->euler_angle[0] * 0.0001f;             
						rm_imu_data->euler_angle_fp32[1] = rm_imu_data->euler_angle[1] * 0.0001f;             
						rm_imu_data->euler_angle_fp32[2] = rm_imu_data->euler_angle[2] * 0.0001f;         
					}         
					if(rm_imu_data->quat_euler == 0 && header->DLC == 8) 
					{             
						memcpy(rm_imu_data->quat, can_rx_data, 8);             
						rm_imu_data->quat_fp32[0] = rm_imu_data->quat[0] * 0.0001f;             
						rm_imu_data->quat_fp32[1] = rm_imu_data->quat[1] * 0.0001f;             
						rm_imu_data->quat_fp32[2] = rm_imu_data->quat[2] * 0.0001f;             
						rm_imu_data->quat_fp32[3] = rm_imu_data->quat[3] * 0.0001f;         
						
						quat_to_euler(rm_imu_data,rm_imu_data->quat_fp32[0],rm_imu_data->quat_fp32[1],rm_imu_data->quat_fp32[2],rm_imu_data->quat_fp32[3]);
					}         
				break;     
			}     
			case RM_IMU_GYRO_ID:     
			{         
					memcpy(rm_imu_data->gyro_int16, can_rx_data,6);         
					rm_imu_data->gyro_fp32[0] = rm_imu_data->gyro_int16[0] * rm_imu_data->gyro_sen;         
					rm_imu_data->gyro_fp32[1] = rm_imu_data->gyro_int16[1] * rm_imu_data->gyro_sen;         
					rm_imu_data->gyro_fp32[2] = rm_imu_data->gyro_int16[2] * rm_imu_data->gyro_sen;         
					rm_imu_data->sensor_temperature = (int16_t)((can_rx_data[6] << 3) | (can_rx_data[7] >> 5));       
					if (rm_imu_data->sensor_temperature > 1023)         
					{             
						rm_imu_data->sensor_temperature -= 2048;         
					}         
					break;     
			}     
			case RM_IMU_ACCEL_ID:     
			{         
					memcpy(rm_imu_data->accel_int16, can_rx_data,6);         
					rm_imu_data->accel_fp32[0] = rm_imu_data->accel_int16[0] * rm_imu_data->accel_sen;         
					rm_imu_data->accel_fp32[1] = rm_imu_data->accel_int16[1] * rm_imu_data->accel_sen;         
					rm_imu_data->accel_fp32[2] = rm_imu_data->accel_int16[2] * rm_imu_data->accel_sen;         
					memcpy(&rm_imu_data->sensor_time, (can_rx_data + 6), 2);         
					break;     
			}     
			case RM_IMU_MAG_ID:     
			{         
					memcpy(rm_imu_data->mag_int16, can_rx_data,6);         
					break;     
			} 
		
	}
	
	return RM_OK;
}

