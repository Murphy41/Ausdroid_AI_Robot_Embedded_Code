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
 *********************F******************************************************/

#include "can.h"
#include "board.h"
#include "dbus.h"
#include "gimbal.h"
#include "timer_task.h"
#include "gimbal_task.h"
#include "infantry_cmd.h"
#include "offline_check.h"
#include "param.h"
#include "ramp.h"

#define DEFAULT_IMU_TEMP 50

/*add by qian*/
/* gimbal patrol angle (degree) */
//#define PATROL_ANGLE  15
/* patrol period time (ms) */
//#define PATROL_PERIOD 1000

/* gimbal period time (ms) */
#define GIMBAL_PERIOD 2
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 3000

float pit_delta, yaw_delta;

static void imu_temp_ctrl_init(void); // staic 局部变量 void 无返回值
static int32_t gimbal_imu_updata(void *argc);
static int32_t imu_temp_keep(void *argc);
static void auto_gimbal_adjust(gimbal_t pgimbal);
static void gimbal_state_init(gimbal_t pgimbal);

uint8_t auto_adjust_f; 
uint8_t auto_init_f;

/* control ramp parameter */
static ramp_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_t pitch_ramp = RAMP_GEN_DAFAULT;

int32_t yaw_angle_fdb_js, yaw_angle_ref_js; // yaw 偏航 左右
int32_t pit_angle_fdb_js, pit_angle_ref_js; // 倾斜 上下
int32_t yaw_spd_fdb_js, yaw_spd_ref_js;
int32_t pit_spd_fdb_js, pit_spd_ref_js;

void gimbal_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
  rc_device_t prc_dev = NULL;
  rc_info_t prc_info = NULL;
  gimbal_t pgimbal = NULL;
  cali_sys_t *pparam = NULL;

  pgimbal = gimbal_find("gimbal");
  prc_dev = rc_device_find("can_rc");  // protocol
  pparam = get_cali_param();

  if (pparam->gim_cali_data.calied_done == CALIED_FLAG)
  {
    gimbal_set_offset(pgimbal, pparam->gim_cali_data.yaw_offset, pparam->gim_cali_data.pitch_offset);
  }
  else
  {
    auto_adjust_f = 1;
  }

  gimbal_init_state_reset();

  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
  else
  {
  }

  soft_timer_register(imu_temp_keep, (void *)pgimbal, 5); // 温度计 没用
  soft_timer_register(gimbal_push_info, (void *)pgimbal, 10);

  imu_temp_ctrl_init();

  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)  //右边的拨片保持在上
    {
      gimbal_set_yaw_mode(pgimbal, GYRO_MODE);       //设定陀螺仪模式（手动模式），按照陀螺仪参数运动
      pit_delta = -(float)prc_info->ch4 * 0.0007f;		//遥控器摇杆控制云台俯仰位置，陀螺仪使其保持
      yaw_delta = -(float)prc_info->ch3 * 0.0007f;  //遥控器遥杆控制云台左右位置，陀螺仪使其保持
      gimbal_set_pitch_delta(pgimbal, pit_delta);    
      gimbal_set_yaw_delta(pgimbal, yaw_delta);    //设置云台转动参数
    }

    if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)  //右边的拨片保持在中，可用作Debug
    {			
			gimbal_set_yaw_mode(pgimbal, PATROL_MODE);  			//进入patrol mode		

      if (rc_device_get_state(prc_dev, RC_S2_UP2MID) == RM_OK)		//切换瞬间？
      {
        gimbal_set_yaw_angle(pgimbal, 0, 0);
      }
    }

    if (rc_device_get_state(prc_dev, RC_S2_DOWN2MID) == RM_OK)		//切换瞬间？
    {
      gimbal_set_yaw_angle(pgimbal, 0, 0);
    }
 
		//****************************************************************改的*************************************************************
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
		{
			gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
		}
    //{
		//	if (Patrol_FLAG == 1)
			/*{				
				gimbal_set_yaw_mode(pgimbal, PATROL_MODE);
			}
			else if
			{
				gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
			}
   // }      */
		 //****************************************************************改的***********************************************************8
		
    if (get_offline_state() == 0)
    {
      gimbal_state_init(pgimbal);
    }

    auto_gimbal_adjust(pgimbal);

    yaw_angle_fdb_js = pgimbal->cascade[0].outer.get * 1000;
    yaw_angle_ref_js = pgimbal->cascade[0].outer.set * 1000;
    pit_angle_fdb_js = pgimbal->cascade[1].outer.get * 1000;
    pit_angle_ref_js = pgimbal->cascade[1].outer.set * 1000;

    yaw_spd_fdb_js = pgimbal->cascade[0].inter.get * 1000;
    yaw_spd_ref_js = pgimbal->cascade[0].inter.set * 1000;
    pit_spd_fdb_js = pgimbal->cascade[1].inter.get * 1000;
    pit_spd_ref_js = pgimbal->cascade[1].inter.set * 1000;

    gimbal_imu_updata(pgimbal);
    gimbal_execute(pgimbal);
    osDelayUntil(&period, 2);
  }
}

static int32_t gimbal_imu_updata(void *argc)
{
  struct ahrs_sensor mpu_sensor;  // ahrs mpu 陀螺仪加速器型号
  struct attitude mahony_atti;
  gimbal_t pgimbal = (gimbal_t)argc;
  mpu_get_data(&mpu_sensor);
  mahony_ahrs_updateIMU(&mpu_sensor, &mahony_atti);
  gimbal_pitch_gyro_update(pgimbal, -mahony_atti.roll);
  gimbal_rate_update(pgimbal, mpu_sensor.wz * RAD_TO_DEG, -mpu_sensor.wx * RAD_TO_DEG);
  return 0;
}

struct pid pid_imu_tmp; //惯性测量单元 用来定位的

static void imu_temp_ctrl_init(void)
{
  pid_struct_init(&pid_imu_tmp, 2000, 500, 1100, 10, 0);
}

static int32_t imu_temp_keep(void *argc)
{
  float temp;
  mpu_get_temp(&temp); //微处理器和内存保护单元
  pid_calculate(&pid_imu_tmp, temp, DEFAULT_IMU_TEMP); 
  mpu_heat_output(pid_imu_tmp.out);
  return 0;
}

uint8_t auto_adjust_f;//    是否adjust判断值
volatile uint32_t pit_time, yaw_time;//volatile 是一个类型修饰符。volatile 的作用是作为指令关键字，确保本条指令不会因编译器的优化而省略
uint32_t pit_cnt;  //cnt计数器
volatile uint16_t yaw_ecd_r, yaw_ecd_l; // ecd是encode模式
volatile uint16_t pit_ecd_c, yaw_ecd_c;

void send_gimbal_current(int16_t iq1, int16_t iq2, int16_t iq3)
{
  static uint8_t tx_data[8];

  tx_data[0] = iq1 >> 8;
  tx_data[1] = iq1;
  tx_data[2] = iq2 >> 8;
  tx_data[3] = iq2;
  tx_data[4] = iq3 >> 8;
  tx_data[5] = iq3;

  can_msg_bytes_send(&hcan1, tx_data, 6, 0x1FF);
}

struct pid pid_pit = {0};
struct pid pid_pit_spd = {0};

static void auto_gimbal_adjust(gimbal_t pgimbal)
{
  if (auto_adjust_f)
  {
    pid_struct_init(&pid_pit, 2000, 0, 60, 0, 0);
    pid_struct_init(&pid_pit_spd, 30000, 3000, 60, 0.2, 0);// spd 是speed 双环pid 
    while (1)
    {
      gimbal_imu_updata(pgimbal);

      pid_calculate(&pid_pit, pgimbal->sensor.gyro_angle.pitch, 0);
      pid_calculate(&pid_pit_spd, pid_pit.out, pgimbal->sensor.rate.pitch_rate);

      send_gimbal_current(0, -pid_pit_spd.out, 0);
      HAL_Delay(2);

      if ((fabs(pgimbal->sensor.gyro_angle.pitch) < 0.1)) // cnt 计数是为了当他小于阈值1000次后即视为稳定
      {
        pit_cnt++;
      }
      else
      {
        pit_cnt = 0;
      }
      if (pit_cnt > 1000)
      {
        pit_ecd_c = pgimbal->motor[PITCH_MOTOR_INDEX].data.ecd;
        break;
      }
    }

    {
      yaw_time = get_time_ms();
      while (get_time_ms() - yaw_time <= 2000)
      {
        send_gimbal_current(6000, 0, 0);
        yaw_ecd_l = pgimbal->motor[YAW_MOTOR_INDEX].data.ecd;
        HAL_Delay(2);
      }

      yaw_time = HAL_GetTick();
      while (HAL_GetTick() - yaw_time <= 2000)
      {
        send_gimbal_current(-6000, 0, 0);
        yaw_ecd_r = pgimbal->motor[YAW_MOTOR_INDEX].data.ecd;
        HAL_Delay(2);
      }

      if (yaw_ecd_l > yaw_ecd_r)
      {
        yaw_ecd_c = (yaw_ecd_l + yaw_ecd_r) / 2;
      }
      else
      {
        if ((yaw_ecd_l + yaw_ecd_r) / 2 > 4096)
        {
          yaw_ecd_c = (yaw_ecd_l + yaw_ecd_r) / 2 - 4096;
        }
        else
        {
          yaw_ecd_c = (yaw_ecd_l + yaw_ecd_r) / 2 + 4096;
        }
      }
    }
    gimbal_save_data(yaw_ecd_c, pit_ecd_c); //存了yaw和pitch center的数
    gimbal_set_offset(pgimbal, yaw_ecd_c, pit_ecd_c); //和yaw和pitch center的数还有pgimbal设成offset 所以pgimbal是啥？？？？？？？
    auto_adjust_f = 0;
    __disable_irq();
    NVIC_SystemReset();
    while (1)
      ;
  }
}

void gimbal_auto_adjust_start(void)
{
  auto_adjust_f = 1;
}

uint8_t get_gimbal_init_state(void)
{
  return auto_init_f;
}

void gimbal_init_state_reset(void)
{
  ramp_init(&pitch_ramp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  ramp_init(&yaw_ramp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  auto_init_f = 0;
}

static void gimbal_state_init(gimbal_t pgimbal)
{
  if (auto_init_f == 0)
  {
    gimbal_set_pitch_mode(pgimbal, ENCODER_MODE);
    gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
    gimbal_yaw_disable(pgimbal);
    gimbal_set_pitch_angle(pgimbal, pgimbal->ecd_angle.pitch * (1 - ramp_calculate(&pitch_ramp)));

    if ((pgimbal->ecd_angle.pitch != 0) && (pgimbal->ecd_angle.yaw != 0))
    {
      if (fabs(pgimbal->ecd_angle.pitch) < 1.5f)
      {
        gimbal_yaw_enable(pgimbal);
        gimbal_set_yaw_angle(pgimbal, pgimbal->ecd_angle.yaw * (1 - ramp_calculate(&yaw_ramp)), 0); 
        if (fabs(pgimbal->ecd_angle.yaw) < 1.5f)
        {
          auto_init_f = 1;
        }
      }
    }
  }
}
