#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

namespace yhs_tool
{

  CanControl::CanControl()
  {
    ros::NodeHandle private_node("~");

    std::string ultrasonic_numbers_str;
    private_node.getParam("ultrasonic_number", ultrasonic_numbers_str);
    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
  	private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
  	private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
  	private_node.param("/yhs_can_control/if_name", if_name_, std::string("can0"));

    private_node.param("/yhs_can_control/io_cmd/enable", io_param_enable_, true);
    private_node.param("/yhs_can_control/io_cmd/lower_beam", io_param_lower_beam_, false);
    private_node.param("/yhs_can_control/io_cmd/upper_beam", io_param_upper_beam_, false);
    private_node.param("/yhs_can_control/io_cmd/turn_lamp", io_param_turn_lamp_, 0); 
    private_node.param("/yhs_can_control/io_cmd/braking_lamp", io_param_braking_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/clearance_lamp", io_param_clearance_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/fog_lamp", io_param_fog_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/speaker", io_param_speaker_, false);
    private_node.param("/yhs_can_control/io_cmd/discharge", io_param_discharge_, 0);

    current_io_cmd_.io_cmd_enable = io_param_enable_;
    current_io_cmd_.io_cmd_lower_beam_headlamp = io_param_lower_beam_;
    current_io_cmd_.io_cmd_upper_beam_headlamp = io_param_upper_beam_;
    current_io_cmd_.io_cmd_turn_lamp = io_param_turn_lamp_;
    current_io_cmd_.io_cmd_braking_lamp = io_param_braking_lamp_;
    current_io_cmd_.io_cmd_clearance_lamp = io_param_clearance_lamp_;
    current_io_cmd_.io_cmd_fog_lamp = io_param_fog_lamp_;
    current_io_cmd_.io_cmd_speaker = io_param_speaker_;
    current_io_cmd_.io_cmd_disCharge = io_param_discharge_;

    current_ctrl_cmd_.ctrl_cmd_gear = 0x03; 
    current_ctrl_cmd_.ctrl_cmd_linear = 0.0;
    current_ctrl_cmd_.ctrl_cmd_angular = 0.0;

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &CanControl::ImuDataCallBack, this);
    last_imu_time_ = ros::Time(0); 
    
    std::istringstream iss(ultrasonic_numbers_str);
    int number;
    while (iss >> number)
    {
      ultrasonic_number_.push_back(number);
    }
  }

  CanControl::~CanControl()
  {
  }

  void CanControl::sendIoCommand()
  {
    static unsigned char count_io = 0;
    memset(sendData_u_io_, 0, 8);

    sendData_u_io_[0] = current_io_cmd_.io_cmd_enable ? 0x01 : 0x00;

    if (unlock_pulse_count_ > 0) {
        sendData_u_io_[0] |= 0x02;
        unlock_pulse_count_--;
    }

    if (current_io_cmd_.io_cmd_lower_beam_headlamp) 
      sendData_u_io_[1] |= 0x01;
    if (current_io_cmd_.io_cmd_upper_beam_headlamp) 
      sendData_u_io_[1] |= 0x02;
    
    sendData_u_io_[1] |= (current_io_cmd_.io_cmd_turn_lamp & 0x03) << 2;

    if (current_io_cmd_.io_cmd_braking_lamp)   
      sendData_u_io_[1] |= 0x10;
    if (current_io_cmd_.io_cmd_clearance_lamp) 
      sendData_u_io_[1] |= 0x20;
    if (current_io_cmd_.io_cmd_fog_lamp)       
      sendData_u_io_[1] |= 0x40;

    if (current_io_cmd_.io_cmd_speaker)        
      sendData_u_io_[2] |= 0x01;

    if (current_io_cmd_.io_cmd_disCharge)      
      sendData_u_io_[5] |= 0x01;

    count_io++;
    if (count_io > 15) count_io = 0;
    sendData_u_io_[6] = count_io << 4;

    sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ 
                        sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

    send_frames_[0].can_id = 0x18C4D7D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_io_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
      ROS_ERROR_THROTTLE(1, "Send IO failed: %d", ret);
  }

  void CanControl::sendCtrlCommand()
  {
    static unsigned char count_ctrl = 0;
    memset(sendData_u_vel_, 0, 8);

    short linear_s = (short)(current_ctrl_cmd_.ctrl_cmd_linear * 1000.0f);
    unsigned short linear = (unsigned short)linear_s;

    short angular_s = (short)(current_ctrl_cmd_.ctrl_cmd_angular * 100.0f);
    unsigned short angular = (unsigned short)angular_s;

    sendData_u_vel_[0] = (0x0f & current_ctrl_cmd_.ctrl_cmd_gear);
    sendData_u_vel_[0] |= (unsigned char)((linear & 0x0f) << 4);

    sendData_u_vel_[1] = (unsigned char)((linear >> 4) & 0xff);

    sendData_u_vel_[2] = (unsigned char)((linear >> 12) & 0x0f);
    sendData_u_vel_[2] |= (unsigned char)((angular & 0x0f) << 4);

    sendData_u_vel_[3] = (unsigned char)((angular >> 4) & 0xff);

    sendData_u_vel_[4] = (unsigned char)((angular >> 12) & 0x0f);

    count_ctrl++;
    if (count_ctrl > 15) count_ctrl = 0;
    sendData_u_vel_[6] = count_ctrl << 4;

    sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ 
                         sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

    send_frames_[0].can_id = 0x18C4D1D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_vel_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
      ROS_ERROR_THROTTLE(1, "Send Ctrl failed: %d", ret);
  }

  void CanControl::sendFreeCtrlCommand()
  {
    short linearl_s = (short)(free_ctrl_cmd_.free_ctrl_cmd_velocity_l * 1000.0f);
    unsigned short linearl = (unsigned short)(linearl_s); 
    short linearr_s = (short)(free_ctrl_cmd_.free_ctrl_cmd_velocity_r * 1000.0f);
    unsigned short linearr = (unsigned short)(linearr_s);
    static unsigned char count_3 = 0;

    unsigned char sendData_u_tem_[8] = {0};

    sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & free_ctrl_cmd_.free_ctrl_cmd_gear);
    sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

    sendData_u_tem_[1] = (linearl >> 4) & 0xff;

    sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));

    sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

    sendData_u_tem_[3] = (linearr >> 4) & 0xff;

    sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

    count_3++;

    if (count_3 == 16)
      count_3 = 0;

    sendData_u_tem_[6] = count_3 << 4;

    sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

    send_frames_[0].can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;

    memcpy(send_frames_[0].data, sendData_u_tem_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));

    if (ret <= 0)
    {
      ROS_ERROR("send message failed, error code: %d", ret);
    }
  }

  void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_io_cmd_ = *msg;

    if (msg->io_cmd_unlock) {
        unlock_pulse_count_ = 1;
    }
  }

  void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_ctrl_cmd_ = *msg;
  }

  void CanControl::free_ctrl_cmdCallBack(const yhs_can_msgs::free_ctrl_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    free_ctrl_cmd_ = *msg;
  }
  
  void CanControl::timerCallBack(const ros::TimerEvent& event)
  {
      static int loop_count = 0;
      boost::mutex::scoped_lock lock(cmd_mutex_);

      sendCtrlCommand();

      if (loop_count % 5 == 0) {
          sendIoCommand();
      }
      loop_count++;
  }

  // 数据接收解析线程
  void CanControl::recvData()
  {

    while (ros::ok())
    {

      if (read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
      {
        for (int j = 0; j < 1; j++)
        {

          switch (recv_frames_[0].can_id)
          {
            case 0x18C4D1EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::ctrl_fb msg;
              msg.ctrl_fb_target_gear = 0x0f & recv_frames_[0].data[0];

              int linear_raw = ((recv_frames_[0].data[0] & 0xf0) >> 4) | 
                                (recv_frames_[0].data[1] << 4) | ((recv_frames_[0].data[2] & 0x0f) << 12);

              if (linear_raw & 0x8000)
                linear_raw |= 0xFFFF0000;
              msg.ctrl_fb_linear = (float)linear_raw / 1000.0f;

              int angular_raw = ((recv_frames_[0].data[2] & 0xf0) >> 4) | (recv_frames_[0].data[3] << 4) | ((recv_frames_[0].data[4] & 0x0f) << 12);
              if (angular_raw & 0x8000)
                angular_raw |= 0xFFFF0000;
              msg.ctrl_fb_angular = (float)angular_raw / 100.0f;

              msg.ctrl_fb_mode = (recv_frames_[0].data[5] & 0x30) >> 4;

              msg.ctrl_fb_RemoteSt = (recv_frames_[0].data[5] & 0x80) >> 7; 

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];
              
              if (crc == recv_frames_[0].data[7])
              {

                ctrl_fb_pub_.publish(msg);
                float current_vel = msg.ctrl_fb_linear;
                if (msg.ctrl_fb_target_gear == 2) 
                  current_vel = -current_vel;
                float current_ang = msg.ctrl_fb_angular * 3.14159f / 180.0f;
                
                OdomPub(current_vel, current_ang);
              }

              break;
            }

            //
            case 0x18C4D7EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::l_wheel_fb msg;
              msg.l_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

              msg.l_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                l_wheel_fb_pub_.publish(msg);
              }

              break;
            }

            //
            case 0x18C4D8EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::r_wheel_fb msg;
              msg.r_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

              msg.r_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                r_wheel_fb_pub_.publish(msg);
              }

              break;
            }

            // io反馈
            case 0x18C4DAEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::io_fb msg;
              if (0x01 & recv_frames_[0].data[0])
                msg.io_fb_enable = true;
              else
                msg.io_fb_enable = false;

              if (0x01 & recv_frames_[0].data[1])
                msg.io_fb_lower_beam_headlamp = true;
              else
                msg.io_fb_lower_beam_headlamp = false;

              if (0x02 & recv_frames_[0].data[1])
                msg.io_fb_upper_beam_headlamp = true;
              else
                msg.io_fb_upper_beam_headlamp = false;

              msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

              if (0x10 & recv_frames_[0].data[1])
                msg.io_fb_braking_lamp = true;
              else
                msg.io_fb_braking_lamp = false;

              if (0x20 & recv_frames_[0].data[1])
                msg.io_fb_clearance_lamp = true;
              else
                msg.io_fb_clearance_lamp = false;

              if (0x40 & recv_frames_[0].data[1])
                msg.io_fb_fog_lamp = true;
              else
                msg.io_fb_fog_lamp = false;

              if (0x01 & recv_frames_[0].data[2])
                msg.io_fb_speaker = true;
              else
                msg.io_fb_speaker = false;

              if (0x01 & recv_frames_[0].data[3])
                msg.io_fb_fl_impact_sensor = true;
              else
                msg.io_fb_fl_impact_sensor = false;

              if (0x02 & recv_frames_[0].data[3])
                msg.io_fb_fm_impact_sensor = true;
              else
                msg.io_fb_fm_impact_sensor = false;

              if (0x04 & recv_frames_[0].data[3])
                msg.io_fb_fr_impact_sensor = true;
              else
                msg.io_fb_fr_impact_sensor = false;

              if (0x08 & recv_frames_[0].data[3])
                msg.io_fb_rl_impact_sensor = true;
              else
                msg.io_fb_rl_impact_sensor = false;

              if (0x10 & recv_frames_[0].data[3])
                msg.io_fb_rm_impact_sensor = true;
              else
                msg.io_fb_rm_impact_sensor = false;

              if (0x20 & recv_frames_[0].data[3])
                msg.io_fb_rr_impact_sensor = true;
              else
                msg.io_fb_rr_impact_sensor = false;

              if (0x01 & recv_frames_[0].data[4])
                msg.io_fb_fl_drop_sensor = true;
              else
                msg.io_fb_fl_drop_sensor = false;

              if (0x02 & recv_frames_[0].data[4])
                msg.io_fb_fm_drop_sensor = true;
              else
                msg.io_fb_fm_drop_sensor = false;

              if (0x04 & recv_frames_[0].data[4])
                msg.io_fb_fr_drop_sensor = true;
              else
                msg.io_fb_fr_drop_sensor = false;

              if (0x08 & recv_frames_[0].data[4])
                msg.io_fb_rl_drop_sensor = true;
              else
                msg.io_fb_rl_drop_sensor = false;

              if (0x10 & recv_frames_[0].data[4])
                msg.io_fb_rm_drop_sensor = true;
              else
                msg.io_fb_rm_drop_sensor = false;

              if (0x20 & recv_frames_[0].data[4])
                msg.io_fb_rr_drop_sensor = true;
              else
                msg.io_fb_rr_drop_sensor = false;

              if (0x01 & recv_frames_[0].data[5])
                msg.io_fb_disChargeflg = true;
              else
                msg.io_fb_disChargeflg = false;

              if (0x02 & recv_frames_[0].data[5])
                msg.io_fb_chargeEn = true;
              else
                msg.io_fb_chargeEn = false;

              if (0x10 & recv_frames_[0].data[5])
                msg.io_fb_ScramSt = true;
              else
                msg.io_fb_ScramSt = false;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {

                io_fb_pub_.publish(msg);
              }

              break;
            }

            // bms反馈
            case 0x18C4E1EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::bms_fb msg;
              msg.bms_fb_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

              msg.bms_fb_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

              msg.bms_fb_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {

                bms_fb_pub_.publish(msg);
              }

              break;
            }

            case 0x18C4E2EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::bms_flag_fb msg;
              uint8_t* d = recv_frames_[0].data;

              msg.bms_flag_fb_soc = d[0];

              msg.bms_flag_fb_single_ov    = (d[1] >> 0) & 0x01;
              msg.bms_flag_fb_single_uv    = (d[1] >> 1) & 0x01;
              msg.bms_flag_fb_ov           = (d[1] >> 2) & 0x01;
              msg.bms_flag_fb_uv           = (d[1] >> 3) & 0x01;
              msg.bms_flag_fb_charge_ot    = (d[1] >> 4) & 0x01;
              msg.bms_flag_fb_charge_ut    = (d[1] >> 5) & 0x01;
              msg.bms_flag_fb_discharge_ot = (d[1] >> 6) & 0x01;
              msg.bms_flag_fb_discharge_ut = (d[1] >> 7) & 0x01;

              msg.bms_flag_fb_charge_oc    = (d[2] >> 0) & 0x01;
              msg.bms_flag_fb_discharge_oc = (d[2] >> 1) & 0x01;
              msg.bms_flag_fb_short        = (d[2] >> 2) & 0x01;
              msg.bms_flag_fb_ic_error     = (d[2] >> 3) & 0x01;
              msg.bms_flag_fb_lock_mos     = (d[2] >> 4) & 0x01;
              
              msg.bms_flag_fb_charge_flag  = (d[2] >> 5) & 0x01;

              msg.bms_flag_fb_SOCWarning   = (d[2] >> 7) & 0x01;

              msg.bms_flag_fb_SOCLowProtection = (d[3] >> 0) & 0x01;

              uint16_t h_temp_raw = ((d[3] & 0xF0) >> 4) | (d[4] << 4);
              msg.bms_flag_fb_hight_temperature = (float)h_temp_raw * 0.1f;

              uint16_t l_temp_raw = d[5] | ((d[6] & 0x0F) << 8);
              msg.bms_flag_fb_low_temperature = (float)l_temp_raw * 0.1f;

              unsigned char crc = d[0]^d[1]^d[2]^d[3]^d[4]^d[5]^d[6];
              if (crc == d[7])
              {
                bms_flag_fb_pub_.publish(msg);
              }
              break;
            }

            // odo
            case 0x18C4DEEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::odo_fb msg;
              msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[j].data[3] << 24 | recv_frames_[j].data[2] << 16 | recv_frames_[j].data[1] << 8 | recv_frames_[j].data[0])) / 1000;

              msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[j].data[7] << 24 | recv_frames_[j].data[6] << 16 | recv_frames_[j].data[5] << 8 | recv_frames_[j].data[4])) / 1000;

              odo_fb_pub_.publish(msg);

              break;
            }

            default:
              break;
          }
        }
      }
    }
  }

  void CanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_time_ = ros::Time::now();
    tf2::Quaternion quaternion;
    tf2::fromMsg(imu_data_msg->orientation, quaternion);

    imu_yaw_ = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void CanControl::OdomPub(const float linear,const float angular)
  {
  	static double x = 0.0;
  	static double y = 0.0;
  	static double th = 0.0;
  
  	static double lastYaw = 0;
  
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
  
  
  	double vx = linear;
  	double vth = angular;
  
  	current_time = ros::Time::now();
  
  	double dt = (current_time - last_time).toSec();

    bool is_imu_active = (current_time - last_imu_time_).toSec() < 0.2;

    if (is_imu_active)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        th = imu_yaw_; 
    }
    else
    {
        th += vth * dt; 
    }
  
  	double delta_x = (vx * cos(th)) * dt;
  	double delta_y = (vx * sin(th)) * dt;
  
  	x += delta_x;
  	y += delta_y;

    tf2::Quaternion quat;
    if (is_imu_active) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat.setRPY(imu_roll_, imu_pitch_, th);
    } else {
        quat.setRPY(0, 0, th);
    }
  
  	geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);
  
  	geometry_msgs::TransformStamped odom_trans;
  	odom_trans.header.stamp = current_time;
  	odom_trans.header.frame_id = odomFrame_;
  	odom_trans.child_frame_id = baseFrame_;
  
  	odom_trans.transform.translation.x = x;
  	odom_trans.transform.translation.y = y;
  	odom_trans.transform.translation.z = 0.0;
  	odom_trans.transform.rotation = odom_quat;
  
  	if(tfUsed_)
  	odom_broadcaster.sendTransform(odom_trans);
  
  	nav_msgs::Odometry odom;
  	odom.header.stamp = current_time;
  	odom.header.frame_id = odomFrame_;
  
  	odom.pose.pose.position.x = x;
  	odom.pose.pose.position.y = y;
  	odom.pose.pose.position.z = 0.0;
  	odom.pose.pose.orientation = odom_quat;
  
  	odom.child_frame_id = baseFrame_;
  	odom.twist.twist.linear.x = vx;
  	odom.twist.twist.linear.y = 0.0;
  	odom.twist.twist.angular.z = vth;
  
  	odom.pose.covariance[0]  = 0.1;   	
  	odom.pose.covariance[7]  = 0.1;		
  	odom.pose.covariance[35] = 0.2;   
  
  	odom.pose.covariance[14] = 1e10; 	
  	odom.pose.covariance[21] = 1e10; 	
  	odom.pose.covariance[28] = 1e10; 	
  	odom_pub_.publish(odom);
  
  	last_time = current_time;
  }

  void CanControl::run()
  {

    ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
    free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::free_ctrl_cmd>("free_ctrl_cmd", 5, &CanControl::free_ctrl_cmdCallBack, this);

    ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb", 5);
    io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb", 5);
    l_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::l_wheel_fb>("l_wheel_fb", 5);
    r_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::r_wheel_fb>("r_wheel_fb", 5);
    bms_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_fb>("bms_fb", 5);
    bms_flag_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_fb>("bms_flag_fb", 5);

    odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
    Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic", 5);

    dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (dev_handler_ < 0)
    {
      ROS_ERROR(">>open can deivce error!");
      return;
    }
    else
    {
      ROS_INFO(">>open can deivce success!");
    }

    struct ifreq ifr;

    std::string can_name("can0");

    strcpy(ifr.ifr_name, can_name.c_str());

    ioctl(dev_handler_, SIOCGIFINDEX, &ifr);

    // bind socket to network interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (ret < 0)
    {
      ROS_ERROR(">>bind dev_handler error!\r\n");
      return;
    }

    boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));

    timer_ = nh_.createTimer(ros::Duration(0.01), &CanControl::timerCallBack, this);

    ros::spin();

    close(dev_handler_);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yhs_can_control_node");

  yhs_tool::CanControl cancontrol;
  cancontrol.run();

  return 0;
}
