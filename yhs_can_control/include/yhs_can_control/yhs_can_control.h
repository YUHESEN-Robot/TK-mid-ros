#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/l_wheel_fb.h"
#include "yhs_can_msgs/r_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/free_ctrl_cmd.h"
#include "yhs_can_msgs/bms_fb.h"
#include "yhs_can_msgs/bms_flag_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"
#include "yhs_can_msgs/ultrasonic.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <string>
#include <sstream>

#include <linux/can.h>
#include <linux/can/raw.h>


namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();
private:
	ros::NodeHandle nh_;

	ros::Publisher ctrl_fb_pub_;
	ros::Publisher l_wheel_fb_pub_;
	ros::Publisher r_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher bms_fb_pub_;
	ros::Publisher bms_flag_fb_pub_;

	ros::Publisher odo_fb_pub_;
	ros::Publisher odom_pub_;
	ros::Publisher Veh_Diag_fb_pub_;
    ros::Publisher ultrasonic_pub_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	ros::Subscriber free_ctrl_cmd_sub_;
	ros::Subscriber imu_sub_;

	ros::Time last_imu_time_;

	std::mutex mutex_;
	boost::mutex cmd_mutex_;

	int dev_handler_;
	can_frame send_frames_[2];
	can_frame recv_frames_[1];

	unsigned char sendData_u_io_[8] = {0};
    unsigned char sendData_u_vel_[8] = {0};

  	std::vector<int> ultrasonic_number_;
	std::string odomFrame_, baseFrame_;
	bool tfUsed_;
	std::string if_name_;

	// --- IO Param 配置 ---
    int unlock_pulse_count_ = 0;
	
    bool io_param_enable_;
    bool io_param_lower_beam_;
    bool io_param_upper_beam_;
    int  io_param_turn_lamp_;
    bool io_param_braking_lamp_;
    bool io_param_clearance_lamp_;
    bool io_param_fog_lamp_;
    bool io_param_speaker_;
    int  io_param_discharge_;

    // --- 指令缓存 ---
    yhs_can_msgs::io_cmd current_io_cmd_;
    yhs_can_msgs::ctrl_cmd current_ctrl_cmd_;
    yhs_can_msgs::free_ctrl_cmd free_ctrl_cmd_;

    // --- 定时器 ---
    ros::Timer timer_;

	double imu_roll_, imu_pitch_, imu_yaw_;
	

	void io_cmdCallBack(const yhs_can_msgs::io_cmd::ConstPtr& msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd::ConstPtr& msg);
	void free_ctrl_cmdCallBack(const yhs_can_msgs::free_ctrl_cmd::ConstPtr& msg);
	void ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg);
	void OdomPub(const float linear,const float angular);


	void recvData();

	void sendIoCommand();
    void sendCtrlCommand();
	void sendFreeCtrlCommand();
    void timerCallBack(const ros::TimerEvent& event);

};

}


#endif

