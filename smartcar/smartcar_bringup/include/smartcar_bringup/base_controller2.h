#ifndef BASE_CONTROLLER2_H
#define BASE_CONTROLLER2_H

#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <iostream>
#include <pthread.h>
#include <string.h>	//字符串功能函数
#include <math.h>

//以下为串口通讯需要的头文件
#include <serial/serial.h>


namespace smartcar
{
	union float2uchar //union的作用为实现char数组和float之间的转换
	{
		float value;
		unsigned char data[4];
	};
	class base_controller2
	{
	public:
		base_controller2();
		~base_controller2();
		void init();
		void cmd_velCallback(const geometry_msgs::Twist & cmd_input);//订阅/cmd_vel主题回调函数
		int usart_reconnect(void);
		int usart_set(int fd, int baude = 115200, int c_flow=0, int bits=8, char parity='N', int stop=1);
		int usart_open(const std::string port_name);
		int usart_close(int fd);
	
		int usart_read(int fd, char *r_buf,int len);
		int usart_write(int fd, const unsigned char *w_buf,int len);
		
		void odometry_publish();
		static void* getOdometry_thread(void* args);
		
	private:
		ros::NodeHandle handle;
		ros::Subscriber cmd_sub;
		ros::Publisher odom_pub;
		
		//定义odom话题相关变量
		tf::TransformBroadcaster odom_broadcaster;//定义tf对象
		geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
		nav_msgs::Odometry odom;//定义里程计对象
		geometry_msgs::Quaternion odom_quat; //四元数变量
		
		serial::Serial ros_serial;
		std::string serial_port;//串口号
		int serial_baud;//串口波特率
		float serial_timeout;
		int serial_fd;
		bool serial_flag;
		std::string handshake_str;
		std::string handcheck_str;
		std::string getreply_str;
		std::string sendcheck_str;
		//static int serial_ret;
		/*串口互斥锁
		 * 0:串口未使用
		 * 1:读串口进程开始
		 * 2:读串口进程结束
		 * 3:写串口进程开始
		 * 4:写串口进程结束
		 */
		char serial_lock;
		
		double convert_ratio;//转速转换比例，执行速度调整比例
		double wheel_dis;//轮距车中心的间距
		double sampling_tim;
		
		float2uchar leftfront_vel, rightfront_vel, leftback_vel, rightback_vel;//发送给下位机的左右轮速度
		float2uchar leftfront_odom, rightfront_odom, leftback_odom, rightback_odom, anglevel_odom, angleyaw_odom;//里程计的相关信息
		double odom_x, odom_y;
		
		char *rec_buffer;  //串口数据接收缓冲器
		int uploading_len;//下位机上传数据大小
		
		pthread_t getOdom_t; 
		//pthread_rwlock_t rwlock;             //声明读写锁  
	};




	
}
#endif // BASE_CONTROLLER_H
