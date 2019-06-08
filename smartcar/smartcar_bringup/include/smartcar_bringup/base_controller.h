#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <pthread.h>
#include <string.h>	//字符串功能函数
#include <math.h>

//以下为串口通讯需要的头文件
#include <unistd.h> 	//UNIX标准函数定义
#include <fcntl.h> 	//文件控制定义
#include <errno.h> 	//错误编号定义
#include <termios.h> 	//POSIX终端控制定义


namespace smartcar
{
	union float2uchar //union的作用为实现char数组和float之间的转换
	{
		float value;
		unsigned char data[4];
	};
	class base_controller
	{
	public:
		base_controller();
		~base_controller();
		void init();
		void cmd_velCallback(const geometry_msgs::Twist & cmd_input);//订阅/cmd_vel主题回调函数
		int usart_reconnect(void);
		int usart_set(int fd, int baude = 115200, int c_flow=0, int bits=8, char parity='N', int stop=1);
		int usart_open(const std::string port_name);
		int usart_close(int fd);
		int safe_write(int fd, const char *ptr, int n);
		int safe_read(int fd, char *ptr,int n);
		int usart_read(int fd, char *r_buf,int len);
		int usart_write(int fd, const char *w_buf,int len);
		void odometry_publish(void);
		void run(void);
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
		
		std::string serial_port;//串口号
		int serial_baud;//串口波特率
		float serial_timeout;
		int serial_fd;
		bool serial_flag;
		unsigned char serial_check;//串口检验次数检查
		std::string HSF2STM;//握手帧,上位机发送给下位机
		std::string HSF2ROS;//握手帧,下位机发送给上位机
		std::string FCS2STM;//校验帧,上位机发送给下位机
		std::string FCS2ROS;//校验帧,下位机发送给上位机
		char DF2STM[19];   //数据帧,由上位机发给下位机的速度指令数据
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
		//里程计信息
		double odom_x, odom_y;
		float leftfront_lastodom, rightfront_lastodom, leftback_lastodom, rightback_lastodom;
		bool odom_init;
		
		char *rec_buffer;  //串口数据接收缓冲器
		int uploading_len;//下位机上传数据大小
		
		pthread_t getOdom_t; 
		//pthread_rwlock_t rwlock;             //声明读写锁  
	};




	
}
#endif // BASE_CONTROLLER_H
