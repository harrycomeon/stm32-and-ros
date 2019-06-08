#include "smartcar_bringup/base_controller2.h"

namespace smartcar
{
	base_controller2::base_controller2()
	{
	
 		handle.getParam("/smartcar_controller2/serial_port", serial_port);
		handle.getParam("/smartcar_controller2/serial_baud", serial_baud);
		handle.getParam("/smartcar_controller2/serial_timeout", serial_timeout);
		
		handle.getParam("/smartcar_controller2/convert_ratio", convert_ratio);
		handle.getParam("/smartcar_controller2/wheel_distance", wheel_dis);
		handle.getParam("/smartcar_controller2/handshake_str", handshake_str);
		handshake_str=handshake_str+"\r\n";
		handle.getParam("/smartcar_controller2/getreply_str", getreply_str);
		getreply_str=getreply_str+"\r\n";
		handle.getParam("/smartcar_controller2/handcheck_str", handcheck_str);
		handle.getParam("/smartcar_controller2/sendcheck_str", sendcheck_str);
		handle.getParam("/smartcar_controller2/uploading_len", uploading_len);
		handle.getParam("/smartcar_controller2/sampling_tim", sampling_tim);

		serial_flag = false;
		serial_lock = 0;
		serial_fd=-1;
		odom_x = 0;
		odom_y = 0;
		rec_buffer = new char[uploading_len];
		memset(rec_buffer, 0, sizeof(rec_buffer));
	}
	base_controller2::~base_controller2()
	{
		delete[] rec_buffer;
		usart_close(serial_fd);
	}
	void base_controller2::init()
	{
		//定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
		float covariance[36] = {0.01,   0,    0,     0,     0,     0,  	// covariance on gps_x
							0,  0.01, 0,     0,     0,     0,  	// covariance on gps_y
							0,  0,    99999, 0,     0,     0,  	// covariance on gps_z
							0,  0,    0,     99999, 0,     0,  	// large covariance on rot x
							0,  0,    0,     0,     99999, 0,  	// large covariance on rot y
							0,  0,    0,     0,     0,     0.01}; 	 // large covariance on rot z 
		//载入covariance矩阵
		for(int i = 0; i < 36; i++)
			odom.pose.covariance[i] = covariance[i];
		
		cmd_sub = handle.subscribe("cmd_vel", 5, &base_controller2::cmd_velCallback, this); //订阅/cmd_vel主题
		odom_pub= handle.advertise<nav_msgs::Odometry>("odom", 5); //定义要发布/odom主题
		
// 		int ret2=0;
// 		ret2 = pthread_create(&getOdom_t, NULL, getOdometry_thread, this);
// 		if (ret2 != 0) ROS_ERROR("The getOdometry_thread pthread_create error: error_code=%d", ret2); 
	}
	void base_controller2::cmd_velCallback(const geometry_msgs::Twist& cmd_input)
	{
		float xlinear_temp=0, ylinear_temp=0, angular_temp=0;//暂存的线速度和角速度
		char speed_data[18]={0};   //要发给串口的数据
		char data_terminal0=0x0d;  //“/r"字符
		char data_terminal1=0x0a;  //“/n"字符
		
		// Copy the velocities (m/s)
		angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
		xlinear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度,m/s
		ylinear_temp = cmd_input.linear.y ;//获取/cmd_vel的线速度,m/s
		
		// Calculate individual wheel linear velocities (m/s)将转换好的小车速度分量为左右轮速度
		leftfront_vel.value 		= -  ( (xlinear_temp - ylinear_temp)/(cos(M_PI/4.0)) - angular_temp *wheel_dis );
		rightfront_vel.value 	= (xlinear_temp +ylinear_temp)/(cos(M_PI/4.0)) +angular_temp *wheel_dis;
		leftback_vel.value 		= -  ( (xlinear_temp +ylinear_temp)/(cos(M_PI/4.0)) +angular_temp *wheel_dis );
		rightback_vel.value 	= (xlinear_temp - ylinear_temp)/(cos(M_PI/4.0)) - angular_temp *wheel_dis;

		////存入数据到要发布的左右轮速度消息 ,并放大ratio倍，即单位mm/s
		leftfront_vel.value = leftfront_vel.value *convert_ratio;
		rightfront_vel.value = rightfront_vel.value *convert_ratio;
		leftback_vel.value = leftback_vel.value *convert_ratio;
		rightback_vel.value = rightback_vel.value *convert_ratio;
		
		//将左右轮速度存入数组中发送给串口
		for(int i=0;i<4;i++)
		{
			speed_data[i]		=leftfront_vel.data[i];
			speed_data[i+4]	=rightfront_vel.data[i];
			speed_data[i+8]	=leftback_vel.data[i];
			speed_data[i+12]	=rightback_vel.data[i];
		}

		//在写入串口的左右轮速度数据后加入”/r/n“
		speed_data[16]=data_terminal0;
		speed_data[17]=data_terminal1;
		
		
		if(serial_flag == true)
		{
			std::cout<<"控制信息发送测试1：leftfront_vel : "<< (leftfront_vel.value) <<"  rightfront_vel : " << (rightfront_vel.value) <<std::endl;
			std::cout<<"控制信息发送测试2：leftback_vel : "<< (leftback_vel.value) <<"  rightback_vel : " << (rightback_vel.value) <<std::endl;
			if(serial_lock == 1)
			{
			    while(serial_lock == 1)	;
			}

			serial_lock =3;
			memset(rec_buffer, 0, sizeof(rec_buffer));
			ros_serial.write((const unsigned char*)speed_data, sizeof(speed_data));
			//usart_write(serial_fd, (const unsigned char*)speed_data, sizeof(speed_data));
			//std::cout<<"sendcheck_str.length() in cmd_vel calllback is "<<sendcheck_str.length()<<std::endl;
			//ros::Duration(2);
			usart_read(serial_fd, rec_buffer, sendcheck_str.length());    //获取串口发送来的数据
			std::cout<<"rec_buffer in cmd_vel calllback is "<<rec_buffer<<std::endl;
			if( (strcmp(rec_buffer, sendcheck_str.c_str())) != 0)
			{
				std::cout<<"baa in cmd_vel calllback!"<<std::endl;
			      memset(rec_buffer, 0, sizeof(rec_buffer));
			      //usart_write(serial_fd, speed_data, sizeof(speed_data));
			      //ros::Duration(1);
			      usart_read(serial_fd, rec_buffer, sendcheck_str.length());    //获取串口发送来的数据
			      if( (strcmp(rec_buffer, sendcheck_str.c_str())) != 0)
						serial_flag=false;
			}
			serial_lock =4;
		}else ROS_ERROR("The serial port has not been opened yet. Please plug in the serial cable to connect ! ");

	}
	int base_controller2::usart_reconnect(void)
	{
		int len=-1;
		char reply_str[20]={0};
		
		if(serial_flag == false)//串口未打开或者已断开
		{
			serial_fd = usart_open(serial_port);//USB口号  /dev/ttyUSBn
			if(serial_fd <0 ) return 2;
			//if( (usart_set(serial_fd, 115200)) < 0) return 3;
			
			std::string temp;
			std::cout<<"请按下任意按键，准备给下位机发数据......";
			std::getline(std::cin, temp);
			
			ros::Duration(1);
			len=usart_write(serial_fd, (const unsigned char*)handshake_str.c_str(), handshake_str.length());
			//ros::Duration(2);
			len = usart_read(serial_fd, reply_str, handcheck_str.length());    //获取串口发送来的数据
			//std::cout<<"Serial connection reply_str : "<<reply_str<< std::endl;
			if(len < 0) return 5;

			//比较参数1和参数（1、若参数1>参数2，返回正数；2、若参数1<参数2，返回负数；3、若参数1=参数2，返回0；） 
			if( (strcmp(reply_str, handcheck_str.c_str())) == 0)
			{
				serial_flag = true;
				std::cout<<"Serial port connection between "<<handshake_str<<" and "<<handcheck_str<<" is successful on "<<serial_port<<std::endl;
				return 1;
			}else{ 
				std::cout<<"Serial connection failed. Please try again after inserting and unplugging！"<< std::endl;
				serial_flag = false;//连接失败
				return 0;
			}
		}
	}
	int base_controller2::usart_set(int fd, int baude, int c_flow, int bits, char parity, int stop)
	{
		fd = -1;
		try{
			ros_serial.setPort(serial_port);
			ros_serial.setBaudrate(baude);
			ros_serial.setFlowcontrol(serial::flowcontrol_none);
			ros_serial.setBytesize(serial::eightbits);
			ros_serial.setStopbits(serial::stopbits_one);
			ros_serial.setParity(serial::parity_none);
			serial::Timeout to = serial::Timeout::simpleTimeout(10000);
			ros_serial.setTimeout(to);
			//ros_serial.open();
			fd = 1;
			
		}catch (serial::IOException& e){
			ROS_ERROR("Set port paramn error: %s !", e.what());
			serial_flag = false;
			fd = -1;
		}

		return fd;
	}
	int base_controller2::usart_open(const std::string port_name)
	{
		int fd=-1;
		//assert(port_name.c_str());//检测串口路径是否存在

		if(ros_serial.isOpen()){
			//ROS_INFO("Serial Port opened on %s ", port_name);
			fd = 1;
		}else{
			std::cout<<"Immediately open the serial port:  "<<(port_name)<<" baudrate: "<< (serial_baud)<<std::endl;
			while((fd<0)&&(ros::ok()))
			{/*打开串口*/
				try{
						fd = usart_set(serial_fd, serial_baud);
						//ros_serial.setPort(port_name);
						ros_serial.open();
						fd = 1;
					} catch (serial::SerialException &e)
					{
						ROS_ERROR("Open the serial port error: %s !", e.what());
						fd = -1;
						serial_flag = false;
					}
			}
		}

		return fd;
	}
	int base_controller2::usart_close(int fd)
	{
		ros_serial.close();

		/*可以在这里做些清理工作*/

		return 0;
	}
	int base_controller2::usart_read(int fd, char* r_buf, int len)
	{
		fd = -1;
		
		if( ! ros_serial.available())
		    ros::Duration(2);
		//if(ros_serial.available())
		{
			try{
				std::string serial_data;
				//获取串口数据
				//ros_serial.flush();
				fd =ros_serial.readline(serial_data, len, "\n");    //获取串口发送来的数据
				ros_serial.flush();
				strcpy(r_buf, serial_data.c_str()); 			//保存串口发送来的数据			
				ROS_INFO_STREAM("Reading from serial port");
				//fd = serial_data.size();
			}catch(serial::IOException& e){
				ROS_ERROR("Usart read error: %s !", e.what());
				serial_flag = false;
			}
		}//else fd = -1;
		
		return fd;	
		
	}
	int base_controller2::usart_write(int fd, const unsigned char* w_buf, int len)
	{
		fd = -1;
		//写入数据到串口
		try{
			ros_serial.flush();
			ros_serial.write(w_buf, len);
			ros_serial.flush();
			fd = 1;
		}catch(serial::IOException& e){
			ROS_ERROR("Write the serial port error: %s !", e.what());
			serial_flag = false;
			fd = -1;
		}
		
		return fd;
	}
	void base_controller2::odometry_publish()
	{
		int rec_len=-1;
		double linear_x=0, linear_y=0;
		double vel1=0, vel2=0, vel3=0, vel4=0;
		//char rec_buf[30];
		
// 		while(ros::ok())
// 		{
			usart_reconnect();
			memset(rec_buffer, 0, sizeof(rec_buffer));
			//memset(rec_buf, 0, 30);
			rec_len=-1;
			if(serial_lock == 3)
			{
			    while(serial_lock == 3)	;
			}
			serial_lock = 1;
			rec_len = usart_read(serial_fd, rec_buffer, uploading_len);    //获取串口发送来的数据
			//rec_len = usart_read(serial_fd, rec_buf, 20);    //获取串口发送来的数据
			//std::cout<<"串口接收调试，已接收"<< rec_len <<"字节信息 : "<<rec_buffer<<std::endl;
			
			if(rec_len == uploading_len ) 
			{
				usart_write(serial_fd, (const unsigned char*)getreply_str.c_str(), getreply_str.length());
				serial_lock = 2;
				std::string reply_str;
				//std::cout<<"串口接收调试，已接收"<<rec_len<<"字节信息！"<<std::endl;
				
				for(int i=0;i<4;i++)//提取前后车轮的里程
				{
					leftfront_odom.data[i]		=rec_buffer[i];
					rightfront_odom.data[i]	=rec_buffer[i+4];
					leftback_odom.data[i]		=rec_buffer[i+8];
					rightback_odom.data[i]		=rec_buffer[i+12];
					anglevel_odom.data[i]		=rec_buffer[i+16];
					angleyaw_odom.data[i]		=rec_buffer[i+20];
				}
				std::cout<<"串口已接收1 leftfront_odom ："<< (leftfront_odom.value) << " rightfront_odom ：" << (rightfront_odom.value) <<std::endl;
				std::cout<<"串口已接收2 leftback_odom ："<< (leftback_odom.value) <<" rightback_odom ："<< (rightback_odom.value)<<std::endl;
				std::cout<<"串口已接收3 anglevel_odom ："<< (anglevel_odom.value)  <<" angleyaw_odom ："<< (angleyaw_odom.value)<<std::endl;
				
				//将相关信息数据缩小1000倍，将单位mm转换成m
				leftfront_odom.value	/=convert_ratio; 
				rightfront_odom.value	/=convert_ratio; 
				leftback_odom.value	/=convert_ratio; 
				rightback_odom.value	/=convert_ratio; 
				
				//计算odom数据：X, Y坐标和x, y线速度				
				linear_x = ( (cos(M_PI/4.0))/4.0 )*(-leftfront_odom.value+rightfront_odom.value-leftback_odom.value+rightback_odom.value);
				linear_y = ( (cos(M_PI/4.0))/4.0 )*(leftfront_odom.value+rightfront_odom.value+leftback_odom.value+rightback_odom.value);
				odom_x = odom_x + linear_x;
				odom_y = odom_y + linear_y;
				linear_x = linear_x/sampling_tim;
				linear_y = linear_y/sampling_tim;
				
				//里程计的偏航角需要转换成四元数才能发布
				odom_quat = tf::createQuaternionMsgFromYaw(anglevel_odom.value);//将偏航角转换成四元数
				
				//载入坐标（tf）变换时间戳
				odom_trans.header.stamp = ros::Time::now();
				//发布坐标变换的父子坐标系
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_footprint";
				//tf位置数据：x,y,z,方向
				odom_trans.transform.translation.x = odom_x;
				odom_trans.transform.translation.y = odom_y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
				//发布tf坐标变化
				odom_broadcaster.sendTransform(odom_trans);
			
				//载入里程计时间戳
				odom.header.stamp = ros::Time::now(); 
				//里程计的父子坐标系
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_footprint";       
				//里程计位置数据：x,y,z,方向
				odom.pose.pose.position.x = odom_x;     
				odom.pose.pose.position.y = odom_y;
				odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation = odom_quat;       
				//载入线速度和角速度
				odom.twist.twist.linear.x = linear_x;
				odom.twist.twist.linear.y = linear_y;
				odom.twist.twist.angular.z = anglevel_odom.value;    
				//发布里程计
				odom_pub.publish(odom);
			}//else serial_flag = false;
			serial_lock = 2;
			//ros::spinOnce();//周期执行
			
// 		}
		
	}
	void* base_controller2::getOdometry_thread(void* args)
	{
		base_controller2 *p = (base_controller2 *)args;

		ROS_INFO("Thread of getOdometry_thread is begin ! ");
		ros::Rate loop_rate(50);//设置周期休眠时间
		while(ros::ok())
		{
			p->odometry_publish();
			loop_rate.sleep();//周期休眠
		}
	}



	
	
	
	
	
}
