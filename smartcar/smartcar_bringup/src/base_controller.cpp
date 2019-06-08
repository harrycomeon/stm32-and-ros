#include "smartcar_bringup/base_controller.h"

namespace smartcar
{
	base_controller::base_controller()
	{
 		handle.getParam("/smartcar_controller/serial_port", serial_port);
		handle.getParam("/smartcar_controller/serial_baud", serial_baud);
		handle.getParam("/smartcar_controller/serial_timeout", serial_timeout);
		
		handle.getParam("/smartcar_controller/convert_ratio", convert_ratio);
		handle.getParam("/smartcar_controller/wheel_distance", wheel_dis);
		handle.getParam("/smartcar_controller/HSF2STM", HSF2STM);
		HSF2STM=HSF2STM+"\r\n";
		handle.getParam("/smartcar_controller/FCS2STM", FCS2STM);
		FCS2STM=FCS2STM+"\r\n";
		handle.getParam("/smartcar_controller/HSF2ROS", HSF2ROS);
		handle.getParam("/smartcar_controller/FCS2ROS", FCS2ROS);
		handle.getParam("/smartcar_controller/uploading_len", uploading_len);
		handle.getParam("/smartcar_controller/sampling_tim", sampling_tim);

		serial_flag = false;
		serial_lock = 0;
		serial_fd=-1;
		odom_x = 0;
		odom_y = 0;
		leftfront_lastodom = 0;
		rightfront_lastodom = 0;
		leftback_lastodom = 0;
		rightback_lastodom = 0;
		odom_init = true;
		rec_buffer = new char[uploading_len];
		memset(rec_buffer, 0, sizeof(rec_buffer));
		serial_check = 0;
		
		DF2STM[0] = 'D';
		//在写入串口的左右轮速度数据后加入”/r/n“
		DF2STM[17]=0x0d;  //“/r"字符
		DF2STM[18]=0x0a;  //“/n"字符
	}
	base_controller::~base_controller()
	{
		delete[] rec_buffer;
		usart_close(serial_fd);
	}
	void base_controller::init()
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
		
		cmd_sub = handle.subscribe("cmd_vel", 5, &base_controller::cmd_velCallback, this); //订阅/cmd_vel主题
		odom_pub= handle.advertise<nav_msgs::Odometry>("odom", 5); //定义要发布/odom主题
		
// 		int ret2=0;
// 		ret2 = pthread_create(&getOdom_t, NULL, getOdometry_thread, this);
// 		if (ret2 != 0) ROS_ERROR("The getOdometry_thread pthread_create error: error_code=%d", ret2); 
	}
	void base_controller::cmd_velCallback(const geometry_msgs::Twist& cmd_input)
	{
		float xlinear_temp=0, ylinear_temp=0, angular_temp=0;//暂存的线速度和角速度
		float theta = 31.83*M_PI/180.0;
		float theta_bie = 13.07*M_PI/180.0;
		
		// Copy the velocities (m/s)
		angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
		xlinear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度,m/s
		ylinear_temp = cmd_input.linear.y ;//获取/cmd_vel的线速度,m/s

		//if(angular_temp>=1)  angular_temp = 1;
		//if(angular_temp<=-1) angular_temp = -1;
		
		if(xlinear_temp>=2.048)
			xlinear_temp = 2.048;
		if(xlinear_temp<=-2.048)
			xlinear_temp = -2.048;
		
		if(ylinear_temp>=2.048)
			ylinear_temp = 2.048;
		if(ylinear_temp<=-2.048)
			ylinear_temp = -2.048;
		
		// Calculate individual wheel linear velocities (m/s)将转换好的小车速度分量为左右轮速度
		leftfront_vel.value = ( xlinear_temp/sin(theta) - ylinear_temp/cos(theta) - angular_temp * wheel_dis)/cos(theta_bie);
		rightfront_vel.value= (-xlinear_temp/sin(theta) - ylinear_temp/cos(theta) - angular_temp * wheel_dis)/cos(theta_bie);
		leftback_vel.value  = ( xlinear_temp/sin(theta) + ylinear_temp/cos(theta) - angular_temp * wheel_dis)/cos(theta_bie);
		rightback_vel.value = (-xlinear_temp/sin(theta) + ylinear_temp/cos(theta) - angular_temp * wheel_dis)/cos(theta_bie);

		////存入数据到要发布的左右轮速度消息 ,并放大ratio倍，即单位mm/s
		leftfront_vel.value = leftfront_vel.value *convert_ratio;
		rightfront_vel.value = rightfront_vel.value *convert_ratio;
		leftback_vel.value = leftback_vel.value *convert_ratio;
		rightback_vel.value = rightback_vel.value *convert_ratio;
		

		std::cout<<"[1] leftfront_vel.value = "<<leftfront_vel.value<<"   [2]rightfront_vel.value"<<rightfront_vel.value<<std::endl;
		std::cout<<"[3] leftback_vel.value = "<<leftback_vel.value<<"   [4]rightback_vel.value"<<rightback_vel.value<<std::endl;
		//将左右轮速度存入数组中发送给串口
		DF2STM[0] = 'D';
		for(int i = 0 ; i < 4 ; i++)
		{
			DF2STM[i+1]		=leftfront_vel.data[i];
			DF2STM[i+4+1]		=rightfront_vel.data[i];
			DF2STM[i+8+1]		=leftback_vel.data[i];
			DF2STM[i+12+1]		=rightback_vel.data[i];
		}
		if(serial_flag == true)
		{
			usart_write(serial_fd, DF2STM, sizeof(DF2STM));
			
		}else ROS_ERROR("The serial port has not been opened yet. Please plug in the serial cable to connect ! ");

	}
	int base_controller::usart_reconnect(void)
	{
		int len=-1;
		char reply_str[20]={0};
		
		if(serial_flag == false)//串口未打开或者已断开
		{
			serial_fd = usart_open(serial_port);//USB口号  /dev/ttyUSBn
			if(serial_fd <0 ) return 2;
			if( (usart_set(serial_fd, serial_baud)) < 0) return 3;
			
// 			std::string temp;
// 			std::cout<<"请按下任意按键，准备给下位机发数据......";
// 			std::getline(std::cin, temp);
			
			ros::Duration(1);
			len=usart_write(serial_fd, HSF2STM.c_str(), HSF2STM.length());
			if(len > 0)
			{
			      len = usart_read(serial_fd, reply_str, HSF2ROS.length());    //获取串口发送来的数据
			      if(len < 0) return 5;
			}

			//比较参数1和参数（1、若参数1>参数2，返回正数；2、若参数1<参数2，返回负数；3、若参数1=参数2，返回0；） 
			if( (strcmp(reply_str, HSF2ROS.c_str())) == 0)
			{
				serial_flag = true;
				std::cout<<"Serial port connection is successful on "<<ttyname(serial_fd)<<std::endl;
				return 1;
			}else{ 
				std::cout<<"Serial connection failed. Please try again after inserting and unplugging！"<< std::endl;
				serial_flag = false;//连接失败
				return 0;
			}
		}
	}
	int base_controller::usart_set(int fd, int baude, int c_flow, int bits, char parity, int stop)
	{
		struct termios options;

		/*获取终端属性*/
		if(tcgetattr(fd,&options) < 0)
		{
			ROS_ERROR("Tcgetattr error: %s !", strerror(errno));
			return -1;
		}
		bzero(&options, sizeof( options ) ); 
		
		/*设置输入输出波特率，两者保持一致*/
		switch(baude)
		{
			case 4800:cfsetispeed(&options,B4800);cfsetospeed(&options,B4800);break;
			case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break; 
			case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break; 
			case 460800: cfsetispeed(&options, B460800); cfsetospeed(&options, B460800); break; 
			default: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break; 
		}

		/*设置控制模式*/
		options.c_cflag |= CLOCAL;//保证程序不占用串口
		options.c_cflag |= CREAD;//保证程序可以从串口中读取数据
		
		/*设置数据流控制*/
		switch(c_flow)
		{
			case 0:options.c_cflag &= ~CRTSCTS;break;//不进行流控制
			case 1:options.c_cflag |= CRTSCTS;break;//进行硬件流控制
			case 2:options.c_cflag |= IXON|IXOFF|IXANY;break;//进行软件流控制
			default:options.c_cflag &= ~CRTSCTS;break;//不进行流控制
		}

		/*设置数据位*/
		switch(bits)
		{
			case 5:options.c_cflag &= ~CSIZE;options.c_cflag |= CS5;break;//屏蔽其它标志位
			case 6:options.c_cflag &= ~CSIZE;options.c_cflag |= CS6;break;//屏蔽其它标志位
			case 7:options.c_cflag &= ~CSIZE;options.c_cflag |= CS7;break;
			case 8:options.c_cflag &= ~CSIZE;options.c_cflag |= CS8;break;
			default:options.c_cflag &= ~CSIZE;options.c_cflag |= CS8;break;
		}
		
		/*设置校验位*/
		switch(parity)
		{
			/*无奇偶校验位*/
			case 'n':
			case 'N':
				options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
				//options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
				break;
			/*设为空格,即停止位为2位*/
			case 's':
			case 'S':
				options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
				options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
				break;
			/*设置奇校验*/
			case 'o':
			case 'O':
				options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
				options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
				options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
				options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
				break;
			/*设置偶校验*/
			case 'e':
			case 'E':
				options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
				options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
				options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
				options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
				break;
			default:/*无奇偶校验位*/
				options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
				options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
				break;
		}
		
		/*设置停止位*/
		switch(stop)
		{
			case 1:options.c_cflag &= ~CSTOPB;break;//CSTOPB：使用1位停止位
			case 2:options.c_cflag |= CSTOPB;break;//CSTOPB：使用两位停止位
			default:options.c_cflag &= ~CSTOPB;break;//CSTOPB：使用1位停止位
		}
		
		/*设置输出模式为原始输出*/
		options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

		/*设置本地模式为原始模式*/
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		/*
		*ICANON：允许规范模式进行输入处理
		*ECHO：允许输入字符的本地回显
		*ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
		*ISIG：允许信号
		*/
		
		/*设置等待时间和最小接受字符*/
		options.c_cc[VTIME] = 0;//可以在select中设置
		options.c_cc[VMIN] = 1;//最少读取一个字符

		/*如果发生数据溢出，只接受数据，但是不进行读操作*/
		//tcflush(fd, TCIFLUSH);
		tcflush(fd, TCIOFLUSH);//清除输入、输出队列缓存
		
		/*激活配置*/
		if(tcsetattr(fd,TCSANOW,&options) < 0)
		{
			ROS_ERROR("Tcsetattr failed: %s !" , strerror(errno));
			return -1;
		}
		return fd;
	}
	int base_controller::usart_open(const std::string port_name)
	{
		int fd=-1;
		//assert(port_name.c_str());//检测串口路径是否存在

		std::cout<<"Immediately open the serial port:  "<<(serial_port)<<" baudrate: "<< (serial_baud)<<std::endl;
		//usart_close(serial_fd);
		while((fd<0)&&(ros::ok()))
		{/*打开串口*/
		      /*设置阻塞模式与非阻塞模式,第一种方法*/
			fd = open(port_name.c_str(), O_RDWR|O_NOCTTY);//阻塞
			//fd = open(port_name.c_str(), O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK);//非阻塞模式
			if(fd == -1)
			{
				ROS_ERROR_ONCE( "Usart open error:  %s !", strerror(errno));
				ROS_ERROR_ONCE("Please plug in the serial port of the stm32 master !");
			}
		}
		
		/*设置阻塞模式与非阻塞模式,第二种方法*/
		//设定阻塞模式，即使前面在open串口设备时设置的是非阻塞的，这里设为阻塞后，以此为准  
// 		if(fcntl(fd, F_SETFL, 0) < 0)
// 		{
// 			ROS_ERROR("Fcntl failed: %s !", strerror(errno));
// 			return -1;
// 		}
		//设定非阻塞，覆盖前面open的属性
// 		if(fcntl(fd,F_SETFL,FNDELAY) < 0)
// 		{
// 			ROS_ERROR("Fcntl failed: %s !", strerror(errno));
// 			return -1;
// 		}
		return fd;
	}
	int base_controller::usart_close(int fd)
	{
		assert(fd);
		close(fd);

		/*可以在这里做些清理工作*/

		return 0;
	}
	int base_controller::safe_write(int fd, const char* ptr, int n)
	{
		int  nleft;
		int nwritten;
		//const char *ptr=vptr;

		//ptr = vptr;
		nleft = n;

		while(nleft > 0)
		{
			if((nwritten = write(fd, ptr, nleft)) <= 0)
			{
				if(nwritten < 0 && errno == EINTR)//被信号中断
					nwritten = 0;
				else
					return -1;
			}
			nleft -= nwritten;
			ptr   += nwritten;
		}
		return	(n);
	}

	int base_controller::safe_read(int fd, char* ptr, int n)
	{
		size_t nleft;
		ssize_t nread;
		//char *ptr;

		//ptr=vptr;
		nleft=n;

		while(nleft > 0)
		{
			if((nread = read(fd,ptr,nleft)) < 0)
			{
				if(errno == EINTR)//被信号中断
					nread = 0;
				else
					return -1;
					//return  0;
			}else if(nread == 0)
				break;
	
			nleft -= nread;//read成功后，剩余要读取的字节自减
			ptr += nread;//指针向后移，避免后读到的字符覆盖先读到的字符
		}
		
		return (n-nleft);
	}
	int base_controller::usart_read(int fd, char* r_buf, int len)
	{
		ssize_t cnt = -1;
		fd_set rfds;
		struct timeval time;

		/*将文件描述符加入读描述符集合*/
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		/*设置超时为serial_timeout 秒*/
		time.tv_sec = serial_timeout;
		time.tv_usec = 0;

		//实现串口的多路I/O
		//Linux下直接用read读串口可能会造成堵塞，或数据读出错误。
		//然而用select先查询com口，如果文件存在就用read去读;如果没有,则等待time时间,超时就退出
		 int serial_ret = select(fd+1, &rfds, NULL, NULL, &time);//这里是serial_timeout时间阻塞模式
		 //int serial_ret = select(fd+1, &rfds, NULL, NULL, NULL);//这里是serial_timeout时间阻塞模式
		 switch(serial_ret)
		{
			case -1:ROS_ERROR("Usart select error: %s !", strerror(errno));return -1;
			case 0 :ROS_INFO_STREAM_THROTTLE_NAMED(7, "Usart wait time over: %s !", strerror(errno));return 0;
			default:{
					cnt = safe_read(fd, r_buf, len);
					tcflush(fd, TCIFLUSH);//清除输入队列缓存
					if(cnt == -1)
					{
						ROS_ERROR("Usart read error: %s !", strerror(errno));
						serial_flag = false;
						return -1;
					}
			}
		}
		return cnt;	
		
	}
	int base_controller::usart_write(int fd, const char* w_buf, int len)
	{
		ssize_t cnt = -1;
		
		fd_set rfds;
		struct timeval time;

		/*将文件描述符加入读描述符集合*/
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		/*设置超时为serial_timeout 秒*/
		time.tv_sec = serial_timeout;
		time.tv_usec = 0;
		
		 tcflush(fd, TCOFLUSH);//清除输出队列缓存
		if(select(fd+1, NULL, &rfds, NULL, &time) > 0) //这里是serial_timeout时间阻塞模式
		{
		    cnt = safe_write(fd, w_buf, len);
		    tcflush(fd, TCOFLUSH);//清除输出队列缓存
		    if(cnt == -1)
		    {
			    ROS_ERROR("Usart write error: %s !", strerror(errno));
			    serial_flag = false;
			    return -1;
		    }
		}
		return cnt;
	}
	void base_controller::odometry_publish()
	{
		int rec_len=-1;
		double linear_x=0, linear_y=0;
		double x_odom_delt = 0;
		double y_odom_delt = 0;
		double omiga_delt = 0,omiga = 0;
		float theta = 31.83*M_PI/180.0;
		float theta_bie = 13.07*M_PI/180.0;

		usart_reconnect();
		rec_len=-1;

		rec_len = usart_read(serial_fd, rec_buffer, uploading_len);    //获取串口发送来的数据
		if(rec_buffer[0] == 'D' && rec_len == uploading_len)
		{
			//serial_lock = 2;
			
			for(int i=0;i<4;i++)//提取前后车轮的里程
			{
				leftfront_odom.data[i]		=rec_buffer[i+1];
				rightfront_odom.data[i]		=rec_buffer[i+4+1];
				leftback_odom.data[i]		=rec_buffer[i+8+1];
				rightback_odom.data[i]		=rec_buffer[i+12+1];
				anglevel_odom.data[i]		=rec_buffer[i+16+1];
				angleyaw_odom.data[i]		=rec_buffer[i+20+1];
			}
		
			//将相关信息数据缩小1000倍，将单位mm转换成m
			leftfront_odom.value		/=convert_ratio; 
			rightfront_odom.value	/=convert_ratio; 
			leftback_odom.value		/=convert_ratio; 
			rightback_odom.value	/=convert_ratio; 
					
//			std::cout<<"串口已接收1 leftfront_odom ："<< (leftfront_odom.value) << " rightfront_odom ：" << (rightfront_odom.value) <<std::endl;
 //				std::cout<<"串口已接收2 leftback_odom ："<< (leftback_odom.value) <<" rightback_odom ："<< (rightback_odom.value)<<std::endl;
// 				std::cout<<"串口已接收3 anglevel_odom ："<< (anglevel_odom.value)  <<" angleyaw_odom ："<< (angleyaw_odom.value)<<std::endl;
			//累计里程差值计算
			if(odom_init)
			{
				x_odom_delt = leftfront_odom.value-rightfront_odom.value+leftback_odom.value-rightback_odom.value;
				y_odom_delt = -leftfront_odom.value-rightfront_odom.value+leftback_odom.value+rightback_odom.value;
				omiga_delt  = -(leftfront_odom.value+rightfront_odom.value+leftback_odom.value+rightback_odom.value);
				odom_init = false;
			}else{
				x_odom_delt = (leftfront_odom.value-leftfront_lastodom)-(rightfront_odom.value-rightfront_lastodom)+
						(leftback_odom.value-leftback_lastodom)-(rightback_odom.value-rightback_lastodom);
				y_odom_delt = -(leftfront_odom.value-leftfront_lastodom)-(rightfront_odom.value-rightfront_lastodom)+
						(leftback_odom.value-leftback_lastodom)+(rightback_odom.value-rightback_lastodom);
				omiga_delt = -(leftfront_odom.value-leftfront_lastodom)-(rightfront_odom.value-rightfront_lastodom)-
						(leftback_odom.value-leftback_lastodom)-(rightback_odom.value-rightback_lastodom);
			}
			leftfront_lastodom = leftfront_odom.value;
			rightfront_lastodom = rightfront_odom.value;
			leftback_lastodom = leftback_odom.value;
			rightback_lastodom = rightback_odom.value;	

			//计算odom数据：X, Y坐标和x, y线速度				
			linear_x = x_odom_delt*cos(theta_bie)*sin(theta)/4.0;
			linear_y = y_odom_delt*cos(theta_bie)*cos(theta)/4.0;
			odom_x = odom_x + linear_x;
			odom_y = odom_y + linear_y;
			linear_x = linear_x/sampling_tim;
			linear_y = linear_y/sampling_tim;
			omiga_delt = omiga_delt*cos(theta_bie)/(4*wheel_dis);
			omiga = omiga + omiga_delt;
			//omiga = anglevel_odom.value;
			
			//里程计的偏航角需要转换成四元数才能发布
			odom_quat = tf::createQuaternionMsgFromYaw(omiga);//将偏航角转换成四元数
			
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
			odom.twist.twist.angular.z = omiga_delt/sampling_tim;    
			//发布里程计
			odom_pub.publish(odom);
		}
		serial_lock = 2;
		
	}
	void base_controller::run(void)
	{
	  ros::Rate r(20); // 10 hz
	  while(ros::ok())
	  {
		  odometry_publish();
		  ros::spinOnce();
		  r.sleep();
	  }
	  DF2STM[0] = 'Q';
	  usart_write(serial_fd, DF2STM, sizeof(DF2STM));
	  
	}
	void* base_controller::getOdometry_thread(void* args)
	{
		base_controller *p = (base_controller *)args;

		ROS_INFO("Thread of getOdometry_thread is begin ! ");
		ros::Rate loop_rate(50);//设置周期休眠时间
		while(ros::ok())
		{
			p->odometry_publish();
			loop_rate.sleep();//周期休眠
		}
	}



	
	
	
	
	
}
