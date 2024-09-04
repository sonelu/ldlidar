#include <ldlidar_node.h>

LD06::LD06() : Node("trilobot_lidar_node")
{
  this->declare_parameter("topic_name", "scan");
  this->declare_parameter("serial_port", "/dev/ttyS0");
  this->declare_parameter("lidar_frame", "lidar_link");
  this->declare_parameter("range_threshold", 0.005);

  std::string topic_name = this->get_parameter("topic_name").as_string();
  std::string port_name = this->get_parameter("serial_port").as_string();
  std::string lidar_frame = this->get_parameter("lidar_frame").as_string();
  double range_threshold = this->get_parameter("range_threshold").as_double();

  lidar_ = new LiPkg;
  lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

  lidar_->SetLidarFrame(lidar_frame);
  lidar_->SetRangeThreshold(range_threshold);
  // if (port_name.empty())
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Autodetecting serial port");
  //   std::vector<std::pair<std::string, std::string>> device_list;
  //   cmd_port_.GetCmdDevices(device_list);
  //   auto found = std::find_if(
  //     device_list.begin(),
  //     device_list.end(),
  //     [](std::pair<std::string, std::string> n)
  //     { return strstr(n.second.c_str(), "CP2102"); }
  //   );

  //   if (found != device_list.end())
  //   {
  //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s %s", found->first.c_str(), found->second.c_str());
  //     port_name = found->first;
  //   }
  //   else
  //   {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't find LiDAR LD06");
  //   }
  // }

  RCLCPP_INFO(get_logger(), "Using port %s", port_name.c_str());

	cmd_port_.SetReadCallback([this](const char *byte, size_t len) {
		if(lidar_->Parse((uint8_t*)byte, len))
		{
			lidar_->AssemblePacket();
		}
	});

  if(cmd_port_.Open(port_name))
  {
    RCLCPP_INFO(get_logger(), "LiDAR_LD06 started successfully");
  } 
  else 
  {
    RCLCPP_ERROR(get_logger(), "Can't open the serial port");
  }

  loop_timer_ = this->create_wall_timer(100ms, std::bind(&LD06::publishLoop, this));
}


void LD06::publishLoop()
{
  if (lidar_->IsFrameReady())
  {
    lidar_pub_->publish(lidar_->GetLaserScan());
    lidar_->ResetFrameReady();
  }
}