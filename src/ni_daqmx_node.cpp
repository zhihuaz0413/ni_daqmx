// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "config.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "niforce.pb.h"
#include "ni_daqmx.hpp"


using namespace std::chrono_literals;

class niDaqmx : public rclcpp::Node {
 public:
  niDaqmx()
  : Node("ni_daqmx") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("ni_force", 10);
    timer_ = this->create_wall_timer(
        10ms, std::bind(&niDaqmx::timer_callback, this));
    std::filesystem::path ws_dir = std::filesystem::current_path();
    std::string config_file = ws_dir.string() + "/src/ni_daqmx/config/config.prototxt";
    int fd = open(config_file.c_str(), O_RDONLY);
    if(fd >= 0) {
      google::protobuf::io::FileInputStream fileInput(fd);
      fileInput.SetCloseOnDelete(true);
      google::protobuf::TextFormat::Parse(&fileInput, &config_);
    }
  }

  static void sensor_callback(int nb_channels, int samplingSize, double* data,
               uint64_t timestamp_sec, uint32_t timestamp_nsec, void* obj){
    niforce_.mutable_force()->set_x(data[0]);
    niforce_.mutable_force()->set_y(data[1]);
    niforce_.mutable_force()->set_z(data[2]);
    niforce_.mutable_torque()->set_x(data[3]);
    niforce_.mutable_torque()->set_y(data[4]);
    niforce_.mutable_torque()->set_z(data[5]);
    niforce_.set_timestamp(std::to_string(timestamp_sec+timestamp_nsec*1e-9));
    //print only once every 1000 samples
    // static int j=0;
    // if(j++%1000==0) {
    //   printf("\xd \bFx=%.3f Fy=%.3f Fz=%.3f Tx=%.3f Ty=%.3f Tz=%.3f", data[0], data[1], data[2], data[3], data[4], data[5]);
    //   fflush(stdout);
    // }	
  }

 private:
  void timer_callback() {
    auto msg = std_msgs::msg::String();
    if (config_.verbose()) {
      std::string prototextOutput;
      google::protobuf::TextFormat::PrintToString(niforce_, &prototextOutput);
      std::cout << prototextOutput << std::endl;
    }
    if (niforce_.SerializeToString(&msg.data)) {
      publisher_->publish(msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Serialization wrong!!!");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  inline static ni_daqmx::NiForce niforce_;
  ni_daqmx::NiConfig config_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  NI::ATI::FT6_sensor sensor;
  sensor.start_thread();
  sensor.set_callback(niDaqmx::sensor_callback, static_cast<void*>(std::make_shared<niDaqmx>().get()));
  rclcpp::spin(std::make_shared<niDaqmx>());
  rclcpp::shutdown();
  return 0;
}
