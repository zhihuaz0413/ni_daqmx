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
#include "fsr_interfaces/msg/ni_force.hpp"
#include "fsr_interfaces/srv/record_force.hpp"
#include "niforce.pb.h"
#include "ni_daqmx.hpp"


using namespace std::chrono_literals;

class niDaqmx : public rclcpp::Node {
 public:
  niDaqmx()
  : Node("ni_daqmx") {
    publisher_ = this->create_publisher<fsr_interfaces::msg::NiForce >("ni_force", 2);
    record_service_ = this->create_service<fsr_interfaces::srv::RecordForce>("record_force",
      std::bind(&niDaqmx::record_force, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(
        1ms, std::bind(&niDaqmx::timer_callback, this));
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
               uint64_t timestamp_sec, uint64_t timestamp_nsec, void* obj){
    niforce_.mutable_force()->set_x(data[0]);
    niforce_.mutable_force()->set_y(data[1]);
    niforce_.mutable_force()->set_z(data[2]);
    niforce_.mutable_torque()->set_x(data[3]);
    niforce_.mutable_torque()->set_y(data[4]);
    niforce_.mutable_torque()->set_z(data[5]);
    niforce_.set_timestamp(std::to_string(timestamp_sec*1e+9+timestamp_nsec));


    if (record_) {
      auto force = msg_force_.add_force();
      force->CopyFrom(niforce_);
      if (msg_force_.force_size() > 1000*6000) {
        std::string record_file = save_folder_ + "_" + std::to_string(timestamp_sec*1e+9+timestamp_nsec) + ".bin";  
        std::ofstream ofs(record_file, std::ios::out | std::ios::app);
        if (ofs.is_open()) {
          msg_force_.SerializeToOstream(&ofs);
          ofs.close();
          msg_force_.clear_force();
        }
      }
    }

    //print only once every 1000 samples
    // static int j=0;
    // if(j++%1000==0) {
    //   printf("\xd \bFx=%.3f Fy=%.3f Fz=%.3f Tx=%.3f Ty=%.3f Tz=%.3f", data[0], data[1], data[2], data[3], data[4], data[5]);
    //   fflush(stdout);
    // }	
  }

  void record_force(const std::shared_ptr<fsr_interfaces::srv::RecordForce::Request> request,
                    std::shared_ptr<fsr_interfaces::srv::RecordForce::Response> response) {
    if (request->record) {
      record_ = true;
      save_folder_ = request->folder + "/ni_daqmx/";
      if (!std::filesystem::exists(save_folder_)) {
        std::filesystem::create_directories(save_folder_);
      }
      save_folder_ += request->labels;
      msg_force_.clear_force();
      response->success = true;
    } else {
      record_ = false;
      rclcpp::Time now = this->get_clock()->now();
      std::string record_file = save_folder_ + ".bin";
      std::ofstream ofs(record_file, std::ios::out | std::ios::trunc);
      if (ofs.is_open()) {
        msg_force_.SerializeToOstream(&ofs);
        ofs.close();
        response->success = true;
      } else {
        response->success = false;
      }
    }
  }

 private:
  void timer_callback() {
    if (config_.verbose()) {
      std::string prototextOutput;
      google::protobuf::TextFormat::PrintToString(niforce_, &prototextOutput);
      std::cout << prototextOutput << std::endl;
    }
    fsr_interfaces::msg::NiForce force_msg;
    force_msg.timestamp = niforce_.timestamp();
    force_msg.force.x = niforce_.force().x();
    force_msg.force.y = niforce_.force().y();
    force_msg.force.z = niforce_.force().z();
    force_msg.torque.x = niforce_.torque().x();
    force_msg.torque.y = niforce_.torque().y();
    force_msg.torque.z = niforce_.torque().z();
    publisher_->publish(force_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  inline static ni_daqmx::NiForce niforce_;
  inline static bool record_ = false;
  inline static ni_daqmx::MsgForce msg_force_;
  inline static std::string save_folder_ = "";
  ni_daqmx::NiConfig config_;
  rclcpp::Publisher<fsr_interfaces::msg::NiForce>::SharedPtr publisher_;
  rclcpp::Service<fsr_interfaces::srv::RecordForce>::SharedPtr record_service_;
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
