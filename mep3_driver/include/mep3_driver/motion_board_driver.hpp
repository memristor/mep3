// Copyright 2021 Memristor Robotics
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

#ifndef MEP3_DRIVER__MOTION_BOARD_DRIVER_HPP_
#define MEP3_DRIVER__MOTION_BOARD_DRIVER_HPP_

extern "C" {
#include <linux/can.h>      // CAN network layer definitions
#include <linux/can/raw.h>  // CAN_RAW_FILTER
#include <net/if.h>         // ifreq
#include <pthread.h>        // POSIX threads, can be switched to realtime
#include <sys/ioctl.h>      // IO control
#include <sys/socket.h>     // socket library
#include <unistd.h>         // read, write to sockets, close
}

#include <cstdint>
#include <tuple>

namespace mep3_driver
{
class MotionBoardDriver
{
private:
  int32_t encoder_left_;
  int32_t encoder_right_;

  int canbus_socket_;
  struct sockaddr_can address_;
  struct ifreq ifr_;

  struct can_filter filter_;

  pthread_t canbus_receive_thread_;
  bool keep_communication_alive_;
  pthread_mutex_t data_lock_;

  static const uint32_t CAN_BASE_ID = 0x80002000;
  static const uint32_t CAN_ENCODER_ID =
    0x80002004;  // 8 because we will receive MSB due to extended ID

  static const uint32_t CMD_RESET_ENCODERS = 0x12;
  static const uint32_t CMD_ENABLE_ENCODER_REPORT = 0x13;
  static const uint32_t CMD_DISABLE_ENCODER_REPORT = 0x14;
  static const uint32_t CMD_SET_SETPOINTS = 0x01;

  static const uint32_t CMD_SET_KP_LINEAR = 0x03;
  static const uint32_t CMD_SET_KI_LINEAR = 0x05;
  static const uint32_t CMD_SET_KD_LINEAR = 0x07;

  static const uint32_t CMD_SET_KP_ANGULAR = 0x09;
  static const uint32_t CMD_SET_KI_ANGULAR = 0x0B;
  static const uint32_t CMD_SET_KD_ANGULAR = 0x0D;

  static int32_t protocol_unpack_int32(uint8_t * buffer);
  static void protocol_pack_int16(uint8_t * buffer, int16_t val);
  static void protocol_pack_float(uint8_t * buffer, float val);

public:
  MotionBoardDriver();
  ~MotionBoardDriver();

  int init();
  void start();
  void halt();
  static void * canbus_thread_entry(void * ptr);
  void * canbus_receive_function();
  std::tuple<int32_t, int32_t> get_encoders();
  void reset_encoders();
  void encoder_report_enable();
  void encoder_report_disable();
  void set_setpoints(int16_t linear, int16_t angular);

  void set_kp_linear(float val);
  void set_ki_linear(float val);
  void set_kd_linear(float val);

  void set_kp_angular(float val);
  void set_ki_angular(float val);
  void set_kd_angular(float val);
};
}  // namespace mep3_driver

#endif  // MEP3_DRIVER__MOTION_BOARD_DRIVER_HPP_
