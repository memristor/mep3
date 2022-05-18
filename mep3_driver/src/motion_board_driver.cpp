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

#include "mep3_driver/motion_board_driver.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <tuple>

namespace mep3_driver
{
MotionBoardDriver::MotionBoardDriver()
{
  encoder_left_ = 0;
  encoder_right_ = 0;
}

MotionBoardDriver::~MotionBoardDriver() { halt(); }

int MotionBoardDriver::init()
{
  pthread_mutex_init(&data_lock_, NULL);

  if ((canbus_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW_FILTER)) < 0) {
    std::cerr << "Socket error!\n";
    return 1;
  }

  filter_.can_id = CAN_ENCODER_ID;
  filter_.can_mask = 0x1FFFFFFF;

  if (setsockopt(canbus_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter_, sizeof(filter_)) < 0) {
    std::cerr << "Can filter setsockopt failed!\n";
    return 1;
  }

  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  if (setsockopt(canbus_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    std::cerr << "Can filter setsockopt failed!\n";
    return 1;
  }

  memset(&ifr_, 0, sizeof(ifr_));
  snprintf(ifr_.ifr_name, sizeof(ifr_.ifr_name), "can0");
  if (ioctl(canbus_socket_, SIOCGIFINDEX, &ifr_) < 0) {
    std::cerr << "ioctl fail! Does can interface 'can0' exist?\n";
    return 1;
  }

  address_.can_family = AF_CAN;
  address_.can_ifindex = ifr_.ifr_ifindex;

  if (bind(canbus_socket_, (struct sockaddr *)&address_, sizeof(address_)) < 0) {
    std::cerr << "Bind error!\n";
    return 1;
  }

  reset_encoders();

  return 0;
}

void MotionBoardDriver::start()
{
  int ret = pthread_create(&canbus_receive_thread_, NULL, canbus_thread_entry, this);
  if (ret != 0) {
    std::cerr << "Failed to create canbus receive thread!\n";
  }
  encoder_report_enable();
}

void MotionBoardDriver::halt()
{
  encoder_report_disable();

  pthread_cancel(canbus_receive_thread_);
  pthread_join(canbus_receive_thread_, NULL);

  close(canbus_socket_);
}

void * MotionBoardDriver::canbus_thread_entry(void * ptr)
{
  return reinterpret_cast<MotionBoardDriver *>(ptr)->canbus_receive_function();
}

void * MotionBoardDriver::canbus_receive_function()
{
  for (;;) {
    struct can_frame frame;
    const int nbytes = read(canbus_socket_, &frame, sizeof(struct can_frame));

    if (nbytes > 0) {
      if (frame.can_id == CAN_ENCODER_ID && frame.can_dlc == 8) {
        const int32_t left = protocol_unpack_int32(&frame.data[0]);
        const int32_t right = protocol_unpack_int32(&frame.data[4]);

        pthread_mutex_lock(&data_lock_);
        encoder_left_ = left;
        encoder_right_ = right;
        pthread_mutex_unlock(&data_lock_);
      }
    }
  }
  return NULL;
}

int32_t MotionBoardDriver::protocol_unpack_int32(uint8_t * buffer)
{
  int32_t val = 0;
  for (int i = 0; i < 4; i++) {
    val |= ((uint32_t)buffer[i] << 8 * (3 - i));
  }

  return val;
}

void MotionBoardDriver::protocol_pack_int16(uint8_t * buffer, int16_t val)
{
  buffer[1] = val & 0xFF;
  buffer[0] = (val >> 8) & 0xFF;
}

void MotionBoardDriver::protocol_pack_float(uint8_t * buffer, float val)
{
  uint32_t as_integer = *(reinterpret_cast<uint32_t *>(&val));  // access float on byte level
  for (int i = 3; i >= 0; i--) {
    buffer[i] = (as_integer >> 8 * (3 - i)) & 0xFF;
  }
}

std::tuple<int32_t, int32_t> MotionBoardDriver::get_encoders()
{
  pthread_mutex_lock(&data_lock_);
  int32_t left = encoder_left_;
  int32_t right = encoder_right_;
  pthread_mutex_unlock(&data_lock_);

  return std::make_tuple(left, right);
}

void MotionBoardDriver::reset_encoders()
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 1;
  frame.data[0] = CMD_RESET_ENCODERS;
  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::encoder_report_enable()
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 1;
  frame.data[0] = CMD_ENABLE_ENCODER_REPORT;
  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::encoder_report_disable()
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 1;
  frame.data[0] = CMD_DISABLE_ENCODER_REPORT;
  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_setpoints(int16_t linear, int16_t angular)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_SETPOINTS;

  protocol_pack_int16(&frame.data[1], linear);
  protocol_pack_int16(&frame.data[3], angular);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_kp_linear(float val)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_KP_LINEAR;
  protocol_pack_float(&frame.data[1], val);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_ki_linear(float val)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_KI_LINEAR;
  protocol_pack_float(&frame.data[1], val);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_kd_linear(float val)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_KD_LINEAR;
  protocol_pack_float(&frame.data[1], val);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_kp_angular(float val)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_KP_ANGULAR;
  protocol_pack_float(&frame.data[1], val);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_ki_angular(float val)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_KI_ANGULAR;
  protocol_pack_float(&frame.data[1], val);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

void MotionBoardDriver::set_kd_angular(float val)
{
  struct can_frame frame;
  frame.can_id = CAN_BASE_ID;
  frame.can_dlc = 5;
  frame.data[0] = CMD_SET_KD_ANGULAR;
  protocol_pack_float(&frame.data[1], val);

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}
}  // namespace mep3_driver
