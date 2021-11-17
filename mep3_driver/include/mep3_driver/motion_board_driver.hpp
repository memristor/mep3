#ifndef MOTION_BOARD_DRIVER_HPP
#define MOTION_BOARD_DRIVER_HPP

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <tuple>

extern "C" {
#include <pthread.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
}



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

        static const uint32_t CAN_BASE_ID =                 0x200;
        static const uint32_t CAN_ENCODER_ID =              0x80000204;     // 8 because we will receive MSB due to extended ID 

        static const uint32_t CMD_RESET_ENCODERS =          0x12;
        static const uint32_t CMD_ENABLE_ENCODER_REPORT =   0x13;
        static const uint32_t CMD_DISABLE_ENCODER_REPORT =  0x14;
        static const uint32_t CMD_SET_SETPOINTS =           0x01;

        static const uint32_t CMD_SET_KP_LEFT =             0x03;
        static const uint32_t CMD_SET_KI_LEFT =             0x05;
        static const uint32_t CMD_SET_KD_LEFT =             0x07;

        static const uint32_t CMD_SET_KP_RIGHT =            0x09;
        static const uint32_t CMD_SET_KI_RIGHT =            0x0B;
        static const uint32_t CMD_SET_KD_RIGHT =            0x0D;


        static int32_t protocol_unpack_int32(uint8_t *buffer);
        static void protocol_pack_int16(uint8_t *buffer, int16_t val);
        static void protocol_pack_float(uint8_t *buffer, float val);

    public:
        MotionBoardDriver();
        ~MotionBoardDriver();

        int init();        
        void start();
        void halt();
        static void* canbus_thread_entry(void *ptr);
        void* canbus_receive_function();
        std::tuple<int32_t, int32_t> get_encoders(); 
        void reset_encoders();
        void encoder_report_enable();
        void encoder_report_disable();
        void set_setpoints(int16_t left, int16_t right);
        
        void set_kp_left(float val);
        void set_ki_left(float val);
        void set_kd_left(float val);

        void set_kp_right(float val);
        void set_ki_right(float val);
        void set_kd_right(float val);
};



#endif