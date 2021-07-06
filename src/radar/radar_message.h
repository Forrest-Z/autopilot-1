#pragma once
#include <hal/udp_device.h>
#include <string>
#include <memory>
#include "proc_apdu.h"
#include "radar_conf.h"
#include <filter/filter.h>

class RadarMessage{
public:
#pragma pack(1)
    struct IPC2ARM{
        int32_t lat;
        int32_t lng;
        IPC2ARM():lat(0),lng(0){}
        IPC2ARM(int32_t lat0,int32_t lng0):lat(lat0),lng(lng0){}
    };

    struct ARM2IPC{
        uint8_t gps_valid;
        int32_t lat;
        int32_t lng;
        float   heading_deg;
        float   ground_course_deg;
        float   ground_speed_kn;
    };
#pragma pack()

    RadarMessage();
    virtual ~RadarMessage();

   /* Do not allow copies */
    RadarMessage(const RadarMessage &other) = delete;
    RadarMessage &operator=(const RadarMessage&) = delete;

    void init(const RadarConf &conf);

  
    static RadarMessage *get_singleton(){
        return singleton_;
    }

    void update_receive(void);

    void update_send(void);
  

    IPC2ARM& ipc2arm_msg() {return ipc2arm_msg_;}
    ARM2IPC& arm2ipc_msg() {return arm2ipc_msg_;}
    uint64_t radar_message_time_stamp() const {return receive_time_stamp_;}

private:

    void connect(){
        udp_ptr_->socket_bind(receive_port_);
    }

    void handle_message(int sysid, int msgid, uint8 * const pBuf);

    void send_message(int sysid, int componetid, int msgid);

    // push an object into the database. Pos is  lat and lng
    void database_push(const int32_t lat,const int32_t lng);

private:
    static RadarMessage* singleton_;

    std::unique_ptr<UDPDevice> udp_ptr_;
    std::unique_ptr<ProcAPDU> m_pProcAPDU;
    std::unique_ptr<MeanFilter> lateral_error_filter_;

    //std::vector<IPC2ARM> obstacle_list_;


private:
    IPC2ARM ipc2arm_msg_;
    ARM2IPC arm2ipc_msg_;
    uint16_t seq = 0;
    uint64_t receive_time_stamp_ = 0;
   
private:
    std::string server_ip_;
    std::string local_ip_;
    uint16_t receive_port_;
    uint16_t send_port_;
    std::uint_fast8_t window_size_;

private:
    char recv_buff_[512];

};


namespace AP {
    RadarMessage *radar_message();
};
