#include <stdio.h>
#include "radar_message.h"
#include <include/usv_include.h>
#include <user_time/user_time.h>
#include <math/common_math.h>
#include <util/easylogging++.h>
#include <math/Location.h>
#include <planning/database.h>

using namespace LOC;

RadarMessage *RadarMessage::singleton_ = nullptr;

namespace AP{
    RadarMessage *radar_message(){
        return RadarMessage::get_singleton();
    }
};


RadarMessage::RadarMessage()
{
    singleton_ = this;
    
    udp_ptr_    = std::unique_ptr<UDPDevice>(new UDPDevice());
    m_pProcAPDU = std::unique_ptr<ProcAPDU>(new ProcAPDU());

    if(udp_ptr_ == nullptr || m_pProcAPDU == nullptr){
        std::cout<<"udp_ptr = nullptr || m_pProcAPDU == nullptr"<<std::endl;
    }
}


RadarMessage::~RadarMessage(){}


 void RadarMessage::init(const RadarConf &conf)
 {
    server_ip_    = conf.app_ip_;
    local_ip_     = conf.arm_ip_;
    receive_port_ = conf.arm_port_; // 8838
    send_port_    = conf.app_port_;
    window_size_  = conf.window_size_;

    // Constructor filter
    lateral_error_filter_ = std::unique_ptr<MeanFilter>(new MeanFilter(window_size_));

    seq = 0;
    connect();
 }

void RadarMessage::update_receive(void)
{
    int ilen = 0;

    ilen = udp_ptr_->socket_recv(recv_buff_, sizeof(recv_buff_), const_cast<char *>(server_ip_.c_str()));
    if(ilen >  0){
        int nDelLen = 0;
        if (m_pProcAPDU->CheckAPDU((uint8 *)recv_buff_, ilen, nDelLen))
        {
            Data_APDU * datapadu = m_pProcAPDU->GetAPDU();
            int tmpmsgid = 0;
            memcpy(&tmpmsgid, datapadu->msgid, 3);
            handle_message(datapadu->sysid, tmpmsgid, datapadu->databuf);
        }
    }

}

 void RadarMessage::update_send(void)
{
    send_message(SysID_Boat,ID_Lidar_IPC_Up,UnMsgID_BoatLidar_Arm_IPC_Cross);
}


void RadarMessage::handle_message(int sysid, int msgid, uint8 * const pBuf)
{
    if(ins_msg.latitude == 0 || ins_msg.longitude == 0){
        return;
    }

    switch (msgid)
    {
        #if IPC2ARM_RELATIVE == 1
        case UnMsgID_BoatLidar_IPC_Arm_Cross:
        {
            receive_time_stamp_= user_time::get_millis();
            uint32_t obstalce_size = pBuf[0];
            uint16_t i = 0;
            IPC2ARM msg;
           
            while( i <8*obstalce_size){
                memcpy(&msg,&pBuf[i+1],sizeof(msg));

                #if 0
                float distance =  msg.lat;
				float angle = math::wrap_360(msg.lng + ins_msg.heading);
                database_push(distance,angle);
                #else
                float distance_m    = msg.lat;
                float angle_deg     = msg.lng;
                const AP_Proximity_Boundary_3D::Face face = boundary.get_face(angle_deg);

                if (face != _last_face) {
                    // distance is for a new face, the previous one can be updated now
                    if (_last_distance_valid) {
                        boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m);
                    } else {
                        // reset distance from last face
                        boundary.reset_face(face);
                    }

                    // initialize the new face
                    _last_face = face;
                    _last_distance_valid = false;
                }

                if (distance_m > 0.20f) {
                    // update shortest distance
                    if (!_last_distance_valid || (distance_m < _last_distance_m)) {
                        _last_distance_m = distance_m;
                        _last_distance_valid = true;
                        _last_angle_deg = angle_deg;
                    }
                    // update OA database
                    database_push( _last_distance_m, math::wrap_360(_last_angle_deg + ins_msg.heading););
                }
                #endif

                i+=8;
            }
           
        }
        break;
        #else
        case UnMsgID_BoatLidar_IPC_Arm_Cross:
        {
            receive_time_stamp_= user_time::get_millis();
            uint32_t obstalce_size = pBuf[0];
            uint16_t i = 0;
            IPC2ARM msg;
         
            while( i <8*obstalce_size){
                memcpy(&msg,&pBuf[i+1],sizeof(msg));

                int32_t lat = msg.lat;
                int32_t lng = msg.lng;

                database_push(lat,lng);
                i+=8;
            }
        }
        break;
        #endif
        default:{}break;
    }

}

extern LOC::Location ekf_origin_;
// push an object into the database. Pos is  lat and lng
void RadarMessage::database_push(const int32_t lat,const int32_t lng)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if(oaDb == nullptr || !oaDb->healthy()){
        printf("OADatabase is unhealthy!\n");
        return;
    }

    // get current position
    if(ekf_origin_.lng == 0 || ekf_origin_.lat == 0){
        printf("OADatabase : EKF original not set!\n");
        return;
    }

    Vector2f current_pos;
    Location current_loc;
    current_loc.lat = ins_msg.latitude*1e7;
    current_loc.lng = ins_msg.longitude*1e7;
    current_loc.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
     if(!current_loc.get_vector_xy_from_origin_NE(current_pos,ekf_origin_)){
        return;
    }

    Vector2f obstacle_pos;
    Location obstacle_loc;
    obstacle_loc.lat = lat;
    obstacle_loc.lng = lng;
    obstacle_loc.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    if(!obstacle_loc.get_vector_xy_from_origin_NE(obstacle_pos,ekf_origin_)){
        return;
    }

    float distance = (obstacle_pos - current_pos).length()*0.01f;
    Vector3f pos{obstacle_pos.x*0.01f,obstacle_pos.y*0.01f,0.0f};

    oaDb->queue_push(pos,user_time::get_millis(),distance);
}

void RadarMessage::database_push(const float distance,const float angle)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if(oaDb == nullptr || !oaDb->healthy()){
        printf("OADatabase is unhealthy!\n");
        return;
    }

    // get current position
    if(ekf_origin_.lng == 0 || ekf_origin_.lat == 0){
        printf("OADatabase : EKF original not set!\n");
        return;
    }


    Vector2f current_pos;
    Location current_loc;
    current_loc.lat = ins_msg.latitude*1e7;
    current_loc.lng = ins_msg.longitude*1e7;
    current_loc.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
     if(!current_loc.get_vector_xy_from_origin_NE(current_pos,ekf_origin_)){
        return;
    }

    Vector2f obstacle_pos;
    Location obstacle_loc = current_loc;
    obstacle_loc.offset_bearing(angle,distance);
    if(!obstacle_loc.get_vector_xy_from_origin_NE(obstacle_pos,ekf_origin_)){
        return;
    }

    Vector3f pos{obstacle_pos.x*0.01f,obstacle_pos.y*0.01f,0.0f};
    oaDb->queue_push(pos,user_time::get_millis(),distance);
}



void RadarMessage::send_message(int sysid, int componetid, int msgid)
{
    Data_APDU datapadu;
    datapadu.sysid = sysid;
    datapadu.compid = componetid;
    //处理消息类型、发送计数
        seq++;
    memcpy(datapadu.msgid, &msgid, 3);
    datapadu.seq = seq;
    switch (msgid)
    {
    case UnMsgID_BoatLidar_Arm_IPC_Cross:{ 
        arm2ipc_msg_.gps_valid         = (ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A') ?(false):(true);
        arm2ipc_msg_.lat               =  static_cast<int32_t>(ins_msg.latitude * 1e7);
        arm2ipc_msg_.lng               =  static_cast<int32_t>(ins_msg.longitude * 1e7);
        arm2ipc_msg_.heading_deg       = ins_msg.heading;
        arm2ipc_msg_.ground_course_deg = ins_msg.motionDirection;
        arm2ipc_msg_.ground_speed_ms   = kn2ms(ins_msg.speed);
        arm2ipc_msg_.omega             = ins_msg.rotRate;
        memcpy(datapadu.databuf,&arm2ipc_msg_,sizeof(arm2ipc_msg_));
        datapadu.len = sizeof(arm2ipc_msg_);
    }
        break;
    }

    //计算crc
    uint8 crcBuf[STD_BUFFER_LEN];
    memset(crcBuf, 0, STD_BUFFER_LEN);
    memcpy(crcBuf, &datapadu, datapadu.len + 9);
    uint16 crc16;
    crc16 = calcCRC16(0xffff, crcBuf, datapadu.len + 9);
    datapadu.checksum = crc16;
    //赋值处理类
    m_pProcAPDU->SetAPDU(datapadu);

    //发送数据
    uint8 pAPDUBuffer[STD_BUFFER_LEN];
    memset(pAPDUBuffer, 0, STD_BUFFER_LEN);
    int buflen = 0;
    if (m_pProcAPDU->GetAPDUBuff(pAPDUBuffer, buflen))
    {
        char sendDataBuf[STD_BUFFER_LEN];
        memset(sendDataBuf, 0, STD_BUFFER_LEN);
        memcpy(sendDataBuf, pAPDUBuffer, buflen);

       udp_ptr_->socket_send(server_ip_.c_str(),sendDataBuf,buflen,send_port_);
    }
}
