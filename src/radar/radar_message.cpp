#include <stdio.h>
#include "radar_message.h"
#include <include/usv_include.h>
#include <user_time/user_time.h>
#include <math/common_math.h>
#include <util/easylogging++.h>

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
    switch (msgid)
    {
        case UnMsgID_BoatLidar_IPC_Arm_Cross:
        {
            receive_time_stamp_= user_time::get_millis();
            memcpy(&ipc2arm_msg_,pBuf,sizeof(ipc2arm_msg_));

            // filter data
            ipc2arm_msg_.relative_distance = lateral_error_filter_->Update(ipc2arm_msg_.relative_distance);
        }
        break;
        default:{}break;
    }

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
       
        arm2ipc_msg_.gps_valid = (ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A') ?(false):(true);
        arm2ipc_msg_.lat       =  static_cast<int32_t>(ins_msg.latitude * 1e7);
        arm2ipc_msg_.lng       =  static_cast<int32_t>(ins_msg.longitude * 1e7);
        arm2ipc_msg_.heading_deg = ins_msg.heading;
        arm2ipc_msg_.ground_course_deg = ins_msg.motionDirection;
        arm2ipc_msg_.ground_speed_kn   = ins_msg.speed;
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
