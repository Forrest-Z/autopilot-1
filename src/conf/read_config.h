#ifndef READ_CONFIG_H_
#define READ_CONFIG_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <fstream>
#include <vector>
using namespace std;
#include "conf.h"

class ReadConfig
{
public:
    ReadConfig();
    ~ReadConfig();
    
    void set_file_path(const string &file_path);
    string get_file_path();
    bool get_value(const string &section,const string &key,string &value,string &error);
    bool modify_value(const string &section,const string &key,const string &value,string &error);
    bool read_config_value(Conf&config_msg,string &_error);

private:
    bool open_file();
    bool find_section(const string &section_name);
    bool find_key(const string key);
    bool open_file_write();
    bool open_file_read();
    bool set_value(const string &key,const string &value);
    void write_file(vector<string> &v_list);

    string file_path_;
    fstream fout_;
    fstream fin_;
    string content_;
    string value_;
    string error_;

private:
    bool read_config_boat(BoatConf &config_msg,string &_error);
    bool read_config_radar(RadarConf &config_msg,string &_error);
    bool read_config_control_lat(ControlConf &config_msg,string &_error);
    bool read_config_control_lon(ControlConf &config_msg,string &_error);


};
#endif //READ_CONFIG_H_