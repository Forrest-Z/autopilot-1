#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <fstream>
#include <vector>
using namespace std;
#include "conf.h"
#include <xml/xml_cfg.h>

#define   CFG_FILE_PATH   "./usv_cfg.xml"
#define   PATH_LDevice    "LDevice"

class ReadConfigXml
{
public:
	ReadConfigXml();
	~ReadConfigXml();
    
	string get_file_path();
	bool open_file(const string &file_path);
    bool read_config_value(Conf&config_msg,string &_error);
private:

	bool get_value(const string &section,const string &key,string &value,string &error);
	bool read_config_boat(BoatConf &config_msg,string &_error);
	bool read_config_radar(RadarConf &config_msg,string &_error);
	bool read_config_control_lat(ControlConf &config_msg,string &_error);
	bool read_config_control_lon(ControlConf &config_msg,string &_error);


private:
    string file_path_;
	XMLcfg* m_xml_cfg;
	int cfg_index;
	mxml_node_t * nodeLD;
};
