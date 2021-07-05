#include "read_config.h"

ReadConfig::ReadConfig()
{

}

ReadConfig::~ReadConfig()
{

}

void ReadConfig::set_file_path(const string &file_path)
{
    file_path_ = file_path;
}

string ReadConfig::get_file_path()
{
    return this->file_path_;
}

bool ReadConfig::open_file()
{
    if(true == fin_.is_open()){
        fin_.close();
    }

    fin_.open(file_path_.c_str());

    if(!fin_.is_open()){
        error_= "the file open failed:" + file_path_;
        return false;
    }
    return true;
}

bool ReadConfig::find_section(const string &section_name)
{
    if(-1 != content_.find('[')){

        string s_temp = content_.substr(content_.find('[') + 1, content_.find(']') - content_.find('[') - 1);

        if(0 == strcmp(s_temp.c_str(), section_name.c_str())){
            return true;
        }
        else{
            return false;
        }

    }else{

        return false;

    }
}

bool ReadConfig::find_key(const string key)
{
    size_t del_comment_place = content_.find((char)'//',0);
    size_t find_equal = content_.find((char)'=',0);
    
    if((-1 != del_comment_place && del_comment_place < find_equal) ||
       (-1 != del_comment_place && -1 == find_equal) || -1 == find_equal){

           return false;

    }
    string this_key = content_.substr(0,content_.find('='));

    if(0 == strcmp(this_key.c_str(),key.c_str())){

        value_ = content_.substr(content_.find('=')+1,content_.length()-content_.find('=') - 1);
        return true;

    }

    return false;
}

bool ReadConfig::get_value(const string &section,const string &key,string &value,string &error)
{
    error_ = "";
    if(false == open_file()){
        error = error_;
        return false;
    }
    char str[4096] = {0};
    bool b_find_section = false;
    while (fin_.getline(str,sizeof(str))){
        content_ = str;
        if(!b_find_section){

            if(find_section(section)){
                b_find_section = true;
            }else{
                error_ = "";
                error_ = "section "+section+" not exist";
            }

        }else{
            if(find_key(key)){
                fin_.close();
                error_ = "";
                value = value_;
                //std::cout <<endl;
                std::cout << "read:" << section <<" : " << key <<" = "<< value << "successful" <<endl;
                return true;
            }else{
                error_="";
                error_="key " + key +" not exist";
            }
        }
        memset(str,0,4096);
    }
    error = error_;
    fin_.close();
    std::cout <<endl;
    std::cout << error << endl;
    return false;
}


bool ReadConfig::open_file_write()
{
    fout_.close();
    fout_.clear();

    fout_.open(file_path_.c_str(),ios::out | ios::trunc);
    if(!fout_.is_open()){
        error_ = "cann't open file" + file_path_;
        return false;
    }

    return true;
}

bool ReadConfig::open_file_read()
{
    fout_.close();
    fout_.clear();
    fout_.open(file_path_.c_str(),ios::in);
    if(!fout_.is_open()){
        error_ = "open file fail:" + file_path_;
        return false;
    }
    return true;
}

bool ReadConfig::set_value(const string &key,const string &value)
{
    size_t del_comment_place = content_.find((char)'//');
	size_t find_equal = content_.find((char)'=');
	//filter //  = 
	if ((-1 != del_comment_place && del_comment_place < find_equal) ||
     (-1 != del_comment_place && -1 == find_equal) || -1 == find_equal){
         
		return false;

	}
	string sKey = content_.substr(0, content_.find('='));

	if (0 == strcmp(sKey.c_str(), key.c_str())){

		content_ = content_.substr(0, content_.find('=') + 1) + value;
		return true;

	}

	return false;
}

bool ReadConfig::modify_value(const string &section,const string &key,const string &value,string &error)
{
    error_ = "";
	if (false == open_file_read())
	{
		error = error_;
		return false;
	}

	char str[4096] = { 0 };
	vector<string> content_list;
	bool isModify = false;
	bool is_find_section = false;
	while ( (fout_.getline(str, sizeof(str))))
	{
		content_ = str;
		if (!is_find_section)
		{
			if (find_section(section))
			{
				is_find_section = true;
			}
			else
			{
				error_ = "";
				error_ = "section " + section + " not exist";
			}
		}
		else
		{
			if (!isModify)
			{
				if (set_value(key, value))
				{
					isModify = true;
				}
				else
				{
					error_ = "";
					error_ = "键名" + key + " not exist";
				}
			}
		}
		content_list.push_back(content_);
		content_ = "";
		memset(str, 0, 4096);
	}
	error = error_;
	write_file(content_list);
	fout_.flush();
	fout_.close();
	return isModify;
}

void ReadConfig::write_file(vector<string> &content_list)
{
    if (false == open_file_write())
	{
		fout_.close();
		return;
	}
	for (size_t iIndex = 0; iIndex < content_list.size(); iIndex++)
	{
		fout_ << content_list[iIndex] << endl;
	}
	fout_.close();
	vector<string>().swap(content_list);
}

bool ReadConfig::read_config_value(Conf &config_msg,string &_error)
{
    string str="";

    bool ret = true;
    ret = read_config_boat(config_msg.boat_conf_,_error);
    if(ret == false){return false;}


    // [VEHICLE_STATE]
    if(!get_value("VEHICLE_STATE","use_ground_speed_angle",str,_error)){
        return false;
    }else{
        config_msg.vehicle_state_conf_.use_ground_speed_angle_ = (str == "false")?(false):(true);
    }
    if(!get_value("VEHICLE_STATE","use_ground_speed",str,_error)){
        return false;
    }else{
        config_msg.vehicle_state_conf_.use_ground_speed_ =  (str == "false")?(false):(true);
    }
    if(!get_value("VEHICLE_STATE","use_external_angular_velocity",str,_error)){
        return false;
    }else{
        config_msg.vehicle_state_conf_.use_external_angular_velocity_ =  (str == "false")?(false):(true);
    }
    if(!get_value("VEHICLE_STATE","heading_error_td_r",str,_error)){
        return false;
    }else{
        config_msg.vehicle_state_conf_.heading_error_td_filter_conf_.r_ = atof(str.c_str());
    }
    if(!get_value("VEHICLE_STATE","heading_error_td_h",str,_error)){
        return false;
    }else{
        config_msg.vehicle_state_conf_.heading_error_td_filter_conf_.h_  = atof(str.c_str());
    }
    if(!get_value("VEHICLE_STATE","heading_error_td_angle",str,_error)){
        return false;
    }else{
        config_msg.vehicle_state_conf_.heading_error_td_filter_conf_.angle_ = (str== "false")?(false):(true);
    }


    ret = read_config_control_lat(config_msg.control_conf_,_error);
    if(ret == false){return false;}

    ret = read_config_control_lon(config_msg.control_conf_,_error);
    if(ret == false){return false;}

    ret = read_config_radar(config_msg.radar_conf_,_error);
    if(ret == false){return false;}
    
  // debug
    if(!get_value("DEBUG","navigation_info_source",str,_error)){
        return false;
    }else{
        config_msg.navigation_info_source = atoi(str.c_str());
    }
    if(!get_value("DEBUG","simualtion_enable",str,_error)){
        return false;
    }else{
        config_msg.simualtion_enable = (str== "false")?(false):(true);
    }
    if(!get_value("DEBUG","old_code_test_enable",str,_error)){
        return false;
    }else{
        config_msg.old_code_test_enable = (str== "false")?(false):(true);
    }
  

    return true;
}

bool ReadConfig::read_config_boat(BoatConf &config_msg,string &_error)
{
     string str="";

    if(!get_value("BOAT_CONF","route_switch_steering_cmd",str,_error)){
        return false;
    }else{
        config_msg.route_switch_steering_cmd_ = atof(str.c_str());
    }
    if(!get_value("BOAT_CONF","position_kp",str,_error)){
        return false;
    }else{
        config_msg.position_kp_ = atof(str.c_str());
    }
    if(!get_value("BOAT_CONF","position_second_limit",str,_error)){
        return false;
    }else{
        config_msg.position_second_limit_ = atof(str.c_str());
    }


    // console 12
    if(!get_value("BOAT_CONF","master_ip",str,_error)){
        return false;
    }else{
        config_msg.local_ip_ = str;
    }
    if(!get_value("BOAT_CONF","console_ip",str,_error)){
        return false;
    }else{
        config_msg.console_ip_ = str;
    }
    if(!get_value("BOAT_CONF","master_port",str,_error)){
        return false;
    }else{
        config_msg.local_port_ = atoi(str.c_str());
    }
    if(!get_value("BOAT_CONF","console_port",str,_error)){
        return false;
    }else{
        config_msg.console_port_ = atoi(str.c_str());
    }


    return true;
}



bool ReadConfig::read_config_radar(RadarConf &config_msg,string &_error)
{
     string str="";

    if(!get_value("RADAR_COMMUNICATION","arm_ip",str,_error)){
        return false;
    }else{
        config_msg.arm_ip_ = str;
    }
    if(!get_value("RADAR_COMMUNICATION","arm_port",str,_error)){
        return false;
    }else{
        config_msg.arm_port_ =atoi(str.c_str());
    }
    if(!get_value("RADAR_COMMUNICATION","app_ip",str,_error)){
        return false;
    }else{
        config_msg.app_ip_ = str;
    }
    if(!get_value("RADAR_COMMUNICATION","app_port",str,_error)){
        return false;
    }else{
        config_msg.app_port_ =atoi(str.c_str());
    }
    if(!get_value("RADAR_COMMUNICATION","filter_window_size",str,_error)){
        return false;
    }else{
        config_msg.window_size_ =atoi(str.c_str());
    }
    return true;
}



bool ReadConfig::read_config_control_lat(ControlConf &config_msg,string &_error)
{
    string str="";

    // [CONTROL_TYPE]
    if(!get_value("CONTROL_TYPE","lat_controller_type",str,_error)){
        return false;
    }else{
        config_msg.lat_controller_type_ = str;
    }
    if(!get_value("CONTROL_TYPE","lon_controller_type",str,_error)){
        return false;
    }else{
        config_msg.lon_controller_type_ = str;
    }

    // [LAT_CONTROLLER_COMMON]
    if(!get_value("LAT_CONTROLLER_COMMON","lat_controller_error_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_controller_error_limit_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_COMMON","lat_controller_input_rate_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_controller_input_rate_limit_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_COMMON","lat_controller_output_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_controller_output_limit_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_COMMON","lat_controller_output_rate_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_controller_output_rate_limit_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_COMMON","lat_sight_track_distance",str,_error)){
        return false;
    }else{
        config_msg.lat_sight_track_distance_ = atof(str.c_str());
    }
 

    // [LAT_CONTROLLER_PID]
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_angle_pid_kp",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_pid_conf_.kp_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_angle_pid_ki",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_pid_conf_.ki_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_angle_pid_second_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_pid_conf_.second_limit_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_angle_pid_output_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_pid_conf_.output_saturation_level_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_angle_pid_integrator_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_pid_conf_.integrator_saturation_level_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_rate_pid_kp",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_rate_pid_conf_.kp_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_rate_pid_ki",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_rate_pid_conf_.ki_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_rate_pid_second_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_rate_pid_conf_.second_limit_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_rate_pid_output_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_rate_pid_conf_.output_saturation_level_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_PID","lat_steering_rate_pid_integrator_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_rate_pid_conf_.integrator_saturation_level_ = atof(str.c_str());
    }

   // [LAT_CONTROLLER_ADRC]
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_leso_model_order",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_leso_conf_.model_order_ = atoi(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_leso_wo",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_leso_conf_.wo_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_leso_b0",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_leso_conf_.b0_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_leso_angle",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_leso_conf_.angle_ = (str=="false")?(false):(true);
    }
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_npd_r",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_npd_conf_.r_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_npd_h",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_npd_conf_.h_ = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_npd_c",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_npd_conf_.c_ = atof(str.c_str());
    }
    return true;
}

bool ReadConfig::read_config_control_lon(ControlConf &config_msg,string &_error)
{
      string str="";

    // [LON_CONTROLLER_COMMON]
    if(!get_value("LON_CONTROLLER_COMMON","lon_controller_error_limit",str,_error)){
        return false;
    }else{
        config_msg.lon_controller_error_limit_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_COMMON","lon_controller_input_rate_limit",str,_error)){
        return false;
    }else{
        config_msg.lon_controller_input_rate_limit_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_COMMON","lon_controller_output_limit",str,_error)){
        return false;
    }else{
        config_msg.lon_controller_output_limit_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_COMMON","lon_controller_output_rate_limit",str,_error)){
        return false;
    }else{
        config_msg.lon_controller_output_rate_limit_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_COMMON","lon_radar_hold_thtottle_cmd",str,_error)){
        return false;
    }else{
        config_msg.lon_radar_hold_thtottle_cmd_ = atof(str.c_str());
    }

    // [LON_CONTROLLER_PID]
    if(!get_value("LON_CONTROLLER_PID","lon_speed_pid_conf_kp",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_pid_conf_.kp_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_PID","lon_speed_pid_conf_ki",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_pid_conf_.ki_  = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_PID","lon_speed_pid_output_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_pid_conf_.output_saturation_level_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_PID","lon_speed_pid_max_acceleration",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_pid_conf_.max_acceleration_= atof(str.c_str());
    }
   
   // [LON_CONTROLLER_ADRC]
   /*
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_target_td_r",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_target_td_conf_.r_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_target_td_h",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_target_td_conf_.h_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_target_td_angle",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_target_td_conf_.angle_ = (str == "false")?(false):(true);
    }
    */
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_leso_model_order",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_leso_conf_.model_order_ = atoi(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_leso_wo",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_leso_conf_.wo_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_leso_b0",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_leso_conf_.b0_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_leso_angle",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_leso_conf_.angle_ = (str=="false")?(false):(true);
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_sqrt_kp",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_sqrt_conf_.kp_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_sqrt_ki",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_sqrt_conf_.ki_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_sqrt_second_limit",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_sqrt_conf_.second_limit_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_sqrt_output_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_sqrt_conf_.output_saturation_level_ = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_sqrt_integrator_saturation_level",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_sqrt_conf_.integrator_saturation_level_ = atof(str.c_str());
    }
    return true;
}