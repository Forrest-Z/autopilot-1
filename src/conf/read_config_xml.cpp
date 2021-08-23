#include "read_config_xml.h"


#define PRINTF_CONFIG_VALUE(index,name ,value)   
//std::cout<<index<<":"<<name<<"\t\t="<<str_value<<"-->successful"<<endl;


ReadConfigXml::ReadConfigXml()
{
	m_xml_cfg = AP::get_XMLcfg();
}

ReadConfigXml::~ReadConfigXml()
{
     delete m_xml_cfg;
}


string ReadConfigXml::get_file_path()
{

    return this->file_path_;
}

bool ReadConfigXml::open_file(const string &file_path)
{
	
	if (m_xml_cfg->init(file_path))
	{
		file_path_ = file_path;
		std::cout<<"open file "<<file_path<<"ok"<<endl;
		return true;
	}
	else
	{
		std::cout<<"open file "<<file_path<<"fail"<<endl;
		return false;
	}
}


bool ReadConfigXml::read_config_value(Conf &config_msg, string &_error)
{
    string str="";

	if ((nodeLD = m_xml_cfg->get_node_by_path(PATH_LDevice)) == NULL) 
	{
			std::cout<<"get node "<<PATH_LDevice<<"--false"<<endl;
		return false;
	}


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

    // path planning
    ret = read_config_path_planning(config_msg.planning_conf_,_error);
    if(ret == false){return false;}

    return true;
}

bool ReadConfigXml::get_value(const string &section,const string &key,string &value,string &error)
{
	const char* str_value;
	mxml_node_t * node;
	char sectionchar[200];
	char keychar[200];
	strcpy(sectionchar,section.c_str());
	strcpy(keychar,key.c_str());
	
	error = "key"+key +"not exist";
	if ((node = m_xml_cfg->get_xml_node(nodeLD, "name", sectionchar)) == NULL) return false;
	if (!(str_value=m_xml_cfg->get_child_node_by_attr(node, "entry", "name", keychar, "value", str_value))) return false;
	
   	error="";
	value = str_value;
	
	return true;

}

bool ReadConfigXml::read_config_boat(BoatConf &config_msg,string &_error)
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

    if(!get_value("BOAT_CONF","ins_type",str,_error)){
        config_msg.ins_type =1;
    }else{
        config_msg.ins_type = atoi(str.c_str());
    }

    return true;
}



bool ReadConfigXml::read_config_radar(RadarConf &config_msg,string &_error)
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



bool ReadConfigXml::read_config_control_lat(ControlConf &config_msg,string &_error)
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
    if(!get_value("LAT_CONTROLLER_ADRC","lat_steering_angle_leso_delta",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_leso_conf_.delta_ = atof(str.c_str());
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
 // [LAT_CONTROLLER_MRAC]
    if(!get_value("LAT_CONTROLLER_MRAC","model_order",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.model_order = atoi(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","wc",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.wc = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","wa",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.wa = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","gama",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.gama = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","q1",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.Q[0] = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","q2",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.Q[1]= atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","e0",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.e0= atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","lv",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.lv= atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","param_tolerance",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.param_tolerance = atof(str.c_str());
    }
    if(!get_value("LAT_CONTROLLER_MRAC","param_limit",str,_error)){
        return false;
    }else{
        config_msg.lat_steering_angle_mrac_conf_.param_limit = atof(str.c_str());
    }

    return true;
}

bool ReadConfigXml::read_config_control_lon(ControlConf &config_msg,string &_error)
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
    if(!get_value("LON_CONTROLLER_ADRC","lon_speed_leso_delta",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_leso_conf_.delta_ = atof(str.c_str());
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
    if(!get_value("LON_CONTROLLER_MRAC","model_order",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.model_order = atoi(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","wc",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.wc = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","wa",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.wa = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","gama",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.gama = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","q1",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.Q[0] = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","q2",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.Q[1]= atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","e0",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.e0= atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","lv",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.lv= atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","param_tolerance",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.param_tolerance = atof(str.c_str());
    }
    if(!get_value("LON_CONTROLLER_MRAC","param_limit",str,_error)){
        return false;
    }else{
        config_msg.lon_speed_mrac_conf_.param_limit = atof(str.c_str());
    }

    return true;
}

bool ReadConfigXml::read_config_path_planning(PlanningConf &config_msg,string &_error)
{
    string str="";

    // [LON_CONTROLLER_COMMON]
    if(!get_value("PATH_PLANNING","BR_lookahead",str,_error)){
        return false;
    }else{
        config_msg.BR_lookahead = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_bendy_ratio",str,_error)){
        return false;
    }else{
        config_msg.BR_bendy_ratio = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_bendy_angle",str,_error)){
        return false;
    }else{
        config_msg.BR_bendy_angle = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_margin_max",str,_error)){
        return false;
    }else{
        config_msg.BR_margin_max = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_max_deviate_angle",str,_error)){
        return false;
    }else{
        config_msg.BR_max_deviate_angle = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_min_near_distance",str,_error)){
        return false;
    }else{
        config_msg.BR_min_near_distance = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_shoreline_safe_pb",str,_error)){
        return false;
    }else{
        config_msg.BR_shoreline_safe_pb = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_shoreline_safe_distance",str,_error)){
        return false;
    }else{
        config_msg.BR_shoreline_safe_distance = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","BR_shoreline_safe_angle",str,_error)){
        return false;
    }else{
        config_msg.BR_shoreline_safe_angle = atof(str.c_str());
    }


    if(!get_value("PATH_PLANNING","DB_queue_size_param",str,_error)){
        return false;
    }else{
        config_msg.DB_queue_size_param = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","DB_database_size_param",str,_error)){
        return false;
    }else{
        config_msg.DB_database_size_param = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","DB_database_expiry_seconds",str,_error)){
        return false;
    }else{
        config_msg.DB_database_expiry_seconds = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","DB_beam_width",str,_error)){
        return false;
    }else{
        config_msg.DB_beam_width = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","DB_radius_min",str,_error)){
        return false;
    }else{
        config_msg.DB_radius_min = atof(str.c_str());
    }
    if(!get_value("PATH_PLANNING","DB_dist_max",str,_error)){
        return false;
    }else{
        config_msg.DB_dist_max = atof(str.c_str());
    }

    if(!get_value("PATH_PLANNING","PP_type",str,_error)){
        return false;
    }else{
        config_msg.PP_type = atof(str.c_str());
    }
    
    if(!get_value("PATH_PLANNING","lon_scan_distance",str,_error)){
        return false;
    }else{
        config_msg.lon_scan_distance = atof(str.c_str());
    }

    if(!get_value("PATH_PLANNING","lon_scan_angle",str,_error)){
        return false;
    }else{
        config_msg.lon_scan_angle = atof(str.c_str());
    }

    if(!get_value("PATH_PLANNING","lon_time_constance",str,_error)){
        return false;
    }else{
        config_msg.lon_time_constance = atof(str.c_str());
    }

    if(!get_value("PATH_PLANNING","lon_dccel_speed",str,_error)){
        return false;
    }else{
        config_msg.lon_dccel_speed = atof(str.c_str());
    }
    return true;
}
