#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <fstream>
using namespace std;
#include <stdint.h>

#include <common/commom.h>

#include <config.h>
#include <mxml-private.h>
#include <fcntl.h>
#ifndef O_BINARY
#  define O_BINARY 0
#endif /* !O_BINARY */


class XMLcfg{
public:

    XMLcfg();
     ~XMLcfg();

    /* Do not allow copies */
    XMLcfg(const XMLcfg &other) = delete;
    XMLcfg &operator=(const XMLcfg&) = delete;

private:
    bool load_xml_file(string str_file_path) WARN_IF_UNUSED ;
    bool get_ldevice_xml_node(string str_ldevice_path);
	const char* get_node_attr(mxml_node_t * top_node, const char*  attr,const char* item_value);
	bool get_node_value();


public:
	const char* get_child_node_by_attr(mxml_node_t * top_node,  char*  nodename,  char*  attr_name, char*  attr_value, char*  target_attr,const  char*  value);
	mxml_node_t * get_xml_node(mxml_node_t * top_node,   char*  attr,  const char* value);
	mxml_node_t * get_child_node_by_attr(mxml_node_t * top_node,  char*  nodename,  char*  attr_name, char*  attr_value);
	mxml_node_t * get_node_by_path(const char*  path);

	bool init(std::string str_file_path);

    static XMLcfg *get_singleton();

public:
    static XMLcfg* singleton_;
    mxml_node_t * node_root;
    mxml_node_t * node_ldevice;


};

namespace AP{
    XMLcfg *get_XMLcfg();
};