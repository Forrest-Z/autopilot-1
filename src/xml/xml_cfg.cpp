#include "xml_cfg.h"

//SimBoat *SimBoat::singleton_;
 XMLcfg* XMLcfg::singleton_=nullptr;

 XMLcfg::XMLcfg()
 {
     node_root = nullptr;
	 node_ldevice = nullptr;
	 singleton_ = this;
 }

 XMLcfg::~XMLcfg()
{

 	if(node_root!=nullptr)
 	{ 
 			mxmlDelete(node_root);
			node_root=nullptr;
	}
 }
//load the configure config ,return true is ok
 bool XMLcfg::load_xml_file(std::string str_file_path)
 {
     FILE *fp; /* File to read */

     if(strlen(str_file_path.c_str())==0) return false;

     if ((fp = fopen(str_file_path.c_str(), "rb")) == NULL)
     {
         return false;
     }
     else
     {
         node_root = mxmlLoadFile(NULL, fp, MXML_NO_CALLBACK);
         fclose(fp);

         if(!node_root) 
            return false;
         return true;
     }
      std::cout <<"read configuration file"<<str_file_path<<"\t successful" << std::endl;
 }

 bool XMLcfg::get_ldevice_xml_node(string str_ldevice_path)
 {
	 
	 if (!node_root) return false;

	 node_ldevice = mxmlFindPath(node_root, str_ldevice_path.c_str());
	 if (node_ldevice == nullptr) return false;

	 return true;
 }

 const char* XMLcfg::get_node_attr(mxml_node_t * top_node, const char*  attr,  const char* item_value)
 {
	 if ((item_value = mxmlElementGetAttr(top_node, attr)) == NULL)
	 	{	 		  
		 	return NULL;
	 	}
	 return item_value;
 }
 const char* XMLcfg::get_child_node_by_attr(mxml_node_t * top_node,  char*  nodename,  char*  attr_name,  char*  attr_value, char*  target_attr,const char* value)
 {

	 mxml_node_t *childnode;
	 mxml_node_t* node=NULL;
	 static int index=0;

	  childnode = mxmlFindElement(top_node, top_node,nodename, NULL, NULL, MXML_DESCEND);
	// childnode = mxmlGetFirstChild(top_node);//top_node->child;

	 while (childnode != NULL)
	 {

		 if (!strcmp(childnode->value.element.name, nodename))
		 {
			 if ((node=get_xml_node(childnode, attr_name, attr_value))!=NULL)
			 {
				 if((value = get_node_attr(node, target_attr, value))!=NULL)
				 	{
						
					std::cout<<index++<<":"<<attr_value<<"="<<value<<"\t"<<endl;
						return value;
				 	}
			 }
		 }
		 
		 childnode = childnode->next;
	 }
	  std::cout<<"childnode is null"<<endl;
	 return NULL;
 }

 mxml_node_t * XMLcfg::get_child_node_by_attr(mxml_node_t * top_node,  char*  nodename,  char*  attr_name, char*  attr_value)
 {
	 mxml_node_t *childnode;
	 mxml_node_t* node = NULL;

	 childnode = top_node->child;
	 while (childnode != NULL)
	 {
		 if (!strcmp(childnode->value.element.name, nodename))
		 {
			 if ((node = get_xml_node(childnode, attr_name, attr_value)) != NULL)
			 {
				 return node;
			 }
		 }
		 childnode = childnode->next;
	 }
	 return NULL;
 }

 mxml_node_t * XMLcfg::get_xml_node(mxml_node_t * top_node,  char*  attr,const char*  value)
 {
	 mxml_node_t *node;
	const char* nodevalue=NULL;

	 if (top_node == nullptr) return NULL;

	 node = top_node;
	 do 
	 {
		 if ((nodevalue = get_node_attr(node, attr, nodevalue) )== NULL)
			 continue;
		 if (strcmp(nodevalue, value)==0)
			{
			return node;
		 	}
		// node = node->next;
	 } while (node = node->next);

	 return NULL;
	
 }

 //"/USV_CFG/USV/LDevice[name='LonControl']/entry"
 mxml_node_t *  XMLcfg::get_node_by_path(const char*  path)
 {
	 mxml_node_t * node;
	 if (path == NULL) return NULL;

	 if ((node = mxmlFindElement(node_root, node_root, path, NULL, NULL,MXML_DESCEND)) == NULL)
		return NULL;
	
	 return node;
 }

 bool XMLcfg::init(std::string str_file_path)
 {
	 if (load_xml_file(str_file_path) == false) return false;
	// if (get_ldevice_xml_node(PATH_LDevice) == false) return false;

	 return true;
 }

XMLcfg *XMLcfg::get_singleton()
{
    if (singleton_ == nullptr)
     {
         singleton_ = new XMLcfg();
     }
    
        return singleton_;
}


namespace AP{
    XMLcfg *get_XMLcfg(){
        return XMLcfg::get_singleton();
    }
};
