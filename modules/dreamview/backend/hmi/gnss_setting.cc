#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "gnss_setting.h"
#include "m2_setting.h"
#include "zhd_setting.h"

int fd;
std::string config_file_path;
apollo::drivers::gnss::config::Stream::Format gnss_format;

void ippoint_to_ipcomma(const char *ippoint, char *ipcomma)
{
  const char *temp = ippoint;
  for(unsigned int i = 0;i<strlen(ippoint);i++)
  {
    if(*(temp + i) == '.')
      *(ipcomma + i) = ',';
    else
      *(ipcomma + i) = *(temp + i);
  }
}

int gnss_setting_conf_clean(apollo::drivers::gnss::config::Config& config)
{
  m2_setting_conf_clean(config);

  return 0;
}

int gnss_conf_init()
{
  std::string planning_conf_path = "/apollo/modules/drivers/gnss/conf/gnss.conf";
  apollo::drivers::gnss::config::Config config;
  char stringline[256] = "";
  char gnss_conf_pb_txt[256] = "";

  // if (FLAGS_use_navigation_mode)
  //   planning_conf_path = "/apollo/modules/planning/conf/planning_navi.conf";
  if (apollo::common::util::PathExists(planning_conf_path)) {
    std::ifstream planning_conf_fs;
    planning_conf_fs.open(planning_conf_path);
    while(planning_conf_fs.getline(stringline,256).good())
    {
      if(strstr((const char *)stringline, (const char *)"--sensor_conf_file") == stringline)
        sscanf(stringline, "--sensor_conf_file=%s", gnss_conf_pb_txt);
    }
    config_file_path = gnss_conf_pb_txt;
    AERROR << "bbbbbbbbbbbbbbbbbb ---" << config_file_path;
    planning_conf_fs.close();
  }

  apollo::common::util::GetProtoFromFile(config_file_path.c_str(), &config);
  AERROR << "ccccccccccccccccc config.data().format()---" << config.data().format();
  gnss_format = config.data().format();
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    AERROR << "cccccccc    apollo::drivers::gnss::config::Config::Stream::Format::NEWTONM2_BINARY";
    m2_conf_init(config_file_path);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_conf_init(config_file_path);
  }

  return 0;
}

int gnss_conf_logincommand_init()
{
  gnss_conf_init();  
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_conf_logincommand_init(config_file_path);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_conf_logincommand_init(config_file_path);
  }

  return 1;
}

int gnss_basestation_setting(std::string rtk_base_addr,std::string rtk_base_port)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_basestation_setting(rtk_base_addr,rtk_base_port);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_basestation_setting(rtk_base_addr,rtk_base_port);
  }

  return 1;
}

int gnss_mountpoint_setting(std::string rtk_base_mountpoint)
{  
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_mountpoint_setting(rtk_base_mountpoint);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_mountpoint_setting(rtk_base_mountpoint);
  }

  return 1;
}

int gnss_user_setting(std::string rtk_base_username,std::string rtk_base_passwd)
{ 
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_user_setting(rtk_base_username,rtk_base_passwd);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_user_setting(rtk_base_username,rtk_base_passwd);
  }

  return 1;
}

int gnss_localaddr_setting(std::string local_addr)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_localaddr_setting(local_addr);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_localaddr_setting(local_addr);
  }

  return 1;
}

int gnss_localnetmask_setting(std::string local_netmask)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_localnetmask_setting(local_netmask);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_localnetmask_setting(local_netmask);
  }

  return 1;
}

int gnss_localgateway_setting(std::string local_gateway)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_localgateway_setting(local_gateway);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_localgateway_setting(local_gateway);
  }

  return 1;
}

int gnss_headoffset_setting(std::string head_offset)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_headoffset_setting(head_offset);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_headoffset_setting(head_offset);
  }

  return 1;
}

int gnss_leverarm1_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_leverarm1_setting(leverarm_x,leverarm_y,leverarm_z);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_leverarm1_setting(leverarm_x,leverarm_y,leverarm_z);
  }

  return 1;
}

int gnss_leverarm2_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_leverarm2_setting(leverarm_x,leverarm_y,leverarm_z);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_leverarm2_setting(leverarm_x,leverarm_y,leverarm_z);
  }

  return 1;
}

int gnss_config_get()
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_config_get();
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_config_get();
  }

  return 1;
}


int gnss_localip_get(std::string& localip)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_localip_get(localip);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_localip_get(localip);
  }
  
  return 0;
}

int gnss_localmask_get(std::string& localmask)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_localmask_get(localmask);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_localmask_get(localmask);
  }
  
  return 0;
}

int gnss_localgateway_get(std::string& localgateway)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_localgateway_get(localgateway);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_localgateway_get(localgateway);
  }
  
  return 0;
}

int gnss_headoffset_get(std::string& headoffset)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_headoffset_get(headoffset);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_headoffset_get(headoffset);
  }
  
  return 1;
}

int gnss_leverarm1_get(std::string& leverarm_x, std::string& leverarm_y, std::string& leverarm_z)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_leverarm1_get(leverarm_x,leverarm_y,leverarm_z);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_leverarm1_get(leverarm_x,leverarm_y,leverarm_z);
  }
  
  return 0;
}

int gnss_leverarm2_get(std::string& leverarm_x, std::string& leverarm_y, std::string& leverarm_z)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_leverarm2_get(leverarm_x,leverarm_y,leverarm_z);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_leverarm2_get(leverarm_x,leverarm_y,leverarm_z);
  }
  
  return 0;
}

int gnss_basestation_get(std::string& rtk_base_addr,std::string& rtk_base_port)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_basestation_get(rtk_base_addr,rtk_base_port);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_basestation_get(rtk_base_addr,rtk_base_port);
  }

  return 0;
}

int gnss_user_get(std::string& rtk_base_username,std::string& rtk_base_passwd)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_user_get(rtk_base_username,rtk_base_passwd);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_user_get(rtk_base_username,rtk_base_passwd);
  }
  
  return 0;
}

int gnss_mountpoint_get(std::string& rtk_base_mountpoint)
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_mountpoint_get(rtk_base_mountpoint);
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_mountpoint_get(rtk_base_mountpoint);
  }
  
  return 0;
}

int gnss_setting_save_and_close()
{
  if(gnss_format == apollo::drivers::gnss::config::Stream::NEWTONM2_BINARY)
  {
    m2_setting_save_and_close();
  }
  else if(gnss_format == apollo::drivers::gnss::config::Stream::ZHD_BINARY)
  {
    zhd_setting_save_and_close();
  }

  return 1;
}   

