#ifndef MODULES_DREAMVIEW_BACKEND_HMI_M2SETTING_H_
#define MODULES_DREAMVIEW_BACKEND_HMI_M2SETTING_H_
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/common/util/file.h"

std::string m2_config_tmp_buf;
std::string m2_conf_path("111");

static void m2_ippoint_to_ipcomma(const char *ippoint, char *ipcomma)
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

int m2_setting_conf_clean(apollo::drivers::gnss::config::Config& config)
{
  std::vector<std::string> conf_item{"$cmd,set,netipport,","$cmd,set,mountpoint,","$cmd,set,netuser",
    "$cmd,set,localip,","$cmd,set,localmask,","$cmd,set,localgate,","$cmd,set,headoffset,",
    "$cmd,set,leverarm,gnss,","$cmd,save,config"};
  for (int i = 0; i< config.login_commands().size(); i++) {
    for(size_t j = 0; j< conf_item.size(); j++)
    {
      if(strstr((const char *)config.login_commands(i).c_str(), (const char *)conf_item[j].c_str()))
        config.mutable_login_commands()->DeleteSubrange(i,1);
    }
  }
  return 0;
}

int m2_conf_init(std::string config_file_path)
{
  m2_conf_path = config_file_path;

  return 0;
}

int m2_conf_logincommand_init(std::string config_file_path)
{
  m2_conf_path = config_file_path;
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);
  m2_setting_conf_clean(config);
  // config.mutable_login_commands()->Clear();
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_basestation_setting(std::string rtk_base_addr,std::string rtk_base_port)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  char ip_string[20] = "";
  m2_ippoint_to_ipcomma(rtk_base_addr.c_str(),ip_string);
  sprintf(buf,"$cmd,set,netipport,%s,%s*ff\n",ip_string,rtk_base_port.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_mountpoint_setting(std::string rtk_base_mountpoint)
{  
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$cmd,set,mountpoint,%s*ff\n",rtk_base_mountpoint.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_user_setting(std::string rtk_base_username,std::string rtk_base_passwd)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$cmd,set,netuser,%s:%s*ff\n",rtk_base_username.c_str(),rtk_base_passwd.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_localaddr_setting(std::string local_addr)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  char ip_string[20] = "";
  m2_ippoint_to_ipcomma(local_addr.c_str(),ip_string);
  sprintf(buf,"$cmd,set,localip,%s*ff\n",ip_string);

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_localnetmask_setting(std::string local_netmask)
{  
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  char ip_string[20] = "";
  m2_ippoint_to_ipcomma(local_netmask.c_str(),ip_string);
  sprintf(buf,"$cmd,set,localmask,%s*ff\n",ip_string);

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_localgateway_setting(std::string local_gateway)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  char ip_string[20] = "";
  m2_ippoint_to_ipcomma(local_gateway.c_str(),ip_string);
  sprintf(buf,"$cmd,set,localgate,%s*ff\n",ip_string);

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_headoffset_setting(std::string head_offset)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$cmd,set,headoffset,%s*ff\n",head_offset.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_leverarm1_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$cmd,set,leverarm,gnss,%s,%s,%s*ff\n",leverarm_x.c_str(),leverarm_y.c_str(),leverarm_z.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}

int m2_leverarm2_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z)
{
  return 1;
}

int m2_config_get()
{
  m2_config_tmp_buf.clear();
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  for (int i = 0; i< config.login_commands().size(); i++) {
    m2_config_tmp_buf += config.login_commands(i);
  }
  AERROR << "^^^^^^^^^^^^^^^^^^ "<< m2_config_tmp_buf; 
  return 1;
}


int m2_localip_get(std::string& localip)
{
  char ip[4][5] = {0};
  const char *p_localip = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"localip");AERROR << "^^^^^^^^^^^^^^^^^^ "<< m2_config_tmp_buf; 
  // printf("-------------*****************%d\n",__LINE__);
  if(!p_localip)
  {
    localip = "no config";
    return 0;
  }
  int ret_num = sscanf(p_localip, "localip,%[^,],%[^,],%[^,],%[^*]", ip[0], ip[1], ip[2], ip[3]);
  if(ret_num < 4)
  {
    localip = "error";
    return 0;
  }
  localip.clear();
  localip += ip[0];
  localip += ".";
  localip += ip[1];
  localip += ".";
  localip += ip[2];
  localip += ".";
  localip += ip[3];
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_localmask_get(std::string& localmask)
{
  char localmask_ip[4][5] = {0};
  const char *p_localmask = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"localmask");
  if(!p_localmask)
  {
    localmask = "no config";
    return 0;
  }
  int ret_num = sscanf(p_localmask, "localmask,%[^,],%[^,],%[^,],%[^*]", localmask_ip[0], localmask_ip[1], localmask_ip[2], localmask_ip[3]);
  localmask.clear();
  if(ret_num < 4)
  {
    localmask = "error";
    return 0;
  }
  localmask += localmask_ip[0];
  localmask += ".";
  localmask += localmask_ip[1];
  localmask += ".";
  localmask += localmask_ip[2];
  localmask += ".";
  localmask += localmask_ip[3];
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_localgateway_get(std::string& localgateway)
{
  char localgateway_ip[4][5] = {0};
  const char *p_localgateway = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"localgate");
  if(!p_localgateway)
  {
    localgateway = "no config";
    return 0;
  }
  int ret_num = sscanf(p_localgateway, "localgate,%[^,],%[^,],%[^,],%[^*]", localgateway_ip[0], localgateway_ip[1], localgateway_ip[2], localgateway_ip[3]);
  localgateway.clear();
  if(ret_num < 4)
  {
    localgateway = "error";
    return 0;
  }
  localgateway += localgateway_ip[0];
  localgateway += ".";
  localgateway += localgateway_ip[1];
  localgateway += ".";
  localgateway += localgateway_ip[2];
  localgateway += ".";
  localgateway += localgateway_ip[3];
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_headoffset_get(std::string& headoffset)
{
  char headoffset_value[30] = {0};
  const char *p_headoffset = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"headoffset");
  if(!p_headoffset)
  {
    headoffset = "no config";
    return 0;
  }
  int ret_num = sscanf(p_headoffset, "headoffset,%[^*]", headoffset_value);
  if(ret_num < 1)
  {
    headoffset = "error";
    return 0;
  }
  headoffset = headoffset_value;
  
  return 1;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_leverarm1_get(std::string& leverarm_x, std::string& leverarm_y, std::string& leverarm_z)
{
  char leverarm_value[3][30] = {0};
  const char *p_leverarm = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"leverarm,gnss");
  if(!p_leverarm)
  {
    leverarm_x = "no config";
    leverarm_y = "no config";
    leverarm_z = "no config";
    return 0;
  }
  int ret_num = sscanf(p_leverarm, "leverarm,gnss,%[^,],%[^,],%[^*]", leverarm_value[0], leverarm_value[1], leverarm_value[2]);
  if(ret_num < 3)
  {
    leverarm_x = "error";
    leverarm_y = "error";
    leverarm_z = "error";
    return 0;
  }
  leverarm_x = leverarm_value[0];
  leverarm_y = leverarm_value[1];
  leverarm_z = leverarm_value[2];
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_leverarm2_get(std::string& leverarm_x, std::string& leverarm_y, std::string& leverarm_z)
{
  leverarm_x = "no leverarm2 config";
  leverarm_y = "no leverarm2 config";
  leverarm_z = "no leverarm2 config";
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_basestation_get(std::string& rtk_base_addr,std::string& rtk_base_port)
{
  char basestation[5][10] = {0};
  const char *p_basestation = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"netipport");
  if(!p_basestation)
  {
    rtk_base_addr = "no config";
    rtk_base_port = "no config";
    return 0;
  }
  int ret_num = sscanf(p_basestation, "netipport,%[^,],%[^,],%[^,],%[^,],%[^*]", basestation[0], basestation[1], basestation[2], basestation[3], basestation[4]);
  rtk_base_addr.clear();
  if(ret_num < 4)
  {
    rtk_base_addr = "error";
    rtk_base_port = "error";
    return 0;
  }
  rtk_base_addr += basestation[0];
  rtk_base_addr += ".";
  rtk_base_addr += basestation[1];
  rtk_base_addr += ".";
  rtk_base_addr += basestation[2];
  rtk_base_addr += ".";
  rtk_base_addr += basestation[3];
  rtk_base_port = basestation[4];
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_user_get(std::string& rtk_base_username,std::string& rtk_base_passwd)
{
  char user_password[2][30] = {0};
  const char *p_user_password = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"netuser");
  if(!p_user_password)
  {
    rtk_base_username = "no config";
    rtk_base_passwd = "no config";
    return 0;
  }
  int ret_num = sscanf(p_user_password, "netuser,%[^:]:%[^*]", user_password[0], user_password[1]);
  if(ret_num < 2)
  {
    rtk_base_username = "error";
    rtk_base_passwd = "error";
    return 0;
  }
  rtk_base_username = user_password[0];
  rtk_base_passwd = user_password[1];
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_mountpoint_get(std::string& rtk_base_mountpoint)
{
  char mountpoint[30] = {0};
  const char *p_mountpoint = strstr((const char *)m2_config_tmp_buf.c_str(), (const char *)"mountpoint");
  if(!p_mountpoint)
  {
    rtk_base_mountpoint = "no config";
    return 0;
  }
  int ret_num = sscanf(p_mountpoint, "mountpoint,%[^*]", mountpoint);
  if(ret_num < 1)
  {
    rtk_base_mountpoint = "error";
    return 0;
  }
  rtk_base_mountpoint = mountpoint;
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int m2_setting_save_and_close()
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(m2_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$cmd,save,config*ff\n");

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, m2_conf_path);
  return 1;
}   

#endif  // MODULES_DREAMVIEW_BACKEND_HMI_M2SETTING_H_
