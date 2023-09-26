#ifndef MODULES_DREAMVIEW_BACKEND_HMI_ZHDSETTING_H_
#define MODULES_DREAMVIEW_BACKEND_HMI_ZHDSETTING_H_
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/common/util/file.h"

std::string zhd_config_tmp_buf;
std::string zhd_conf_path("222");

static void zhd_ippoint_to_ipcomma(const char *ippoint, char *ipcomma)
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

int zhd_setting_conf_clean(apollo::drivers::gnss::config::Config& config)
{
  std::vector<std::string> conf_item{"$HSET,LEVER1,","$HSET,LEVER2,"};
  for (int i = 0; i< config.login_commands().size(); i++) {
    for(size_t j = 0; j< conf_item.size(); j++)
    {
      if(strstr((const char *)config.login_commands(i).c_str(), (const char *)conf_item[j].c_str()))
        config.mutable_login_commands()->DeleteSubrange(i,1);
    }
  }
  return 0;
}

int zhd_conf_init(std::string config_file_path)
{
  zhd_conf_path = config_file_path;

  return 0;
}

int zhd_conf_logincommand_init(std::string config_file_path)
{
  zhd_conf_path = config_file_path;
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);
  zhd_setting_conf_clean(config);
  // config.mutable_login_commands()->Clear();
  apollo::common::util::SetProtoToASCIIFile(config, zhd_conf_path);
  return 1;
}

int zhd_basestation_setting(std::string rtk_base_addr,std::string rtk_base_port)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  char ip_string[20] = "";
  zhd_ippoint_to_ipcomma(rtk_base_addr.c_str(),ip_string);
  config.mutable_rtk_from()->mutable_ntrip()->set_address(rtk_base_addr);
  config.mutable_rtk_from()->mutable_ntrip()->set_port(atoi(rtk_base_port.c_str()));

  apollo::common::util::SetProtoToASCIIFile(config, zhd_conf_path);
  return 1;
}

int zhd_mountpoint_setting(std::string rtk_base_mountpoint)
{  
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  config.mutable_rtk_from()->mutable_ntrip()->set_mount_point(rtk_base_mountpoint);

  apollo::common::util::SetProtoToASCIIFile(config, zhd_conf_path);
  return 1;
}

int zhd_user_setting(std::string rtk_base_username,std::string rtk_base_passwd)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  config.mutable_rtk_from()->mutable_ntrip()->set_user(rtk_base_username);
  config.mutable_rtk_from()->mutable_ntrip()->set_password(rtk_base_passwd);

  apollo::common::util::SetProtoToASCIIFile(config, zhd_conf_path);
  return 1;
}

int zhd_localaddr_setting(std::string local_addr)
{
  return 1;
}

int zhd_localnetmask_setting(std::string local_netmask)
{  
  return 1;
}

int zhd_localgateway_setting(std::string local_gateway)
{
  return 1;
}

int zhd_headoffset_setting(std::string head_offset)
{
  return 1;
}

int zhd_leverarm1_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$HSET,LEVER1,%s,%s,%s\r\n",leverarm_x.c_str(),leverarm_y.c_str(),leverarm_z.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, zhd_conf_path);
  return 1;
}

int zhd_leverarm2_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  char buf[100] = "";
  sprintf(buf,"$HSET,LEVER2,%s,%s,%s\r\n",leverarm_x.c_str(),leverarm_y.c_str(),leverarm_z.c_str());

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  config.add_login_commands(buf);
  apollo::common::util::SetProtoToASCIIFile(config, zhd_conf_path);
  return 1;
}


int zhd_config_get()
{
  zhd_config_tmp_buf.clear();
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  // AERROR << "^^^^^^^^^^^^^^^^^^config.login_commands().size()=" << config.login_commands().size();
  for (int i = 0; i< config.login_commands().size(); i++) {
    zhd_config_tmp_buf += config.login_commands(i);
  }
  AERROR << "^^^^^^^^^^^^^^^^^^ "<< zhd_config_tmp_buf; 
  return 1;
}


int zhd_localip_get(std::string& localip)
{
  localip = "no localip config";
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_localmask_get(std::string& localmask)
{
  localmask = "no localmask config";
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_localgateway_get(std::string& localgateway)
{
  localgateway = "no localgateway config";
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_headoffset_get(std::string& headoffset)
{
  headoffset = "no headoffset config";
  
  return 1;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_leverarm1_get(std::string& leverarm_x, std::string& leverarm_y, std::string& leverarm_z)
{
  char leverarm_value[3][30] = {0};
  const char *p_leverarm = strstr((const char *)zhd_config_tmp_buf.c_str(), (const char *)"$HSET,LEVER1,");
  if(!p_leverarm)
  {
    leverarm_x = "no config";
    leverarm_y = "no config";
    leverarm_z = "no config";
    return 0;
  }
  int ret_num = sscanf(p_leverarm, "$HSET,LEVER1,%[^,],%[^,],%[^\r\n]", leverarm_value[0], leverarm_value[1], leverarm_value[2]);
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

int zhd_leverarm2_get(std::string& leverarm_x, std::string& leverarm_y, std::string& leverarm_z)
{
  char leverarm_value[3][30] = {0};
  const char *p_leverarm = strstr((const char *)zhd_config_tmp_buf.c_str(), (const char *)"$HSET,LEVER2,");
  if(!p_leverarm)
  {
    leverarm_x = "no config";
    leverarm_y = "no config";
    leverarm_z = "no config";
    return 0;
  }
  int ret_num = sscanf(p_leverarm, "$HSET,LEVER2,%[^,],%[^,],%[^\r\n]", leverarm_value[0], leverarm_value[1], leverarm_value[2]);
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

int zhd_basestation_get(std::string& rtk_base_addr,std::string& rtk_base_port)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  rtk_base_addr = config.rtk_from().ntrip().address();
  rtk_base_port = std::to_string(config.rtk_from().ntrip().port());
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_user_get(std::string& rtk_base_username,std::string& rtk_base_passwd)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  rtk_base_username = config.rtk_from().ntrip().user();
  rtk_base_passwd = config.rtk_from().ntrip().password();
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_mountpoint_get(std::string& rtk_base_mountpoint)
{
  apollo::drivers::gnss::config::Config config;
  apollo::common::util::GetProtoFromFile(zhd_conf_path.c_str(), &config);

  rtk_base_mountpoint = config.rtk_from().ntrip().mount_point();
  
  return 0;
  //HMIWorker::instance()->RunToolCommand("leverarm_x", &leverarm_x);
}

int zhd_setting_save_and_close()
{
  return 1;
}   

#endif  // MODULES_DREAMVIEW_BACKEND_HMI_ZHDSETTING_H_
