#ifndef MODULES_DREAMVIEW_BACKEND_HMI_GNSSSETTING_H_
#define MODULES_DREAMVIEW_BACKEND_HMI_GNSSSETTING_H_

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <vector>
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/common/util/file.h"

extern int fd;

extern int gnss_conf_init();

extern int gnss_conf_logincommand_init();

extern int gnss_basestation_setting(std::string rtk_base_addr,std::string rtk_base_port);

extern int gnss_mountpoint_setting(std::string rtk_base_mountpoint);

extern int gnss_user_setting(std::string rtk_base_username,std::string rtk_base_passwd);

extern int gnss_localaddr_setting(std::string local_addr);

extern int gnss_localnetmask_setting(std::string local_netmask);

extern int gnss_localgateway_setting(std::string local_gateway);

extern int gnss_headoffset_setting(std::string head_offset);

extern int gnss_leverarm1_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z);

extern int gnss_leverarm2_setting(std::string leverarm_x,std::string leverarm_y,std::string leverarm_z);

extern int gnss_setting_save_and_close();

extern int gnss_config_get();

extern int gnss_localip_get(std::string& localip);

extern int gnss_localmask_get(std::string& localmask);

extern int gnss_localgateway_get(std::string& localgateway);

extern int gnss_headoffset_get(std::string& headoffset);

extern int gnss_leverarm1_get(std::string& leverarm_x,std::string& leverarm_y,std::string& leverarm_z);

extern int gnss_leverarm2_get(std::string& leverarm_x,std::string& leverarm_y,std::string& leverarm_z);

extern int gnss_basestation_get(std::string& rtk_base_addr,std::string& rtk_base_port);

extern int gnss_user_get(std::string& rtk_base_username,std::string& rtk_base_passwd);

extern int gnss_mountpoint_get(std::string& rtk_base_mountpoint);

#endif  // MODULES_DREAMVIEW_BACKEND_HMI_GNSSSETTING_H_
