
# include <iostream>
# include <ros/ros.h>
# include <string>
# include "mpc_pkg/logger.h"


Logger::Logger(std::string prefix, int log_frequence, bool enable_logging)
:prefix(prefix), log_frequence(log_frequence), enable_logging(enable_logging)
{
    std::string para_name;
    para_name = prefix + "_enable_logging";
    ros::param::get(para_name, enable_logging);
}


void Logger::log(const std::string& msg, int type)
{
    // type
    //   -1 -> error
    //   0 -> normal
    //   1 -> important
    if (enable_logging)
    {
        std::string prefix_temp = prefix;
        auto out_put_msg = prefix_temp + ": " + msg;
        if (type == 0 && log_counts % log_frequence == 0)
        {
            ROS_INFO(out_put_msg.c_str());
        }
        else if(type == 1)
        {
            ROS_BLUE_STREAM(out_put_msg.c_str());
        }

        if(type == -1)
        {
            ROS_RED_STREAM(out_put_msg.c_str());
        }

        std::string end_s = "end";
        if (!end_s.compare(msg))
        {
                log_counts += 1;
        }
    }

}

void Logger::get_enable(const bool& a)
{
    enable_logging = a;
}

std::ostream& pc::operator<<(std::ostream& os, PRINT_COLOR c)
{
switch(c)
{
    case BLACK    : os << "\033[1;30m"; break;
    case RED      : os << "\033[1;31m"; break;
    case GREEN    : os << "\033[1;32m"; break;
    case YELLOW   : os << "\033[1;33m"; break;
    case BLUE     : os << "\033[1;34m"; break;
    case MAGENTA  : os << "\033[1;35m"; break;
    case CYAN     : os << "\033[1;36m"; break;
    case WHITE    : os << "\033[1;37m"; break;
    case ENDCOLOR : os << "\033[0m";    break;
    default       : os << "\033[1;37m";
}
return os;
}