#include "modules/common/log.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/timer.h"
#include <sstream>

namespace apollo {
namespace perception {

Timer::Timer() {
}

void Timer::start() {
    _start_time = boost::posix_time::microsec_clock::local_time();
}

void Timer::end(const char * title) {
    _end_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration dt = _end_time - _start_time;
    std::stringstream ss;
    if (title) {
        ss << title;
    }
    ss << " Elapsed time: " << dt.seconds() << "s " <<
        (dt.total_milliseconds() - dt.seconds()*1000) << "ms";
    AINFO << ss.str();
    _start_time = boost::posix_time::microsec_clock::local_time();
}

TimeAccumulator::TimeAccumulator() {

}

void TimeAccumulator::start() {
    _start_time = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::end(const char * title) {
    boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration dt = end_time - _start_time;
    _duration += dt;
    std::stringstream ss;
    if (title) {
        ss << title << " Elapsed time: " << dt.seconds() << "s " <<
            (dt.total_milliseconds() - dt.seconds()*1000) << "ms";
        AINFO << ss.str();
    }
    _start_time = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::clear() {
    _duration = boost::posix_time::time_duration();
}

void TimeAccumulator::get_duration(const char * title) {
    std::stringstream ss;
    if (title) {
        ss << title;
    }
    ss << " Total elapsed time: " << _duration.seconds() << "s " <<
        (_duration.total_milliseconds() - _duration.seconds()*1000) << "ms";
    AINFO << ss.str();
}

}  // namespace perception
}  // namespace apollo
