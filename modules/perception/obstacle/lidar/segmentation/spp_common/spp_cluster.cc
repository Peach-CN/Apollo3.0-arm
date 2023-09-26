#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"
#include <boost/algorithm/string.hpp>
//#include <xlog.h>

namespace apollo {
namespace perception {

bool SppCluster::output_score = false;

SppVectorPool<SppPoint> SppCluster::s_pool;

std::ostream& operator << (std::ostream& out, const SppCluster& rhs) {
    if (rhs.size() > 0) {
        out << translate_type_to_string(rhs.type) << " ";
        if (SppCluster::output_score) {
            out << rhs.confidence << " ";
        }
        out << 1 << " " << std::tan(rhs.yaw) << " " << rhs.size() << " ";
        for (auto& point : *rhs.points) {
            out << point.x << " " << point.y << " " 
                << point.z << " " << 0 << " "; 
        }
        out << std::endl;
    } 
    return out;
}

std::string translate_type_to_string(const SppClassType& type) {
    switch (type) {
        case SPPOTHERS:
            return "unknow";
        case SPPPEDESTRIAN:
            return "pedestrian";
        case SPPCYCLIST:
            return "nonMot";
        case SPPSMALLMOT:
            return "smallMot";
        case SPPBIGMOT:
            return "smallMot"; // note here we dont distinguish small and big mot
        case SPPCONE:
            return "cone";
        default: 
            return "unknow";
    }
}

SppClassType translate_string_to_type(const std::string& str) {
    if (str == "bigMot" || str == "smallMot" || str == "vehicle") {
        return SPPSMALLMOT;
    } else if (str == "pedestrian") {
        return SPPPEDESTRIAN;
    } else if (str == "nonMot" || str == "bicyclist" || str == "motorcyclist") {
        return SPPCYCLIST;
    } else if (str == "TrafficCone") {
        return SPPCONE;
    } else {
        return SPPOTHERS;
    } 
}

}  // namespace perception
}  // namespace apollo
