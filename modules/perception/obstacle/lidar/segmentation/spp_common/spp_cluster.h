#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CLUSTER_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CLUSTER_H
#include <vector>
#include <iostream>
#include <fstream>
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_vector_pool.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

static const float g_default_top_z = 50.f;

struct SppPoint {
    float x;
    float y;
    float z;
    float h;
    int point_id;

    SppPoint() : x(0.f), y(0.f), z(0.f), h(0.f), point_id(-1) {
    }

    SppPoint(const pcl_util::Point& point,
            int point_id) {
        x = point.x;
        y = point.y;
        z = point.z;
        h = point.h;
        this->point_id = point_id;
    }

    SppPoint(const SppPoint& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        h = rhs.h;
        point_id = rhs.point_id;
    }

    SppPoint& operator=(const SppPoint& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        h = rhs.h;
        point_id = rhs.point_id;
        return *this;
    }
};

enum SppClassType {
    SPPOTHERS = 0,
    SPPSMALLMOT = 1,
    SPPBIGMOT = 2,
    SPPCYCLIST = 3,
    SPPPEDESTRIAN = 4,
    SPPCONE = 5,
};

std::string translate_type_to_string(const SppClassType& type);

SppClassType translate_string_to_type(const std::string& str);

struct SppCluster {
    std::vector<SppPoint>* points;
    unsigned int key;

    std::vector<float> class_prob;
    std::vector<float> direction;
    SppClassType type;
    float yaw;
    float confidence;
    float top_z;
    static bool output_score;

    static SppVectorPool<SppPoint> s_pool;
    static const unsigned int s_default_key = UINT_MAX;

    SppCluster() : type(SPPOTHERS), yaw(0.f), confidence(1.f), top_z(g_default_top_z),
                    key(s_default_key) {
        allocate_memory();
    }

    SppCluster(const SppCluster& rhs) {
        allocate_memory();
        *points = *(rhs.points);
        class_prob = rhs.class_prob;
        direction = rhs.direction;
        type = rhs.type;
        yaw = rhs.yaw;
        confidence = rhs.confidence;
        top_z = rhs.top_z;
    }

    ~SppCluster() {
        release_memory();
    }

    inline void add_sample(const pcl_util::Point& rhs, int point_id) {
        points->push_back(SppPoint(rhs, point_id));
    }

    inline unsigned int size() const {
        return points->size();
    }

    inline void resize(unsigned int size) {
        points->resize(size);
    }

    inline void swap(SppCluster& rhs) {
        std::swap(points, rhs.points);
        std::swap(key, rhs.key);
        std::swap(type, rhs.type);
        std::swap(yaw, rhs.yaw);
        std::swap(confidence, rhs.confidence);
        class_prob.swap(rhs.class_prob);
        direction.swap(rhs.direction);
        std::swap(top_z, rhs.top_z);
    }

    inline void clear() {
        points->clear();
        class_prob.clear();
        direction.clear();
        type = SPPOTHERS;
        yaw = 0.f;
        confidence = 1.f;
        top_z = g_default_top_z;
    }

    void allocate_memory() {
        if (!s_pool.get_vector(points, &key)) {
            AERROR << "Fail to init points!";
            AERROR << s_pool;
            points = new std::vector<SppPoint>;
            key = s_default_key;
        } 
    }

    void release_memory() {
        if (key == s_default_key) {
            delete points;
        }
        else if (!s_pool.release_vector(key)) {
            AERROR << "Fail to release vector, key " << key;
            AERROR << s_pool;
        } 
    }

    friend std::ostream& operator << (std::ostream& out, const SppCluster& rhs);
};

}  // namespace perception
}  // namespace apollo

#endif
