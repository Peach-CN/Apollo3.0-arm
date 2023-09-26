#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_LABEL_MAP_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_LABEL_MAP_H
#include <vector>
#include <memory>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/i_alloc.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_mono_inc_vector.h"

namespace apollo {
namespace perception {

class LabelObstacle {
public:
    LabelObstacle() : _confidence(1.f), _yaw(0.f), _top_z(10.f) {
        reserve(_s_reserve_pixel_num);
    }
    void get_xy(unsigned int id, int* x, int* y) const {
        const auto& pair = _xys[id];
        *x = pair.first;
        *y = pair.second;
    }
    unsigned int get_xy_id(unsigned int id) const {
        return _xy_ids[id];
    }
    void push_back(int x, int y) {
        _xys.push_back(std::make_pair(x, y));
        _xy_ids.push_back(y * _s_width + x);
    }
    unsigned int size() const {
        return _xys.size();
    }
    void clear() {
        _xys.clear();
        _xy_ids.clear();
        _class_probs.clear();
        _confidence = 1.f;
        _yaw = 0.f;
    }
    void resize(unsigned int size) {
        _xys.resize(size);
        _xy_ids.resize(size);
    }
    void reserve(unsigned int size) {
        _xys.reserve(size);
        _xy_ids.reserve(size);
    }
    float get_confidence() const {
        return _confidence;
    }
    void set_confidence(const float& confidence) {
        _confidence = confidence;
    }
    std::vector<float>& get_probs() {
        return _class_probs;
    }
    const std::vector<float>& get_probs() const {
        return _class_probs;
    }
    void set_yaw(float yaw) {
        _yaw = yaw;
    }
    float get_yaw() const {
        return _yaw;
    }
    void set_top_z(float top_z) {
        _top_z = top_z;
    }
    float get_top_z() const {
        return _top_z;
    }
public:
    static void set_width(unsigned int width) {
        _s_width = width;
    }
private:
    std::vector<std::pair<int, int> > _xys; 
    std::vector<int> _xy_ids;
    std::vector<float> _class_probs;
    float _confidence;
    float _yaw;
    float _top_z;
private:
    static unsigned int _s_width;
    static const unsigned int _s_reserve_pixel_num = 500;
};

typedef std::shared_ptr<LabelObstacle> LabelObstaclePtr;
typedef std::shared_ptr<const LabelObstacle> LabelObstacleConstPtr;

class LabelMap {
public:
    LabelMap() : _labels(nullptr), _width(0), _height(0) {
        _occs.reserve(_s_reserve_occ_num);
    }
    ~LabelMap() {
        if (_labels) {
            idl::i_free2(_labels);
        }
    }
    void init(unsigned int width, unsigned int height);
    void from_label_map();
    //void to_label_map();
    void filter_label_map(const float* confidence_map, float threshold);
    void cal_label_class(const float* class_map, unsigned int class_num);
    void cal_label_heading(const float* heading_map);
    void dilate_erode_occs(int dilate_erode_times);
    void dilate_erode_occ(int dilate_erode_times, unsigned int occ_id);
    void cal_label_top_z(const float* top_z_map);
    unsigned short** get_label_map() {
        return _labels;
    }
    const unsigned short* const* get_label_map() const {
        return _labels;
    }
    unsigned short* operator[] (unsigned int id) {
        return _labels[id];
    }
    const unsigned short* operator[] (unsigned int id) const {
        return _labels[id];
    }
    unsigned int get_obstacle_num() const {
        return _occs.size();
    }
    LabelObstaclePtr get_obstacle(unsigned int id) {
        return _occs[id];
    }
    LabelObstacleConstPtr get_obstacle(unsigned int id) const {
        return _occs[id];
    }
    void push_back(unsigned int id, int x, int y);
    void clear_occs();
private:
    unsigned short** _labels;
    unsigned int _width;
    unsigned int _height;
    //std::vector<LabelObstaclePtr> _occs;
    SppMonoIncVector<LabelObstacle> _occs;
private:
    static const unsigned int _s_reserve_occ_num = 500;
};

typedef std::shared_ptr<LabelMap> LabelMapPtr;
typedef std::shared_ptr<const LabelMap> LabelMapConstPtr;

}  // namespace perception
}  // namespace apollo

#endif
