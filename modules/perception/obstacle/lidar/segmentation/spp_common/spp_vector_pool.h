#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_VECTOR_POOL_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_VECTOR_POOL_H

#include <set>
#include <vector>
#include <iostream>

namespace apollo {
namespace perception {

template <typename T>
class SppVectorPool {
public:
    SppVectorPool() {
        init();
    }
    void init();
    bool get_vector(std::vector<T>* &vec, unsigned int* key);
    bool release_vector(unsigned int key);
    friend std::ostream& operator << (std::ostream& out, const SppVectorPool& rhs) {
        out << "Busy: " << rhs._busy_ids.size() << " Free: " << rhs._free_ids.size()
            << " Total: " << rhs._pool.size() << std::endl;
        out << "Busy ids: ";
        for (auto& id : rhs._busy_ids) {
            out << id;
        }
        out << "Free ids: ";
        for (auto& id : rhs._free_ids) {
        }
        return out;
    }
private:
    std::vector<std::vector<T> > _pool;
    std::set<unsigned int> _busy_ids;
    std::set<unsigned int> _free_ids;
private:
    static const unsigned int _s_max_vector_length = 4000;
    static const unsigned int _s_init_element_length = 1000;
};

template <typename T>
void SppVectorPool<T>::init() {
    _pool.resize(_s_max_vector_length);
    for (auto& v : _pool) {
        v.reserve(_s_init_element_length);
    }
    _busy_ids.clear();
    _free_ids.clear();
    for (unsigned int i = 0; i < _pool.size(); ++i) {
        _free_ids.insert(i);
    }
}

template <typename T>
bool SppVectorPool<T>::get_vector(std::vector<T>* &vec, unsigned int* key) {
    if (_free_ids.size() > 0) {
        *key = *_free_ids.begin();
        _busy_ids.insert(*key);
        _free_ids.erase(_free_ids.begin());
        vec = &_pool[*key];
        vec->clear();
    } else {
        return false;
    }
    return true;
}

template <typename T>
bool SppVectorPool<T>::release_vector(unsigned int key) {
    auto iter = _busy_ids.find(key);
    if (iter == _busy_ids.end()) {
        return false;
    }
    _busy_ids.erase(iter);
    _free_ids.insert(key);
    return true;
}

}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_VECTOR_POOL_H
