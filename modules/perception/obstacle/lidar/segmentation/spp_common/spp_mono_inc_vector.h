#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_MONO_INC_VECTOR_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_MONO_INC_VECTOR_H
#include <vector>
#include <memory>

namespace apollo {
namespace perception {

template <typename T>
class SppMonoIncVector {
public:
    SppMonoIncVector() : _valid_size(0) {
        _buffer.resize(_s_default_size, nullptr);
        for (auto& value : _buffer) {
            value.reset(new T);
        }
    }
    ~SppMonoIncVector() = default;
    inline void push_back(const std::shared_ptr<const T>& data) {
        ++_valid_size;
        if (_valid_size > _buffer.size()) {
            _buffer.push_back(data);
        }
        back()->clear();
    }
    inline void resize(std::size_t size) {
        std::size_t current_size = _buffer.size();
        if (current_size < size) {
            _buffer.resize(size, nullptr);
            for (std::size_t i = current_size; i < _buffer.size(); ++i) {
                _buffer[i].reset(new T);
            }
        }
        for (std::size_t i = _valid_size; i < size; ++i) {
            _buffer[i]->clear();
        }
        _valid_size = size;
    }
    inline void reserve(std::size_t size) {
        _buffer.reserve(size);
    }
    inline void clear() {
        for (auto& data : _buffer) {
            data->clear();
        }
        _valid_size = 0;
    }
    inline std::shared_ptr<T>& operator[] (std::size_t id) {
        return _buffer[id];
    }
    inline const std::shared_ptr<const T>& operator[] (std::size_t id) const {
        return _buffer[id];
    }
    inline std::size_t size() const {
        return _valid_size;
    }
    inline auto begin() -> 
        typename std::vector<std::shared_ptr<T> >::iterator {
        return _buffer.begin();
    }
    inline auto end() ->
        typename std::vector<std::shared_ptr<T> >::iterator {
        return _buffer.begin() + _valid_size;
    }
    inline auto back() ->
        typename std::vector<std::shared_ptr<T> >::iterator {
        return _buffer.begin() + _valid_size - 1;
    }
    inline auto begin() const -> 
        typename std::vector<std::shared_ptr<T> >::const_iterator {
        return _buffer.begin();
    }
    inline auto end() const ->
        typename std::vector<std::shared_ptr<T> >::const_iterator {
        return _buffer.begin() + _valid_size;
    }
    inline auto back() const ->
        typename std::vector<std::shared_ptr<T> >::const_iterator {
        return _buffer.begin() + _valid_size - 1;
    }
private:
    std::vector<std::shared_ptr<T> > _buffer;
    std::size_t _valid_size;
private:
    static const std::size_t _s_default_size = 500;
};

}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_MONO_INC_VECTOR_H
