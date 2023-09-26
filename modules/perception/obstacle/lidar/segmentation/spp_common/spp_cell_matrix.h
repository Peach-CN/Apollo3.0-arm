#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CELL_MATRIX_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CELL_MATRIX_H

#include "modules/perception/obstacle/lidar/segmentation/spp_common/i_alloc.h"
#include <vector>
#include <iostream>
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cloud_mask.h"

namespace apollo {
namespace perception {

struct SppCell {
    SppCell() : x(0.f), y(0.f), altitude(0.f), id(0.f), count(0) {
    }
    float distance(const SppCell& rhs);
    float x;
    float y;
    float altitude;
    unsigned int id;
    unsigned int count;
};

class SppCellMatrix {
public:
    SppCellMatrix() : _cells(NULL), _rows(0), _cols(0), _range(0), _label_count(0) {
    }
    ~SppCellMatrix() {
        if (_cells) {
            idl::i_free2(_cells);
        }
    }
    inline void init(unsigned int rows, unsigned int cols, float range) {
        if (_cells) {
            idl::i_free2(_cells);
        }
        _cells = idl::i_alloc2<SppCell>(rows, cols);
        _rows = rows;
        _cols = cols;
        _range = range;
        _label_count = 0;
    }
    inline void reset() {
        if (_cells) {
            memset(_cells[0], 0, sizeof(SppCell) * _rows * _cols);
        }
        _label_count = 0;
    }
    inline SppCell* operator [](int row) {
        return _cells[row];
    }
    inline const SppCell* operator [](int row) const {
        return _cells[row];
    }
    inline unsigned int get_cols() const {
        return _cols;
    }
    inline unsigned int get_rows() const {
        return _rows;
    }
    inline float get_range() const {
        return _range;
    }
    inline unsigned int get_label_count() const {
        return _label_count;
    }
    void add_point_cloud(
            pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask);
private:
    SppCell** _cells;
    unsigned int _rows;
    unsigned int _cols;
    unsigned int _range;
    unsigned int _label_count;
};

}  // namespace perception
}  // namespace apollo
#endif
