#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cell_matrix.h"

namespace apollo {
namespace perception {

float SppCell::distance(const SppCell& rhs) {
    float diff_x = x - rhs.x;
    float diff_y = y - rhs.y;
    float diff_z = altitude - rhs.altitude;
    float dist = sqrt(diff_x * diff_x + diff_y * diff_y);// + diff_z * diff_z);
    return dist;
}

void SppCellMatrix::add_point_cloud(
        pcl_util::PointCloudConstPtr point_cloud,
        const SppCloudMask& mask) {
    float half_width = static_cast<float>(_cols) / 2; 
    float half_height = static_cast<float>(_rows) / 2;
    int x = 0;
    int y = 0;
    float ratio_x = half_width / _range;
    float ratio_y = half_height / _range;
    int cols = static_cast<int>(_cols);
    int rows = static_cast<int>(_rows);
    unsigned int countp1 = 0;
    for (unsigned int i = 0; i < point_cloud->points.size(); ++i) {
        if (mask.size() && mask[i] == 0) {
            continue;
        }
        const pcl_util::Point& point = point_cloud->points[i];
        if (point.x * point.x + point.y * point.y < 25.f) {
            continue;
        }
        y = std::floor((_range - point.x) * ratio_x);
        x = std::floor((_range - point.y) * ratio_y);
        if (x < 0 || x >= cols || y < 0 || y >= rows) {
            continue;
        }
        else {
            if (_cells[y][x].count == 0) {
                _cells[y][x].id = _label_count++;
            }
            countp1 = 1 + _cells[y][x].count;
            _cells[y][x].x = (_cells[y][x].x * _cells[y][x].count 
                    + point.x) / countp1;
            _cells[y][x].y = (_cells[y][x].y * _cells[y][x].count 
                    + point.y) / countp1;
            _cells[y][x].altitude = (_cells[y][x].altitude * _cells[y][x].count 
                    + point.z) / countp1;
            //_cells[y][x].altitude = std::max(_cells[y][x].altitude, 
            //        point.z + 100.f);
            _cells[y][x].count = countp1;
        }
    }
}

}  // namespace perception
}  // namespace apollo
