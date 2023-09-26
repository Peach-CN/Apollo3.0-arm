#include <algorithm>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_label_map.h"

namespace apollo {
namespace perception {

unsigned int LabelObstacle::_s_width = 0;

void LabelMap::init(unsigned int width, unsigned int height) {
    if (_labels) {
        idl::i_free2(_labels);
    }
    _width = width;
    _height = height;
    _labels = idl::i_alloc2<unsigned short>(height, width);
    memset(_labels[0], 0, sizeof(unsigned short) * _width * _height);
    LabelObstacle::set_width(width);
}

void LabelMap::from_label_map() {
    unsigned int size = _width * _height; 
    unsigned short max_label = *(std::max_element(_labels[0], _labels[0] + size));
    _occs.resize(max_label);
    for (auto& occ : _occs) {
        if (occ == nullptr) {
            occ.reset(new LabelObstacle);
        } else {
            occ->clear();
        }
    }
    for (unsigned int y = 0; y < _height; ++y) {
        for (unsigned int x = 0; x < _width; ++x) {
            unsigned short& label = _labels[y][x];
            if (label) {
                _occs[label-1]->push_back(x, y);
            }
        }
    }
}

//void LabelMap::to_label_map() {
//    memset(_labels[0], 0, sizeof(unsigned short) * _width * _height);
//    int x = 0;
//    int y = 0;
//    for (unsigned int n = 0; n < _occs.size(); ++n) {
//        auto& occ = _occs[n];
//        for (unsigned int i = 0; i < occ->size(); ++i) {
//            occ->get_xy(i, &x, &y);
//            _labels[y][x] = static_cast<unsigned short>(n + 1);
//        }
//    }
//}

void LabelMap::filter_label_map(const float* confidence_map, float threshold) {
    for (unsigned int n = 0; n < _occs.size(); ++n) {
        float sum = 0.f;
        unsigned int count = 0;
        for (unsigned int i = 0; i < _occs[n]->size(); ++i) {
            sum += confidence_map[_occs[n]->get_xy_id(i)];
            ++count;
        }
        sum = count > 0 ? sum / count : sum; 
        _occs[n]->set_confidence(sum);
    }

    unsigned int current = 0;
    for (unsigned int n = 0; n < _occs.size(); ++n) {
        if (_occs[n]->get_confidence() >= threshold) {
            if (current != n) {
                *_occs[current] = *_occs[n];
                for (unsigned int j = 0; j < _occs[n]->size(); ++j) {
                    _labels[0][_occs[n]->get_xy_id(j)] = current + 1;
                }
            }
            ++current;
        } else {
            for (unsigned int j = 0; j < _occs[n]->size(); ++j) {
                _labels[0][_occs[n]->get_xy_id(j)] = 0;
            }

        } 
    }
    _occs.resize(current);
}

void LabelMap::cal_label_class(const float* class_map, unsigned int class_num) {
    for (auto& occ : _occs) {
        occ->get_probs().assign(class_num, 0.f);
    }
    unsigned int size = _width * _height;
    for (unsigned int c = 0; c < class_num; ++c) {
        const float* class_map_ptr = class_map + c * size; 
        for (unsigned int n = 0; n < _occs.size(); ++n) {
            auto& probs = _occs[n]->get_probs();
            for (unsigned int i = 0; i < _occs[n]->size(); ++i) {
                probs[c] += class_map_ptr[_occs[n]->get_xy_id(i)];
            }
        }
    }
    for (auto& occ : _occs) {
        auto& probs = occ->get_probs();
        float sum = std::accumulate(probs.begin(), probs.end(), 0.f);
        if (sum > 1e-9) {
            for (auto& value : probs) {
                value /= sum;
            }
        }
    }
}

void LabelMap::cal_label_heading(const float* heading_map) {
    std::vector<std::pair<float, float> > directions(_occs.size(), 
            std::make_pair(0.f, 0.f));
    const float* heading_map_x_ptr = heading_map;
    const float* heading_map_y_ptr = heading_map + _width * _height;

    for (unsigned int n = 0; n < _occs.size(); ++n) {
        for (unsigned int i = 0; i < _occs[n]->size(); ++i) {
            directions[n].first += heading_map_x_ptr[_occs[n]->get_xy_id(i)];
        }
    }
    for (unsigned int n = 0; n < _occs.size(); ++n) {
        for (unsigned int i = 0; i < _occs[n]->size(); ++i) {
            directions[n].second += heading_map_y_ptr[_occs[n]->get_xy_id(i)];
        }
        _occs[n]->set_yaw(std::atan2(directions[n].second, directions[n].first) * 0.5f);
    }
}

void LabelMap::dilate_erode_occs(int dilate_erode_times) {
    for (unsigned int i = 0; i < _occs.size(); ++i) {
        dilate_erode_occ(dilate_erode_times, i);
    }
}

void LabelMap::dilate_erode_occ(int dilate_erode_times, unsigned int occ_id) {
    unsigned short cur_obj_id = _labels[0][_occs[occ_id]->get_xy_id(0)];
    if (dilate_erode_times < 0) { //erode
        for (unsigned int i = 0; i < -dilate_erode_times; ++i) {
            //to do
        }
    }
    if (dilate_erode_times > 0) { // dilate
        for (unsigned int i = 0; i< dilate_erode_times; ++i) {
            unsigned int src_size = _occs[occ_id]->size();
            _occs[occ_id]->reserve(src_size * 4);
            for (unsigned int j = 0; j < src_size; ++j) {
                unsigned int map_pos = _occs[occ_id]->get_xy_id(j);
                unsigned int pos_x = map_pos % _width;
                unsigned int pos_y = map_pos / _width;
                if (pos_x <= 0 || _width - 1 <= pos_x ||
                        pos_y <= 0 || _height - 1 <= pos_y) {
                    continue;
                }

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        unsigned int cur_pos = map_pos + dx + dy * _width;
                        if (_labels[0][cur_pos] == 0) {
                            _labels[0][cur_pos] = cur_obj_id;
                            _occs[occ_id]->push_back(pos_x + dx, pos_y + dy);
                        }
                    }
                }
            }
        }
    }
}

void LabelMap::cal_label_top_z(const float* top_z_map) {
    for (unsigned int n = 0; n < _occs.size(); ++n) {
        float sum = 0.f;
        unsigned int count = 0;
        for (unsigned int i = 0; i < _occs[n]->size(); ++i) {
            sum += top_z_map[_occs[n]->get_xy_id(i)];
            ++count;
        }
        sum = count > 0 ? sum / count : sum; 
        _occs[n]->set_top_z(sum);
    }
}

void LabelMap::push_back(unsigned int id, int x, int y) {
    if (_occs.size() <= id) {
        _occs.resize(id + 1);
    }
    if (_occs[id] == nullptr) {
        _occs[id].reset(new LabelObstacle);
    }
    _occs[id]->push_back(x, y);
}

void LabelMap::clear_occs() {
    _occs.clear();
    //for (auto& occ : _occs) {
    //    occ->clear();
    //}
}

}  // namespace perception
}  // namespace apollo

