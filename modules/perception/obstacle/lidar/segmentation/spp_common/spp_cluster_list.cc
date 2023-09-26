#include "modules/common/log.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster_list.h"

namespace apollo {
namespace perception {

unsigned int SppClusterList::move_from_tail(unsigned int move_num, SppClusterList& rhs) {
    unsigned int size = _clusters.size();
    unsigned int valid_num = std::min(move_num, size);  
    for (unsigned int i = 0; i < valid_num; ++i) {
        rhs._clusters.push_back(SppCluster());
        _clusters.back().swap(rhs._clusters.back());
        _clusters.resize(_clusters.size() - 1);
    }
    return valid_num;
}

void SppClusterList::merge(SppClusterList& rhs) {
    unsigned int size = _clusters.size();
    unsigned int size_plus = _clusters.size() + rhs._clusters.size();
    _clusters.resize(size_plus);
    for (unsigned int i = 0; i < rhs._clusters.size(); ++i) {
        _clusters[size+i].swap(rhs._clusters[i]);
    }
}

unsigned int SppClusterList::analyze_clusters(float max_gap, 
        unsigned int start_id, 
        bool select_max) {
    unsigned int size = _clusters.size();
    unsigned int count = 0;
    for (unsigned int i = start_id; i < size; ++i) {
        if (_clusters[i].size() > 0) {
            if (compute_altitude_and_split_cluster(i, max_gap, select_max)) {
                ++count;
            }
        }
    }
    AINFO << "Split " << count << " clusters in 3d";
    return count;
}

bool SppClusterList::compute_altitude_and_split_cluster(
        unsigned int id, float max_gap, bool select_max) {
    if (id >= _clusters.size()) {
        return false;
    }
    std::vector<SppPoint>& points = *(_clusters[id].points);
    std::sort(points.begin(), points.end(), 
            [](const SppPoint& lhs, const SppPoint& rhs) {
            return lhs.z < rhs.z;
            }); 
    float gap = 0.f;
    std::vector<unsigned int> split_indices(1, 0);
    for (unsigned int i = 1; i < points.size(); ++i) {
        if (points[i].h < 0) {
            continue;
        }
        gap = points[i].z - points[i-1].z;
        if (gap > max_gap) {
            split_indices.push_back(i);
        }
    }
    unsigned int split_num = split_indices.size();
    if (split_num > 0) {
        unsigned int max_length = 0;
        unsigned int max_index = 0;
        unsigned int length = 0;
        for (unsigned int i = 1; i < split_indices.size(); ++i) {
            length = split_indices[i] - split_indices[i-1];
            if (length > max_length) {
                max_length = length;
                max_index = i - 1;
            }
        }
        if (select_max) {
            length = points.size() - split_indices[split_indices.size() - 1];
            if (length > max_length) {
                max_length = length;
                max_index = split_indices.size() - 1;
            }
        }
        if (max_index != split_indices.size() - 1) {//need split
            unsigned int split_index = split_indices[max_index+1];
            //SppCluster new_cluster;
            //for (unsigned int i = split_index; i < points.size(); ++i) {
            //    new_cluster.points.push_back(points[i]);
            //}
            points.resize(split_index);

            // note resize should be done before this operation since the re-allocate matter
            //_clusters.push_back(new_cluster); 
            return true;
        }
    }
    return false;
}

std::ostream& operator << (std::ostream& out, const SppClusterList& rhs) {
    for (unsigned int i = 0; i < rhs._clusters.size(); ++i) {
        out << rhs._clusters[i];
    }
    return out;
}

void SppClusterList::remove_empty_clusters() {
    unsigned int size = _clusters.size();
    if (!size) {
        return;
    }
    if (size == 1) {
        if (!_clusters[0].size()) {
            _clusters.clear();
        }
        return;
    }
    unsigned int i = 0;
    unsigned int j = _clusters.size() - 1;
    while (true) {
        while (i < j && _clusters[i].size()) {
            ++i;
        }
        while (i < j && !_clusters[j].size()) {
            --j;
        }
        if (i < j) {
            _clusters[i].swap(_clusters[j]);
            ++i;
            --j;
        }
        else {
            break;
        }
    }
    size = 0;
    while (size < _clusters.size() && _clusters[size].size()) {
        ++size;
    }
    AINFO << "find " << _clusters.size() - size << " empty out of " 
        << _clusters.size() << " clusters";
    _clusters.resize(size);
}

void SppClusterList::filter_clusters(float threshold, 
        unsigned int start_id, 
        const float* data) {
    unsigned int n = start_id;
    for (unsigned int i = start_id; i < _clusters.size(); ++i) {
        if (_clusters[i].confidence >= threshold) {
            if (i != n) {
                _clusters[n].swap(_clusters[i]);
            }
            ++n;
        }
    }
    AINFO << "filter " << _clusters.size() - n << " low confidence clusters of " 
        << _clusters.size();
    _clusters.resize(n);
}

void SppClusterList::filter_clusters(float threshold, 
        SppClusterList& rhs, unsigned int start_id,
        const float* data) {
    if (_clusters.size() <= start_id) {
        return;
    }
    unsigned int i = start_id;
    unsigned int n = 0;
    unsigned int size = _clusters.size();
    while (i < _clusters.size()) {
        if (_clusters[i].confidence < threshold) {
            split_into_another_cluster(rhs, i);
            ++n;
        }
        else {
            ++i;
        }
    }
    AINFO << "filter " << n << " low confidence clusters of " << size
        << " left " << _clusters.size() << " clusters";
}

void SppClusterList::filter_clusters(float threshold, 
        SppCluster& rhs, unsigned int start_id,
        const float* data) {
    if (_clusters.size() <= start_id) {
        return;
    }
    unsigned int n = start_id;
    for (unsigned int i = start_id; i < _clusters.size(); ++i) {
        if (_clusters[i].confidence >= threshold) {
            if (i != n) {
                _clusters[n].swap(_clusters[i]);
            }
            ++n;
        }
    }
    unsigned int extend_size = 0;
    for (unsigned int i = n; i < _clusters.size(); ++i) {
        extend_size += _clusters[i].size();
    }
    rhs.points->reserve(rhs.points->size() + extend_size);
    for (unsigned int i = n; i < _clusters.size(); ++i) {
        for (unsigned int j = 0; j < _clusters[i].size(); ++j) {
            rhs.points->push_back(_clusters[i].points->at(j));
        }
    }
    AINFO << "filter " << _clusters.size() - n << " low confidence clusters of " 
        << _clusters.size();
    _clusters.resize(n);
}

void SppClusterList::split_into_another_cluster(
        SppClusterList& rhs, unsigned int this_id) {
    if (this_id >= _clusters.size()) {
        return;
    }
    rhs._clusters.push_back(SppCluster());
    _clusters[this_id].swap(rhs._clusters.back());
    if (this_id != _clusters.size() - 1) {
        _clusters[this_id].swap(_clusters.back());
    }
    _clusters.resize(_clusters.size() - 1);
}

void SppClusterList::erase_cluster(unsigned int id) {
    if (id >= _clusters.size()) {
        return;
    }
    if (id < _clusters.size() - 1) {
        _clusters[id].swap(_clusters.back());
    }
    _clusters.resize(_clusters.size() - 1);
}

}  // namespace perception
}  // namespace apollo

