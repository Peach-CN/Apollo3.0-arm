#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_2D_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_2D_H
#include "modules/common/log.h"

#include <common-i-lib/pc/i_segm.h>
#include <common-i-lib/core/i_constant.h>
#include <queue>
#include <map>

namespace apollo {
namespace perception {

template<typename T>
class SppOccDetector
{
    public:
        SppOccDetector();
        ~SppOccDetector();
        bool init(int width, int height, int scale);
        bool initialized() const { return _initialized; }

        int detect(const float * const *prob_map,
                const float *offset_map,
                unsigned short *labels,
                unsigned char *canvas,
                float prob_cutoff_thre_contribute,
                float prob_cutoff_thre_seed,
                float prob_cutoff_thre_obj,
                float prob_cutoff_thre_diff,
                double cc_to_center_dist_thre,
                float center_to_center_dist_thre,
                unsigned int occ_neighbor_thre,
                bool verbose = true);

        const unsigned int* get_labels(unsigned int scale = 0) const
        {
            if (!_initialized)
            {
                return NULL;
            }
            return (_labels->const_data(scale));
        }

        const float* get_probabilities(unsigned int scale = 0) const
        {
            if (!_initialized)
            {
                return NULL;
            }
            return (_probs->const_data(scale));
        }

        const unsigned char* get_masks(unsigned int scale = 0) const
        {
            if (!_initialized)
            {
                return NULL;
            }
            return (_masks->const_data(scale));
        }

        int nr_scale() const 
        { 
            if (!_initialized)
            {
                return 0;
            }
            return (int)(_occs->size());
        };

        int width() const
        {
            if (!_initialized)
            {
                return 0;
            }
            return _labels->width();
        }

        int height() const
        {
            if (!_initialized)
            {
                return 0;
            }
            return _labels->height();
        }

        int width(unsigned int scale) const
        {
            if (!_initialized)
            {
                return 0;
            }
            return _labels->width(scale);
        }

        int height(unsigned int scale) const
        {
            if (!_initialized)
            {
                return 0;
            }
            return _labels->height(scale);
        }

    private:
        void cleanup();

    private:
        idl::MultiScaleLumPyramid<float>         *_probs;  /*masks*/
        idl::MultiScaleLumPyramid<unsigned char> *_masks;  /*masks*/
        idl::MultiScaleLumPyramid<unsigned int>  *_labels; /*labels*/
        idl::OCCTable<T>                         *_occs;   /*multi-scale cc array*/
        unsigned int  **_workarray_uint;
        unsigned char **_workarray_uchar;
        float **_workarray_flt, **_workarray_flt_up;
        float **_upsample_seed_map;
        idl::Pair<float, float> **_center_map;
        bool _initialized;
};

template <typename T>
bool get_occ_offset_center(const idl::OCC<T>& occ, 
        const float* offset_map, unsigned int width, unsigned int height, idl::Pair<T, T>& result) {
    T x = 0;
    T y = 0;
    const float* offset_ptr = NULL;
    T sum_x = 0;
    T sum_y = 0;
    for (unsigned int i = 0; i < occ.size(); ++i) {
        x = occ.x(i);
        y = occ.y(i);
        offset_ptr = offset_map + (static_cast<int>(y) * width + static_cast<int>(x)) * 2;
        sum_x += x + static_cast<T>(*offset_ptr);
        sum_y += y + static_cast<T>(*(offset_ptr+1));
    }
    if (occ.size() > 0) {
        sum_x /= static_cast<T>(occ.size());
        sum_y /= static_cast<T>(occ.size());
        if (sum_x >= 0 && sum_x < static_cast<T>(width) 
                && sum_y >= 0 && sum_y < static_cast<T>(height)) {
            result.first = sum_x;
            result.second = sum_y;
            return true;
        }
    }
    return false;
}

template <typename T>
inline T pair_to_pair_distance(const idl::Pair<T, T>& lhs, const idl::Pair<T, T>& rhs) {
    T diff_first = lhs.first - rhs.first;
    T diff_second = lhs.second - rhs.second;
    return idl::i_sqrt(diff_first * diff_first + diff_second * diff_second);
}

template <typename T>
void build_search_index(const std::vector<idl::Pair<T, T> >& seeds, 
        std::vector<idl::Pair<int, int> >& index, unsigned int height, double radius) {
    index.clear();
    if (!seeds.size()) {
        return;
    }
    std::vector<idl::Pair<int, int> > index_per_y(height, idl::i_make_pair(-1, -1));
    int i = 0;
    int j = 0;
    int k = 0;
    int nr_seeds = static_cast<int>(seeds.size());
    for (i = 0; i < nr_seeds; ++i) {
        if (seeds[i].second >= 0) {
            break;
        }
    }
    if (i == nr_seeds) {
        return;
    }
    int pre_y = static_cast<int>(seeds[i].second);
    int y = 0;
    index_per_y[pre_y].first = 0;
    for (; i < nr_seeds; ++i) {
        y = static_cast<int>(seeds[i].second);
        if (y >= 0 && y != pre_y) {
            index_per_y[pre_y].second = i - 1;
            index_per_y[y].first = i;
            pre_y = y;
        }
    }
    index_per_y[pre_y].second = nr_seeds - 1;
    index.resize(height, idl::i_make_pair(-1, -1));
    int radius_i = static_cast<int>(idl::i_ceil(radius)) + 1;
    int bi = 0;
    int ei = 0;
    for (i = 0; i < height; ++i) {
        bi = std::max(i - radius_i, 0);
        ei = std::min(i + radius_i, static_cast<int>(height) - 1);
        for (j = bi; j <= ei; ++j) {
            if (index_per_y[j].first >= 0) {
                index[i].first = index_per_y[j].first;
                break;
            }
        }
        for (k = ei; k >= j; --k) {
            if (index_per_y[k].second >= 0) {
                index[i].second = index_per_y[k].second;
                break;
            }
        }
    }
}

template <typename T>
int match_node_and_seeds_with_center_map(const unsigned char* const* mask,
        const idl::Pair<float, float>* const* center_map,
        const float* const* ref_map,
        const std::vector<idl::Pair<T, T> >& seeds,
        const std::vector<idl::Pair<int, int> >& seeds_index,
        idl::OCCArray<T>& output,
        unsigned int width,
        unsigned int height,
        double max_dist_thre,
        double max_ref_dist)
{
    unsigned int nr_seeds = seeds.size();
    int nr_matched_node = 0;
    int x = 0;
    int y = 0;
    int y_index = 0;
    unsigned int best = 0;
    T best_dist = 0;
    T dist = 0;
    /*note that the output vector will be updated incrementally, so unless
      the size is not consistent, it will not be cleared at the beginning of this routine*/
    if (output.size() != nr_seeds)
    {
        output.resize(nr_seeds);
    }

    if (!nr_seeds || !mask)
    {
        return (0);
    }

    for (unsigned int i = 0; i < height; ++i)
    {
        for (unsigned int j = 0; j < width; ++j)
        {
            if (!mask[i][j])
            {
                continue;
            }

            y_index = static_cast<int>(center_map[i][j].second);
            if (y_index < 0 || y_index >= height) { 
                continue;
            }
            if (seeds_index[y_index].first < 0) {
                continue;
            }

            best = seeds_index[y_index].first;
            best_dist = pair_to_pair_distance(seeds[seeds_index[y_index].first], center_map[i][j]);
            dist = 0;
            for (unsigned int k = seeds_index[y_index].first + 1; 
                    k <= seeds_index[y_index].second; ++k)
            {
                dist = pair_to_pair_distance(seeds[k], center_map[i][j]);

                if (dist < best_dist)
                {
                    best = k;
                    best_dist = dist;
                }
            }

            if ((double)best_dist >= max_dist_thre)
            {
                continue; /*no seed can be matched*/
            }

            x = static_cast<int>(seeds[best].first);
            y = static_cast<int>(seeds[best].second);
            if (ref_map && x >= 0 && y >= 0 && ref_map[y][x] - ref_map[i][j] > max_ref_dist) {
                continue;
            }
            output[best].push_back(static_cast<T>(j), static_cast<T>(i));
            ++nr_matched_node;
        }
    }
    return nr_matched_node;
}

template <>
int match_node_and_seeds_with_center_map<float>(const unsigned char* const* mask,
        const idl::Pair<float, float>* const* center_map,
        const float* const* ref_map,
        const std::vector<idl::Pair<float, float> >& seeds,
        const std::vector<idl::Pair<int, int> >& seeds_index,
        idl::OCCArray<float>& output,
        unsigned int width,
        unsigned int height,
        double max_dist_thre,
        double max_ref_dist);

template <typename T>
bool occarray_to_label_image(unsigned short* label, unsigned int width, unsigned height, 
        const idl::OCCArray<T>& occs) {
    if (label == NULL) {
        return false;
    }
    memset(label, 0, sizeof(unsigned short) * width * height);
    unsigned int ix = 0;
    unsigned int iy = 0;
    for (unsigned int i = 0; i < occs.size(); ++i) {
        for (unsigned int j = 0; j < occs[i].size(); ++j) {
            ix = static_cast<unsigned int>(occs[i].x(j));
            iy = static_cast<unsigned int>(occs[i].y(j));
            label[iy*width+ix] = static_cast<unsigned short>(i + 1);
        }
    }
    return true;
}

template <typename T>
struct OCCGraphNode {
    int id;
    unsigned int size;
    idl::Pair<T, T> center; 
    OCCGraphNode(int in_id, unsigned int in_size, idl::Pair<T, T> in_center) :
        id(in_id), size(in_size), center(in_center) {
    }
    void clear() {
        id = -1;
        size = 0;
    }
    void merge(const OCCGraphNode& rhs) {
        unsigned merge_size = size + rhs.size;
        center.first = (center.first * size + rhs.center.first * rhs.size) / merge_size;
        center.second = (center.second * size + rhs.center.second * rhs.size) / merge_size;
        size = merge_size;
    }
};

template <typename T>
struct OCCGraphEdge {
    int v1;
    int v2;
    T distance;
    unsigned int neighbor_count;
    OCCGraphEdge() {
        v1 = v2 = 0;
        distance = idl::Constant<T>::MAX_VAL();
        neighbor_count = 0;
    }
    OCCGraphEdge(int in_v1, int in_v2, T in_distance, unsigned int in_neighbor_count) :
        v1(in_v1), v2(in_v2), distance(in_distance), neighbor_count(in_neighbor_count) {
    }
    void clear() {
        v1 = v2 = -1;
        neighbor_count = 0;
        distance = idl::Constant<T>::MAX_VAL();
    }
    void sort_ids() {
        if (v2 > v1) {
            std::swap(v1, v2);
        }
    }
    bool operator > (const OCCGraphEdge<T>& rhs) const {
        return distance > rhs.distance;
    }
    bool operator < (const OCCGraphEdge<T>& rhs) const {
        return distance < rhs.distance;
    }
};

template <typename T>
void occ_graph_edge_vector_unique(std::vector<OCCGraphEdge<T> >& edges) {
    std::map<std::pair<T, T>, OCCGraphEdge<T>> edge_map;
    typename std::map<std::pair<T, T>, OCCGraphEdge<T>>::iterator iter;
    std::pair<T, T> key;
    for (unsigned int i = 0; i < edges.size(); ++i) {
        key.first = edges[i].v1;
        key.second = edges[i].v2;
        iter = edge_map.find(key);
        if (iter == edge_map.end()) {
            edge_map.insert(std::make_pair(key, edges[i]));
        }
        else {
            iter->second.neighbor_count += edges[i].neighbor_count;
        }
    }
    edges.clear();
    for (iter = edge_map.begin(); iter != edge_map.end(); ++iter) {
        edges.push_back(iter->second);
    }
}

template <typename T>
void build_occ_graph(const idl::OCCArray<T>& occs, const std::vector<idl::Pair<T, T> >& seeds, 
        const std::vector<idl::Pair<int, int> >& index, 
        unsigned short* label, unsigned int width, unsigned int height,
        std::vector<OCCGraphNode<T> >& nodes, std::vector<OCCGraphEdge<T> >& edges,
        T dist_thred, unsigned int neighbor_thred) {
    nodes.clear();
    edges.clear();
    nodes.reserve(2000);
    occarray_to_label_image(label, width, height, occs);
    unsigned int nr_occs = occs.size();
    std::vector<std::vector<unsigned int> > neighbors(nr_occs, 
            std::vector<unsigned int>(nr_occs, 0));
    const unsigned short* ptr = label + width + 1;
    unsigned short label_current = 0;
    unsigned short label_neighbor = 0;
    for (unsigned int y = 1; y < height; ++y, ++ptr) {
        for (unsigned int x = 1; x < width; ++x, ++ptr) {
            label_current = *ptr;
            if (label_current == 0) {
                continue;
            }
            --label_current;
            label_neighbor = *(ptr - 1);
            if (label_neighbor > 0) {
                --label_neighbor;
                if (label_current < label_neighbor) {
                    ++neighbors[label_current][label_neighbor];
                }
                else if (label_current > label_neighbor) {
                    ++neighbors[label_neighbor][label_current];
                }
            }
            label_neighbor = *(ptr - width);
            if (label_neighbor > 0) {
                --label_neighbor;
                if (label_current < label_neighbor) {
                    ++neighbors[label_current][label_neighbor];
                }
                else if (label_current > label_neighbor) {
                    ++neighbors[label_neighbor][label_current];
                }
            }
        }
    }
    for (unsigned int i = 0; i < seeds.size(); ++i) {
        nodes.push_back(OCCGraphNode<T>(i, occs[i].size(), seeds[i]));
    }
    T dist = 0;
    T dist_thred2 = dist_thred * 1.5;
    T neighbor_thred_half = static_cast<T>(neighbor_thred) / 2;
    int index_y = 0;
    for (int i = 0; i < static_cast<int>(seeds.size()); ++i) {
        index_y = static_cast<int>(seeds[i].second);
        if (index[index_y].first < 0) {
            continue;
        }
        for (int j = std::max(i + 1, index[index_y].first); 
                j <= index[index_y].second; ++j) {
            dist =  pair_to_pair_distance(seeds[i], seeds[j]);
            if (dist < dist_thred2 && neighbors[i][j] > neighbor_thred_half) {
                edges.push_back(OCCGraphEdge<T>(i, j, dist, neighbors[i][j]));
            }
        }
    }
    AINFO << "OCCGraph: #node " << nodes.size() << " #edge " << edges.size();
}

template <typename T>
int merge_seeds_and_occs(idl::OCCArray<T>& occs, std::vector<idl::Pair<T, T> >& seeds, 
        const std::vector<idl::Pair<int, int> >& index,
        unsigned short* label, unsigned int width, unsigned int height,
        T dist_thred, unsigned int neighbor_thred) {
    std::vector<OCCGraphNode<T> > nodes;
    std::vector<OCCGraphEdge<T> > edges;
    build_occ_graph(occs, seeds, index, label, width, height, 
            nodes, edges, dist_thred, neighbor_thred);
    int nr_nodes = nodes.size();
    while (edges.size()) {
        unsigned int index 
            = std::min_element(edges.begin(), edges.end()) - edges.begin();
        OCCGraphEdge<T> edge = edges[index];
        if (edge.distance >= dist_thred) {
            break;
        } 
        // remove this edge
        edges[index] = edges.back();
        edges.resize(edges.size() - 1);
        // merge
        if (edge.neighbor_count > neighbor_thred) {
            const int& v1 = edge.v1;
            const int& v2 = edge.v2;
            // update node
            nodes[v1].merge(nodes[v2]);
            nodes[v2].clear();
            occs[v1] += occs[v2];
            occs[v2].clear();
            seeds[v1] = nodes[v1].center;
            seeds[v2] = idl::i_make_pair(static_cast<T>(-1000), static_cast<T>(-1000));
            --nr_nodes;
            // update edge stage 1: find e2i and add or insert to e1i (update neighbor)
            for (unsigned int i = 0; i < edges.size(); ++i) {
                if (edges[i].v1 == v2) {
                    edges[i].v1 = v1;
                }
                else if (edges[i].v2 == v2) {
                    edges[i].v2 = v1;
                }
                edges[i].sort_ids();
            }
            occ_graph_edge_vector_unique(edges);
            // update edge stage 2; update distance of e1i 
            for (unsigned int i = 0; i < edges.size(); ++i) {
                if (edges[i].v1 == v1 || edges[i].v2 == v1) {
                    edges[i].distance = pair_to_pair_distance(nodes[edges[i].v1].center,
                            nodes[edges[i].v2].center);
                }
            }
        }
    }
    AINFO << "#nodes " << nr_nodes;
    return nr_nodes;
}

template <typename T>
SppOccDetector<T>::SppOccDetector() :
    _initialized(false), _probs(NULL), _masks(NULL), 
    _labels(NULL), _occs(NULL), 
    _workarray_uint(NULL),
    _workarray_uchar(NULL),
    _workarray_flt(NULL),
    _workarray_flt_up(NULL),
    _upsample_seed_map(NULL),
    _center_map(NULL){}

template <typename T>
SppOccDetector<T>::~SppOccDetector()
{
    cleanup();
}

template <typename T>
void SppOccDetector<T>::cleanup()
{
    if (_probs)
    {
        delete _probs;
    }
    if (_masks)
    {
        delete _masks;
    }
    if (_labels)
    {
        delete _labels;
    }
    if (_occs)
    {
        delete _occs;
    }
    if (_workarray_uint)
    {
        idl::i_free2<unsigned int>(_workarray_uint);
    }
    if (_workarray_uchar)
    {
        idl::i_free2<unsigned char>(_workarray_uchar);
    }
    if (_workarray_flt)
    {
        idl::i_free2<float>(_workarray_flt);
    }
    if (_workarray_flt_up)
    {
        idl::i_free2<float>(_workarray_flt_up);
    }
    if (_upsample_seed_map)
    {
        idl::i_free2<float>(_upsample_seed_map);
    }
    if (_center_map) 
    {
        idl::i_free2<idl::Pair<float, float> >(_center_map);
    }
    _initialized = false;
}

template <typename T>
bool SppOccDetector<T>::init(int width, int height, int scale)
{
    if (_initialized)
    {
        cleanup();
    }

    _probs = new idl::MultiScaleLumPyramid<float>();
    _masks = new idl::MultiScaleLumPyramid<unsigned char>();
    _labels = new idl::MultiScaleLumPyramid<unsigned int>();
    _occs   = new idl::OCCTable<T>();
    _workarray_uint = idl::i_alloc2<unsigned int>(height, width);
    _workarray_uchar = idl::i_alloc2<unsigned char>(height, width);
    _workarray_flt = idl::i_alloc2<float>(height, width);
    _workarray_flt_up = idl::i_alloc2<float>((height << 1), (width << 1));
    _upsample_seed_map = idl::i_alloc2<float>((height << 1), (width << 1));
    _center_map = idl::i_alloc2<idl::Pair<float, float> >(height, width);

    if (!_probs || !_masks || !_labels || !_occs || !_workarray_uint || !_workarray_uchar || 
            !_workarray_flt || !_workarray_flt_up || !_upsample_seed_map || !_center_map)
    {
        cleanup();
        return false;
    }

    _probs->init(width, height, scale);
    _masks->init(width, height, scale);
    _labels->init(width, height, scale);
    _occs->resize(scale);

    if (!_probs->initialized() || !_masks->initialized() || !_labels->initialized())
    {
        cleanup();
        return false;
    }

    _initialized = true;
    return (true);
}

template <typename T>
int SppOccDetector<T>::detect(const float * const *prob_map,
        const float *offset_map,
        unsigned short *labels,
        unsigned char *canvas,
        float prob_cutoff_thre_contribute,
        float prob_cutoff_thre_seed,
        float prob_cutoff_thre_obj,
        float prob_cutoff_thre_diff,
        double cc_to_center_dist_thre,
        float center_to_center_dist_thre,
        unsigned int occ_neighbor_thre,
        bool verbose)
{
    if (!offset_map || !prob_map)
    {
        return (-2);
    }

    if (!_initialized)
    {
        return (-1);
    }

    const int width = _labels->width();
    const int height = _labels->height();
    const int width_up = (_labels->width() << 1);
    const int height_up = (_labels->height() << 1);
    const int size = (width * height);
    const int size_up = (width_up * height_up);
    unsigned int i, j, iter;
    int cx, cy, nr_cc = 0;
    float prob, prob_half, prob_quarter;
    idl::OCCArray<T> result;

    /*accumulate prob*/
    std::vector<idl::Pair<float, float> > seeds;
    seeds.reserve(3000);/*pre-allocate seeds memory*/

    const float *cptr_prob = prob_map[0];
    const float *cptr_offset = offset_map;
    idl::Pair<float, float>* cptr_center = _center_map[0];

#if 0
    //clear the upsampled seed buffer
    memset((void*)_upsample_seed_map[0], 0, size_up*sizeof(float));

    for (i = 0; i < (unsigned int)height; ++i)
    {
        for (j = 0; j < (unsigned int)width; ++j, ++cptr_prob, cptr_offset += 2, ++cptr_center)
        {
            prob = cptr_prob[0];

            if (prob <= prob_cutoff_thre_contribute)
            {
                continue; /*small prob does not contribute*/
            }

            cptr_center->second = (float)i + cptr_offset[0];
            cptr_center->first = (float)j + cptr_offset[1];
            cy = idl::i_round(cptr_center->second * 2.0f); //upsample
            cx = idl::i_round(cptr_center->first * 2.0f); //upsample

            int cym1 = cy - 1;
            int cyp1 = cy + 1;
            int cxm1 = cx - 1;
            int cxp1 = cx + 1;

            if (cx <= 0 || cy <= 0 || cx >= (width_up-1) || cy >= (height_up-1))
            {
                continue;
            }

            /*assign probs*/
            prob_half = prob * 0.5f;
            prob_quarter = prob_half * 0.5f;

            _upsample_seed_map[cym1][cxm1] += prob_quarter;
            _upsample_seed_map[cym1][cx] += prob_half;
            _upsample_seed_map[cym1][cxp1] += prob_quarter;

            _upsample_seed_map[cy][cxm1] += prob_half;
            _upsample_seed_map[cy][cx] += prob;
            _upsample_seed_map[cy][cxp1] += prob_half;

            _upsample_seed_map[cyp1][cxm1] += prob_quarter;
            _upsample_seed_map[cyp1][cx] += prob_half;
            _upsample_seed_map[cyp1][cxp1] += prob_quarter;
        }
    }

    idl::i_dense_nms_squared_5x5_rot_f(_upsample_seed_map,//_probs->const_data2(0),
            _workarray_flt_up,
            width_up,
            height_up,
            2,
            2,
            width_up - 2,
            height_up - 2,
            prob_cutoff_thre_seed);

    for (i = 0; i < (unsigned int)height_up; ++i)
    {
        for (j = 0; j < (unsigned int)width_up; ++j)
        {
            if (_workarray_flt_up[i][j] > 0.f) /*valid seed with high confidence*/
            {
                seeds.push_back(idl::i_make_pair<float, float>((float)j * 0.5f, (float)i * 0.5f));
            }
        }
    }
#else
    //clear the seed buffer
    memset((void*)_probs->data(0), 0, size * sizeof(float));
    float** ptr_prob = _probs->data2();
    float cxmx, cymy, value;
    float prob_xp1mcx, prob_cxmx;

    for (i = 0; i < (unsigned int)height; ++i) {
        for (j = 0; j < (unsigned int)width; ++j, cptr_offset += 2,
                ++cptr_prob, ++cptr_center) {

            prob = cptr_prob[0];

            if (prob <= prob_cutoff_thre_contribute) {
                continue; /*small prob does not contribute*/
            }

            cptr_center->second = (float)i + cptr_offset[0];
            cptr_center->first = (float)j + cptr_offset[1];

            cy = (int)(cptr_center->second);
            cx = (int)(cptr_center->first);

            if (cx < 0 || cy < 0 || cx >= width - 1 || cy >= height - 1) {
                continue;
            }
            cxmx = cptr_center->first - (float)cx;
            cymy = cptr_center->second - (float)cy;

            prob_cxmx = prob * cxmx;                                                                
            prob_xp1mcx = prob - prob_cxmx;                                                                                                                                                              

            value = prob_cxmx * cymy;                                                               
            ptr_prob[cy][cx+1] += prob_cxmx - value;                                                
            ptr_prob[cy+1][cx+1] += value;                                                                                                                                                              

            value = prob_xp1mcx * cymy;                                                             
            ptr_prob[cy][cx] += prob_xp1mcx - value;                                                
            ptr_prob[cy+1][cx] += value;  
        }
    }

    idl::i_dense_nms_squared_3x3_f(_probs->const_data2(0),
            _workarray_flt,
            width,
            height,
            2,
            2,
            width - 2,
            height - 2,
            prob_cutoff_thre_seed);

    for (i = 0; i < (unsigned int)height; ++i) {
        for (j = 0; j < (unsigned int)width; ++j) {
            if (_workarray_flt[i][j] > 0.f) {
                seeds.push_back(idl::i_make_pair<float, float>((float)j, (float)i));
            }
        }
    }
#endif

    if (verbose)
    {
        AINFO << "# of Seeds " << seeds.size();
    }

    /*clear buffer:*/
    memset((void*)_masks->data(0), 0, size*sizeof(unsigned char));
    memset((void*)_workarray_uchar[0], 0, size*sizeof(unsigned char));

    float prob_cutoff_thre_obj_low 
        = std::max(0.33f, prob_cutoff_thre_obj - prob_cutoff_thre_diff);
    for (i = 0; i < (unsigned int)size; ++i)
    {
        prob = prob_map[0][i];
        if (prob > prob_cutoff_thre_obj)
        {
            _workarray_uchar[0][i] = 1;
        }
        else if (prob > prob_cutoff_thre_obj_low) {
            _masks->data(0)[i] = 1;
        }
    }
    std::vector<idl::Pair<int, int> > index;
    build_search_index(seeds, index, height, cc_to_center_dist_thre);
    nr_cc = match_node_and_seeds_with_center_map(_workarray_uchar, _center_map, NULL, seeds, 
            index, result, width, height, cc_to_center_dist_thre, 0.0);
    nr_cc = merge_seeds_and_occs(result, seeds, index, labels, width, height, 
            (T)center_to_center_dist_thre, occ_neighbor_thre);
    for (i = 0; i < seeds.size(); ++i) {
        cy = static_cast<int>(seeds[i].second);
        cx = static_cast<int>(seeds[i].first);
        if (cx < 0 || cy < 0) {
            continue;
        }
        if (_workarray_uchar[cy][cx]) {
            seeds[i].first = static_cast<T>(-1000);
            seeds[i].second = static_cast<T>(-1000);
        }
    }
    nr_cc = match_node_and_seeds_with_center_map(_masks->data2(0), _center_map, prob_map, 
            seeds, index, result, width, height, 
            cc_to_center_dist_thre * 0.5, prob_cutoff_thre_diff); 
    if (verbose) {
        AINFO << "add " << nr_cc << " low support points..";
    }
    /*remove empty ccs (seeds that with no match)*/
    (*_occs)[0].clear();

    for (i = 0; i < result.size(); ++i)
    {
        if (!result[i].empty())
        {
            (*_occs)[0].push_back(result[i]);
        }
    }

    if (verbose)
    {
        AINFO << "scale: " << 0 << ", # of cc (processed): " << (*_occs)[0].size();
    }

    /*save results*/
    if (labels != NULL) {
        occarray_to_label_image(labels, width, height, (*_occs)[0]);
    }

    return (int)(((*_occs)[0]).size());
}

}  // namespace perception
}  // namespace apollo

#endif
