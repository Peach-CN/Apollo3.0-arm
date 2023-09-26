#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_2d.h"
#ifdef USE_AVX
#include <immintrin.h>
#include <stdlib.h>
#endif

namespace apollo {
namespace perception {

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
        double max_ref_dist)
{
    unsigned int nr_seeds = seeds.size();
    int nr_matched_node = 0;
    int k = 0;
    int x = 0;
    int y = 0;
    int y_index = 0;
    unsigned int best = 0;
    float best_dist = 0;
    float dist = 0;
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
#ifdef USE_AVX
    max_dist_thre *= max_dist_thre;
    int l = 0;
    int end = 0;
    float* array_f = idl::i_alloc_aligned<float>(8, 5);//32bytes align
    float* seeds_xs = idl::i_alloc_aligned<float>(seeds.size() + 8, 5);//32bytes align
    float* seeds_ys = idl::i_alloc_aligned<float>(seeds.size() + 8, 5);//32bytes align
    float* array_max_value = idl::i_alloc_aligned<float>(8, 5);//32bytes align
    float* array_max_indices = idl::i_alloc_aligned<float>(8, 5);//32bytes align
    for (unsigned int i = 0; i < seeds.size(); ++i) {
        seeds_xs[i] = seeds[i].first;
        seeds_ys[i] = seeds[i].second;
    }
    float *seeds_xs_ptr = NULL;
    float *seeds_ys_ptr = NULL;
    __m256 avx_xs;
    __m256 avx_ys;
    __m256 avx_cx;
    __m256 avx_cy;
    __m256 avx_max_value;
    __m256 avx_max_indices;
    __m256 avx_indices;
    __m256 avx_cmp_mask;
    __m256 avx_indices_inc = _mm256_set1_ps(8.f);
#endif

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
#ifndef USE_AVX
            best = seeds_index[y_index].first;
            best_dist = pair_to_pair_distance(seeds[seeds_index[y_index].first], center_map[i][j]);
            dist = 0;
            for (k = seeds_index[y_index].first + 1; 
                    k <= seeds_index[y_index].second; ++k)
            {
                dist = pair_to_pair_distance(seeds[k], center_map[i][j]);

                if (dist < best_dist)
                {
                    best = k;
                    best_dist = dist;
                }
            }
#else
            best_dist = 10000.f;
            end = seeds_index[y_index].second - seeds_index[y_index].first + 1;
            end = seeds_index[y_index].first + (end >> 3 << 3) - 1;
            avx_cx = _mm256_set1_ps(center_map[i][j].first);
            avx_cy = _mm256_set1_ps(center_map[i][j].second);
            avx_max_value = _mm256_set1_ps(10000.f);
            avx_max_indices = _mm256_setr_ps(0, 1, 2, 3, 4, 5, 6, 7);
            avx_indices = _mm256_setr_ps(0, 1, 2, 3, 4, 5, 6, 7);
            for (k = seeds_index[y_index].first; k <= end; k += 8) {
                seeds_xs_ptr = seeds_xs + k;
                seeds_ys_ptr = seeds_ys + k;
                if (k & 255) {
                    avx_xs = _mm256_loadu_ps(seeds_xs_ptr);
                    avx_ys = _mm256_loadu_ps(seeds_ys_ptr);
                }
                else {
                    avx_xs = _mm256_load_ps(seeds_xs_ptr);
                    avx_ys = _mm256_load_ps(seeds_ys_ptr);
                }

                avx_xs = _mm256_sub_ps(avx_xs, avx_cx);
                avx_ys = _mm256_sub_ps(avx_ys, avx_cy);
                avx_xs = _mm256_mul_ps(avx_xs, avx_xs);
                avx_ys = _mm256_mul_ps(avx_ys, avx_ys);
                avx_xs = _mm256_add_ps(avx_xs, avx_ys);

                avx_cmp_mask = _mm256_cmp_ps(avx_xs, avx_max_value, _CMP_LT_OQ); 
                avx_max_value = _mm256_blendv_ps(avx_max_value, avx_xs, avx_cmp_mask);
                avx_max_indices = _mm256_blendv_ps(avx_max_indices, avx_indices, avx_cmp_mask);
                avx_indices = _mm256_add_ps(avx_indices, avx_indices_inc);
            }
            if (k != seeds_index[y_index].first) {
                _mm256_store_ps(array_max_value, avx_max_value);
                _mm256_store_ps(array_max_indices, avx_max_indices);
                best_dist = array_max_value[0];
                best = static_cast<unsigned int>(array_max_indices[0]);
                for (l = 1; l < 8; ++l) {
                    if (array_max_value[l] < best_dist) {
                        best_dist = array_max_value[l];
                        best = static_cast<unsigned int>(array_max_indices[l]);
                    }
                }
                best += seeds_index[y_index].first;
            }
            if (k <= seeds_index[y_index].second) {
                if (k & 255) {
                    avx_xs = _mm256_loadu_ps(seeds_xs + k);
                    avx_ys = _mm256_loadu_ps(seeds_ys + k);
                }
                else {
                    avx_xs = _mm256_load_ps(seeds_xs + k);
                    avx_ys = _mm256_load_ps(seeds_ys + k);
                }

                avx_xs = _mm256_sub_ps(avx_xs, avx_cx);
                avx_ys = _mm256_sub_ps(avx_ys, avx_cy);
                avx_xs = _mm256_mul_ps(avx_xs, avx_xs);
                avx_ys = _mm256_mul_ps(avx_ys, avx_ys);
                avx_xs = _mm256_add_ps(avx_xs, avx_ys);
                _mm256_store_ps(array_f, avx_xs);
                for (l = 0; k <= seeds_index[y_index].second; ++k, ++l) {
                    if (array_f[l] < best_dist) {
                        best_dist = array_f[l];
                        best = k;
                    }
                }
            }
#endif
            if ((double)best_dist >= max_dist_thre)
            {
                continue; /*no seed can be matched*/
            }

            x = static_cast<int>(seeds[best].first);
            y = static_cast<int>(seeds[best].second);
            if (ref_map && x >= 0 && y >= 0 && ref_map[y][x] - ref_map[i][j] > max_ref_dist) {
                continue;
            }
            output[best].push_back(static_cast<float>(j), static_cast<float>(i));
            ++nr_matched_node;
        }
    }
#ifdef USE_AVX
    idl::i_free_aligned(array_f);
    idl::i_free_aligned(seeds_xs);
    idl::i_free_aligned(seeds_ys);
    idl::i_free_aligned(array_max_value);
    idl::i_free_aligned(array_max_indices);
#endif

    return nr_matched_node;
}

}  // namespace perception
}  // namespace apollo
