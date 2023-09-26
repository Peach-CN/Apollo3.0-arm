/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef ADU_PERCEPTION_LIB_GRAPH_FELZENSZWALB_DISJOINT_H
#define ADU_PERCEPTION_LIB_GRAPH_FELZENSZWALB_DISJOINT_H

#include <vector>

namespace apollo {
namespace perception {

// disjoint forests using union-by-rank and path compression (sort of).

typedef struct {
    int rank;
    int p;
    int size;
} uni_elt;

class Universe {
public:
    Universe() {
        _num = 0;
    }
    Universe(int elements);
    ~Universe();
    void reset(const int elements);
    int find(int x);
    void join(int x, int y);
    int size(int x) const {
        return _elts[x].size;
    }
    int num_sets() const {
        return _num;
    }
private:
    std::vector<uni_elt> _elts;
    int _num;
};

}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_LIB_GRAPH_FELZENSZWALB_DISJOINT_H
