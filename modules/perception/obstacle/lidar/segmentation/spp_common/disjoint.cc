#include "modules/perception/obstacle/lidar/segmentation/spp_common/disjoint.h"

namespace apollo {
namespace perception {

// disjoint forests using union-by-rank and path compression (sort of).

Universe::Universe(int elements) {
    _elts.resize(elements);
    _num = elements;
    for (int i = 0; i < elements; ++i) {
        _elts[i].rank = 0;
        _elts[i].size = 1;
        _elts[i].p = i;
    }
}

Universe::~Universe() {
    _elts.clear();
}

void Universe::reset(const int elements) {
    _num = elements;
    _elts.resize(elements);
    for (int i = 0; i < elements; ++i) {
        _elts[i].rank = 0;
        _elts[i].size = 1;
        _elts[i].p = i;
    }
}

int Universe::find(int x) {
    int y = x;
    while (y != _elts[y].p) {
        y = _elts[y].p;
    }
    while (true) {
        const int z = _elts[x].p;
        if (z == x) {
            break;
        }
        _elts[x].p = y;
        x = z;
    }
    return y;
}

void Universe::join(int x, int y) {
    if (_elts[x].rank > _elts[y].rank) {
        _elts[y].p = x;
        _elts[x].size += _elts[y].size;
    } else {
        _elts[x].p = y;
        _elts[y].size += _elts[x].size;
        if (_elts[x].rank == _elts[y].rank) {
            ++_elts[y].rank;
        }
    }
    --_num;
}

}  // namespace perception
}  // namespace apollo

