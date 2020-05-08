#pragma

#include <queue>
#include "Edge.hpp"

class Front
{
    std::queue<Edge*> edges;
    Front();
    ~Front();
    Edge get_active_edge() {

    };
    void add_edge();
    void set_inactive();
};
