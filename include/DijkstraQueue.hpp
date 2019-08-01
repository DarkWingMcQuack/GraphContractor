#pragma once

#include <GraphEssentials.hpp>
#include <queue>
#include <vector>

namespace pathfinding {

struct QueueElem
{
    datastructure::Distance cost_;
    datastructure::NodeId current_position_;

    QueueElem(datastructure::Distance cost,
              datastructure::NodeId current_pos)
        : cost_(cost), current_position_(current_pos) {}

    auto operator<(const QueueElem& other) const
        -> bool
    {
        return cost_ > other.cost_;
    }
};

using MinHeap = std::priority_queue<QueueElem,
                                    std::vector<QueueElem>>;
} // namespace pathfinding
