#pragma once

#include <GraphEssentials.hpp>
#include <queue>
#include <vector>

namespace pathfinding {

struct QueueElem
{
    datastructure::EdgeCost cost_;
    datastructure::NodeId current_position_;
};

constexpr static inline auto QueueElemCmp =
    [](auto&& lhs, auto&& rhs) {
        return lhs.cost_ > rhs.cost_;
    };

using MinHeap = std::priority_queue<QueueElem,
                                    std::vector<QueueElem>,
                                    decltype(QueueElemCmp)>;
} // namespace pathfinding
