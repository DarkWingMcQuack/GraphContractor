#pragma once

#include <GraphEssentials.hpp>
#include <queue>
#include <vector>

namespace pathfinding {

struct QueueElem
{
    datastructure::Distance cost_;
    datastructure::NodeId current_position_;

    auto operator<(const QueueElem& other) const
        -> bool
    {
        return cost_ > other.cost_;
    }
};

// constexpr static inline auto QueueElemCmp =
//     [](auto&& lhs, auto&& rhs) constexpr
// {
//     return lhs.cost_ > rhs.cost_;
// };

using MinHeap = std::priority_queue<QueueElem,
                                    std::vector<QueueElem>>;
// decltype(QueueElemCmp)>;
} // namespace pathfinding
