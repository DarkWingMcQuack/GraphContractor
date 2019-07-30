#pragma once
#include <Graph.hpp>
#include <queue>

namespace pathfinding {



class MultiTargetDijkstra
{
public:
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

public:
    MultiTargetDijkstra(const datastructure::Graph& graph);

    auto shortestDistanceFromTo(const datastructure::NodeId& source,
                                const std::vector<datastructure::NodeId>& targets)
        -> std::vector<datastructure::EdgeCost>;

    auto cleanup()
        -> void;

private:
    const datastructure::Graph& graph_;
    std::vector<datastructure::NodeId> touched_nodes_;
    std::vector<datastructure::NodeId> settled_nodes_;
    std::vector<datastructure::EdgeCost> shortest_distances_;
};

} // namespace pathfinding
