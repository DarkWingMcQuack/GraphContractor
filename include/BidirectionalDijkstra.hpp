#pragma once
#include <Graph.hpp>
#include <queue>

namespace pathfinding {



class BidirectionalDijkstra
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
    BidirectionalDijkstra(const datastructure::Graph& graph);

    auto shortestDistanceFromTo(const datastructure::NodeId& source,
                                const datastructure::NodeId& target)
        -> datastructure::EdgeCost;

    auto cleanup()
        -> void;

private:
    auto fillForwardInfo(const datastructure::NodeId& source)
        -> void;
    auto fillBackwardInfo(const datastructure::NodeId& target)
        -> void;

    auto findShortestPathInSettledNodes()
        -> datastructure::EdgeCost;

private:
    const datastructure::Graph& graph_;
    std::vector<datastructure::NodeId> forward_touched_nodes_;
    std::vector<datastructure::NodeId> forward_settled_nodes_;
    std::vector<datastructure::EdgeCost> forward_shortest_distances_;
    std::vector<datastructure::NodeId> backward_touched_nodes_;
    std::vector<datastructure::NodeId> backward_settled_nodes_;
    std::vector<datastructure::EdgeCost> backward_shortest_distances_;
};

} // namespace pathfinding
