#pragma once
#include <Graph.hpp>
#include <queue>

namespace pathfinding {



class MultiTargetDijkstra
{
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
