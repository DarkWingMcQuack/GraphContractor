#pragma once
#include <DijkstraQueue.hpp>
#include <Graph.hpp>

namespace pathfinding {



class MultiTargetDijkstra
{
public:
    MultiTargetDijkstra(const datastructure::Graph& graph);

    auto shortestDistanceFromTo(const datastructure::NodeId& source,
                                const std::vector<datastructure::NodeId>& targets)
        -> std::vector<datastructure::EdgeCost>;

    auto shortestDistanceFromTo(const datastructure::NodeId& source,
                                const datastructure::NodeId& target)
        -> datastructure::EdgeCost;

    auto cleanup()
        -> void;

private:
    const datastructure::Graph& graph_;
    MinHeap queue_;
    std::optional<datastructure::NodeId> last_source_;
    std::vector<datastructure::NodeId> touched_nodes_;
    std::vector<bool> settled_;
    std::vector<datastructure::EdgeCost> shortest_distances_;
};

} // namespace pathfinding
