#pragma once
#include <Graph.hpp>
#include <queue>

namespace pathfinding {



class CHDijkstra
{
public:
    CHDijkstra(const datastructure::Graph& graph);

    auto shortestDistanceFromTo(const datastructure::NodeId& source,
                                const datastructure::NodeId& target)
        -> datastructure::Distance;


private:
    auto fillForwardInfo(const datastructure::NodeId& source)
        -> void;
    auto fillBackwardInfo(const datastructure::NodeId& target)
        -> void;
    auto cleanup()
        -> void;

    auto findShortestPathInSettledNodes()
        -> datastructure::Distance;

private:
    const datastructure::Graph& graph_;
    std::vector<datastructure::NodeId> forward_touched_nodes_;
    std::vector<datastructure::NodeId> forward_settled_nodes_;
    std::vector<datastructure::Distance> forward_shortest_distances_;
    std::vector<datastructure::NodeId> backward_touched_nodes_;
    std::vector<datastructure::NodeId> backward_settled_nodes_;
    std::vector<datastructure::Distance> backward_shortest_distances_;
};

} // namespace pathfinding
