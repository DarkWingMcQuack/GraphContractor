#pragma once

#include <Graph.hpp>
#include <GraphEssentials.hpp>
#include <MultiTargetDijkstra.hpp>
#include <UnidirectionGraph.hpp>
#include <cstdint>
#include <functional>
#include <span.hpp>
#include <unordered_map>
#include <vector>

namespace datastructure {

class GraphContractor
{
public:
    // GraphContractor(std::vector<std::vector<Edge>> node_edges);
    GraphContractor(Graph graph);

    auto contractGraph()
        -> void;

    auto getGraph()
        -> Graph&;

private:
    auto graphFullContracted() const
        -> bool;

    auto numberOfIngoingEdges(NodeId node) const
        -> std::int64_t;

    auto numberOfOutgoingEdges(NodeId node) const
        -> std::int64_t;

    auto contract(NodeId node)
        -> std::pair<std::unordered_map<NodeId, std::vector<Edge>>,
                     int>; //deleted edges - shortcuts

    auto constructIndependentSet() const
        -> std::vector<NodeId>;

    auto getEdgeDegreeSortedNodes() const
        -> std::vector<NodeId>;

    auto getDegreeOf(NodeId node) const
        -> std::int64_t;

    auto getBestContractions(std::vector<NodeId> independent_set)
        -> std::pair<
            std::unordered_map<NodeId, std::vector<Edge>>,
            std::vector<NodeId>>; //contracted nodes

private:
    Graph graph_;
    pathfinding::MultiTargetDijkstra dijkstra_;
    NodeLevel current_level{0};
    std::unordered_map<NodeId, std::vector<Edge>> deleted_edges_;
};

} // namespace datastructure
