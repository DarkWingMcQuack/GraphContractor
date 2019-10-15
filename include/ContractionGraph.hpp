#pragma once

#include "Graph.hpp"
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <cstdint>
#include <functional>
#include <span.hpp>
#include <vector>

namespace datastructure {

class ContractionGraph
{
public:
    ContractionGraph(std::vector<std::vector<Edge>> node_edges);

private:
    auto isContracted(NodeId node) const
        -> bool;

    auto areIndependent(NodeId first, NodeId second) const
        -> bool;

    auto numberOfIngoingEdges(NodeId node) const
        -> std::int64_t;

    auto numberOfOutgoingEdges(NodeId node) const
        -> std::int64_t;

    auto contract(const std::vector<NodeId>& nodes) const
        -> std::vector<Edge>;

    auto contract(NodeId node) const
        -> std::pair<std::vector<std::pair<NodeId, Edge>>, //edges to delete
                     std::vector<std::pair<NodeId, Edge>>>; //edges to add

    auto rebuild(std::vector<Edge> new_edges,
                 std::vector<NodeId> contracted_nodes)
        -> void;

    auto constructIndependentSet() const
        -> std::vector<NodeId>;

    auto getEdgeDegreeSortedNodes() const
        -> std::vector<NodeId>;

    auto getDegreeOf(NodeId node) const
        -> std::int64_t;

    auto getBestContractions(std::vector<NodeId> independent_set)
        -> std::pair<
            std::vector<std::pair<NodeId, // source
                                  Edge>>, //edges to delete
            std::vector<std::pair<NodeId, //source
                                  Edge>>>; //edges to add

private:
    std::vector<bool> contracted_nodes;
    Graph graph_;
    NodeLevel current_level{0};
};

} // namespace datastructure
