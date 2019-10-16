#pragma once

#include "Graph.hpp"
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <cstdint>
#include <functional>
#include <span.hpp>
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

    auto contract(const std::vector<NodeId>& nodes) const
        -> std::vector<Edge>;

    auto contract(NodeId node) const
        -> std::pair<std::vector<std::pair<NodeId, Edge>>, //edges to delete
                     std::vector<std::pair<NodeId, Edge>>>; //edges to add

    auto constructIndependentSet() const
        -> std::vector<NodeId>;

    auto getEdgeDegreeSortedNodes() const
        -> std::vector<NodeId>;

    auto getDegreeOf(NodeId node) const
        -> std::int64_t;

    auto getBestContractions(std::vector<NodeId> independent_set)
        -> std::tuple<
            std::vector<std::pair<NodeId, // source
                                  Edge>>, //edges to delete
            std::vector<std::pair<NodeId, //source
                                  Edge>>,
            std::vector<NodeId>>;

private:
    Graph graph_;
    NodeLevel current_level{0};
    std::int64_t already_contracted{0};
};

} // namespace datastructure
