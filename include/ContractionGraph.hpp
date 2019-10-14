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

    auto getForwardEdgesOf(NodeId node) const
        -> tcb::span<const Edge>;

    auto getBackwardEdgesOf(NodeId node) const
        -> tcb::span<const Edge>;

private:
    auto getForwardEdgesOf(NodeId node)
        -> tcb::span<Edge>;

    auto getBackwardEdgesOf(NodeId node)
        -> tcb::span<Edge>;

    auto isContracted(NodeId node) const
        -> bool;

    auto getForwardReachableNodes(NodeId node) const
        -> std::vector<NodeId>;

    auto getBackwardReachableNodes(NodeId node) const
        -> std::vector<NodeId>;

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

    auto getNSmallestEdgeDifferenceContractions(std::vector<NodeId> independent_set,
                                                std::int64_t number_of_maximal_contractions)
        -> std::vector<
            std::pair<std::vector<std::pair<NodeId, // source
                                            Edge>>, //edges to delete
                      std::vector<std::pair<NodeId, //source
                                            Edge>>>>; //edges to add

private:
    /*
	* offset_array points into edge_ids
	* edge_ids then points into edges to get the properties
	*/
    std::vector<NodeOffset> forward_offset_array_;
    std::vector<Edge> forward_edges_;
    std::vector<NodeOffset> backward_offset_array_;
    std::vector<Edge> backward_edges_;
    std::vector<std::int64_t> node_level;
    std::vector<bool> contracted_nodes;
    std::vector<std::int64_t> number_of_ingoing_edges;

    //needed to perform dijkstras
    //offset arrays will be moved into the graph before performin dijkstras
    //and will then be moved out of it again after the dijkstras are finished
    std::optional<Graph> graph_opt_;
};

} // namespace datastructure
