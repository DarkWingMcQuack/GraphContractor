#pragma once

#include <cstdint>
#include <span.hpp>
#include <vector>

namespace datastructure {

using NodeId = std::uint_fast32_t;
using NodeOffset = std::uint_fast32_t;
using EdgeCost = std::uint_fast32_t;
using NodeLevel = std::uint_fast32_t;

class Edge
{
public:
    Edge(EdgeCost cost,
         NodeId destination);

    auto getCost() const
        -> EdgeCost;

    auto getDestination() const
        -> NodeId;

private:
    EdgeCost cost_;
    NodeId destination_;
};

class GraphPart
{
public:
    auto getEdgesOf(const NodeId& node) const
        -> tcb::span<Edge>;

    auto getOffsetArray() const
        -> const std::vector<NodeOffset>&;

    auto getEdges() const
        -> const std::vector<Edge>&;

private:
    /*
     * offset_array points into edge_ids
     * edge_ids then points into edges to get the properties
     */
    std::vector<NodeOffset> offset_array_;
    std::vector<Edge> edges_;
};

class Graph
{
public:
    auto getForwardEdgesOf(const NodeId& node) const
        -> tcb::span<NodeId>;

    auto getBackwardEdgesOf(const NodeId& node) const
        -> tcb::span<NodeId>;

    auto getLevelOf(const NodeId& node) const
        -> NodeLevel;

private:
    GraphPart forward_graph_;
    GraphPart backward_graph_;
    std::vector<NodeLevel> node_levels_;
};

} // namespace datastructure
