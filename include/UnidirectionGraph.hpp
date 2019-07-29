#pragma once

#include <GraphEssentials.hpp>
#include <cstdint>
#include <span.hpp>
#include <vector>

namespace datastructure {

class UnidirectionGraph
{
public:
    UnidirectionGraph(std::vector<std::pair<NodeId, Edge>> node_edges);

    auto getEdgesOf(const NodeId& node) const
        -> tcb::span<const Edge>;

    auto getNumberOfEdgesOf(const NodeId& node) const
        -> std::int_fast32_t;

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

auto createBackwardGraphpart(const UnidirectionGraph& forward)
    -> UnidirectionGraph;


auto createBackwardGraphpart(const UnidirectionGraph& forward)
    -> UnidirectionGraph;

} // namespace datastructure
