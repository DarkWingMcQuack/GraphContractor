#pragma once

#include <GraphEssentials.hpp>
#include <cstdint>
#include <span.hpp>
#include <vector>

namespace datastructure {

class UnidirectionGraph
{
public:
    UnidirectionGraph(std::vector<std::vector<Edge>> node_edges);

    UnidirectionGraph(std::vector<std::vector<Edge>> node_edges,
                      const std::vector<NodeLevel>& node_levels);

    auto getEdgesOf(const NodeId& node) const
        -> tcb::span<const Edge>;

private:
    auto getEdgesOf(const NodeId& node)
        -> tcb::span<Edge>;

private:
    /*
     * offset_array points into edge_ids
     * edge_ids then points into edges to get the properties
     */
    std::vector<NodeOffset> offset_array_;
    std::vector<Edge> edges_;
};

} // namespace datastructure
