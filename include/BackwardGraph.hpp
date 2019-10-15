#pragma once

#include <GraphEssentials.hpp>
#include <cstdint>
#include <span.hpp>
#include <vector>

namespace datastructure {

class BackwardGraph
{
public:
    BackwardGraph(std::vector<std::vector<Edge>> node_edges);

    BackwardGraph(std::vector<std::vector<Edge>> node_edges,
                  const std::vector<NodeLevel>& node_levels);

    auto getEdgesOf(const NodeId& node) const
        -> tcb::span<const Edge>;

    auto rebuild(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                 const std::vector<std::pair<NodeId, Edge>>& needless_edges)
        -> void;

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
