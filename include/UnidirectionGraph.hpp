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

    UnidirectionGraph(std::vector<NodeOffset> offset_array,
                      std::vector<Edge> edges);

    auto getEdgesOf(const NodeId& node) const
        -> tcb::span<const Edge>;

    auto rebuild(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                 const std::vector<NodeId>& contracted)
        -> void;

    auto rebuildBackward(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                         const std::vector<NodeId>& contracted)
        -> void;

private:
    auto getEdgesOf(const NodeId& node)
        -> tcb::span<Edge>;

private:
    friend class Graph;
    /*
	* offset_array points into edge_ids
	* edge_ids then points into edges to get the properties
	*/
    std::vector<NodeOffset> offset_array_;
    std::vector<Edge> edges_;
};

} // namespace datastructure
