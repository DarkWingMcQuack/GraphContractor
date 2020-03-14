#pragma once

#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <cstdint>
#include <optional>
#include <span.hpp>
#include <vector>

namespace datastructure {

class Graph
{
public:
    Graph(UnidirectionGraph&& forward_graph,
          UnidirectionGraph&& backward_graph,
          std::vector<NodeLevel>&& levels);

    auto getForwardEdgesOf(const NodeId& node) const
        -> tcb::span<const Edge>;

    auto getBackwardEdgesOf(const NodeId& node) const
        -> tcb::span<const Edge>;

    auto getNumberOfNodes() const
        -> std::uint_fast32_t;

    auto getLevelOf(const NodeId& node) const
        -> NodeLevel;

    auto setLevelOf(NodeId node, NodeLevel level)
        -> void;

    auto rebuild(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                 const std::vector<std::pair<NodeId, Edge>>& needless_edges,
                 const std::vector<NodeId>& contracted_nodes,
                 NodeLevel level)
        -> void;

    auto addEdges(std::vector<std::pair<NodeId, Edge>> new_edges)
        -> void;

    auto getLevels() const
        -> const std::vector<NodeLevel>&;

private:
    UnidirectionGraph forward_graph_;
    UnidirectionGraph backward_graph_;
    std::vector<NodeLevel> node_levels_;
};

auto readFromAllreadyContractedFile(std::string_view path)
    -> std::optional<Graph>;

auto readFromNonContractedFile(std::string_view path)
    -> std::optional<Graph>;

} // namespace datastructure
