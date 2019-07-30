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

    auto getForwardEdgesOf(const NodeId& node,
                           const NodeLevel& level = 0) const
        -> tcb::span<const Edge>;

    auto getBackwardEdgesOf(const NodeId& node,
                            const NodeLevel& level = 0) const
        -> tcb::span<const Edge>;

    auto getForwardOffsetArray() const
        -> const std::vector<NodeOffset>&;

    auto getBackwardOffsetArray() const
        -> const std::vector<NodeOffset>&;

    auto getNumberOfNodes() const
        -> std::uint_fast32_t;

    auto getNumberOfEdges() const
        -> std::uint_fast32_t;

    auto getLevelOf(const NodeId& node) const
        -> NodeLevel;

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
