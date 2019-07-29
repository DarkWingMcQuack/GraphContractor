#include <Graph.hpp>
#include <algorithm>
#include <span.hpp>

using datastructure::Edge;
using datastructure::GraphPart;
using datastructure::Graph;


Edge::Edge(EdgeCost cost,
           NodeId destination)
    : cost_(cost),
      destination_(destination) {}

auto Edge::getCost() const
    -> EdgeCost
{
    return cost_;
}

auto Edge::getDestination() const
    -> NodeId
{
    return destination_;
}


GraphPart::GraphPart(std::vector<std::pair<NodeId, Edge>> node_edges,
                     const std::vector<NodeLevel>& node_levels)
    : levels_(node_levels)
{
    if(node_edges.empty()) {
        return;
    }

    std::sort(std::begin(node_edges),
              std::end(node_edges),
              [](auto&& lhs, auto&& rhs) {
                  return lhs.first < rhs.first;
              });

    std::vector offsets(0, node_levels.size());
    std::vector<Edge> edges;
    edges.reserve(node_edges.size());

    for(auto&& [node, edge] : node_edges) {
        offsets[node]++;
        edges.push_back(std::move(edge));
    }

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}

auto GraphPart::getEdgesOf(const NodeId& node) const
    -> tcb::span<const Edge>
{
    auto number_of_edges = getNumberOfEdgesOf(node);
    auto offset = offset_array_[node];
    auto* start = &edges_[offset];

    return {start, number_of_edges};
}

auto GraphPart::getOffsetArray() const
    -> const std::vector<NodeOffset>&
{
    return offset_array_;
}

auto GraphPart::getEdges() const
    -> const std::vector<Edge>&
{
    return edges_;
}


auto GraphPart::getNumberOfEdgesOf(const NodeId& node) const
    -> std::int_fast32_t
{
    if(__builtin_expect((node == offset_array_.size() - 1), 0)) {
        return edges_.size() - 1
            - offset_array_[node];
    }

    return offset_array_[node + 1]
        - offset_array_[node];
}
