#include <Graph.hpp>
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
    if(node == offset_array_.size() - 1) {
        return offset_array_.size()
            - offset_array_[node];
    }

    return offset_array_[node + 1]
        - offset_array_[node];
}
