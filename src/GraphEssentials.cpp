#include <GraphEssentials.hpp>

using datastructure::Edge;

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
