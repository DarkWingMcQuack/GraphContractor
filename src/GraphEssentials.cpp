#include <GraphEssentials.hpp>

using datastructure::Edge;

Edge::Edge(Distance cost,
           NodeId destination)
    : cost_(cost),
      destination_(destination) {}

auto Edge::getCost() const
    -> Distance
{
    return cost_;
}

auto Edge::getDestination() const
    -> NodeId
{
    return destination_;
}
