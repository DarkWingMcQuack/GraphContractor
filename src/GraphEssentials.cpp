#include <GraphEssentials.hpp>

using datastructure::Edge;
using datastructure::Distance;
using datastructure::NodeId;

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

auto Edge::setDestination(NodeId destination)
    -> void
{
    destination_ = destination;
}


auto Edge::operator==(const Edge& rhs) const
    -> bool
{
    return cost_ == rhs.cost_
        && destination_ == rhs.destination_;
}
