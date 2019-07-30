#pragma once

#include <cstdint>
#include <tuple>

namespace datastructure {

using NodeId = std::int_fast32_t;
using NodeOffset = std::int_fast32_t;
using EdgeCost = std::int_fast32_t;
using NodeLevel = std::int_fast32_t;

struct Edge
{
public:
    Edge(EdgeCost cost,
         NodeId destination);

    auto getCost() const
        -> EdgeCost;

    auto getDestination() const
        -> NodeId;

private:
    EdgeCost cost_;
    NodeId destination_;
};

} // namespace datastructure
