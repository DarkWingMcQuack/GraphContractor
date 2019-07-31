#pragma once

#include <cstdint>
#include <tuple>

namespace datastructure {

using NodeId = std::int_fast32_t;
using NodeOffset = std::int_fast32_t;
using Distance = std::int_fast32_t;
using NodeLevel = std::int_fast32_t;

struct Edge
{
public:
    Edge(Distance cost,
         NodeId destination);

    auto getCost() const
        -> Distance;

    auto getDestination() const
        -> NodeId;

private:
    Distance cost_;
    NodeId destination_;
};

} // namespace datastructure
