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

    Edge(Edge&&) = default;
    Edge(const Edge&) = default;

    auto operator=(const Edge&)
        -> Edge& = default;

    auto operator=(Edge &&)
        -> Edge& = default;

    auto getCost() const
        -> Distance;

    auto getDestination() const
        -> NodeId;

    auto setDestination(NodeId destination)
        -> void;

    auto operator==(const Edge&) const
        -> bool;


private:
    Distance cost_;
    NodeId destination_;
};

} // namespace datastructure
