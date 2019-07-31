#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <span.hpp>

using datastructure::Edge;
using datastructure::UnidirectionGraph;

UnidirectionGraph::UnidirectionGraph(std::vector<std::vector<Edge>> node_edges)
{
    if(node_edges.empty()) {
        return;
    }

    std::vector<NodeOffset> offsets(node_edges.size() + 1, 0);
    std::vector<Edge> edges;

    NodeOffset offset{0};
    for(int i{0}; i < node_edges.size(); i++) {
        offsets[i] = offset;
        std::move(std::cbegin(node_edges[i]),
                  std::cend(node_edges[i]),
                  std::back_inserter(edges));
        offset = edges.size();
    }


    offsets[node_edges.size()] = edges.size();

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}

UnidirectionGraph::UnidirectionGraph(std::vector<std::vector<Edge>> node_edges,
                                     const std::vector<NodeLevel>& node_levels)
{

    std::vector new_edges(node_edges.size(),
                          std::vector<Edge>{});

    //erase edges where level[source] >= level[target]
    for(int from{0}; from < node_edges.size(); from++) {
        std::copy_if(std::make_move_iterator(std::begin(node_edges[from])),
                     std::make_move_iterator(std::end(node_edges[from])),
                     std::back_inserter(new_edges[from]),
                     [&](auto&& edge) {
                         auto to = edge.getDestination();
                         return node_levels[to] > node_levels[from];
                     });
    }

    *this = UnidirectionGraph{std::move(new_edges)};
}

auto UnidirectionGraph::getEdgesOf(const NodeId& node) const
    -> tcb::span<const Edge>
{
    auto start_offset = offset_array_[node];
    auto end_offset = offset_array_[node + 1];
    auto* start = &edges_[start_offset];
    auto* end = &edges_[end_offset];

    return {start, end};
}

auto UnidirectionGraph::getEdgesOf(const NodeId& node)
    -> tcb::span<Edge>
{
    auto start_offset = offset_array_[node];
    auto end_offset = offset_array_[node + 1];
    auto* start = &edges_[start_offset];
    auto* end = &edges_[end_offset];

    return {start, end};
}
