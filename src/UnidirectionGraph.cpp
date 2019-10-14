#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <span.hpp>

using datastructure::Edge;
using datastructure::UnidirectionGraph;

UnidirectionGraph::UnidirectionGraph(std::vector<std::vector<Edge>> adjacency_list)
{
    if(adjacency_list.empty()) {
        return;
    }

    std::vector<NodeOffset> offsets(adjacency_list.size() + 1, 0);
    std::vector<Edge> edges;

    NodeOffset offset{0};
    for(int i{0}; i < adjacency_list.size(); i++) {
        offsets[i] = offset;
        std::move(std::cbegin(adjacency_list[i]),
                  std::cend(adjacency_list[i]),
                  std::back_inserter(edges));
        offset = edges.size();
    }


    offsets[adjacency_list.size()] = edges.size();

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}

UnidirectionGraph::UnidirectionGraph(std::vector<std::vector<Edge>> adjacency_list,
                                     const std::vector<NodeLevel>& node_levels)
{

    std::vector new_edges(adjacency_list.size(),
                          std::vector<Edge>{});

    //erase edges where level[source] >= level[target]
    for(int from{0}; from < adjacency_list.size(); from++) {
        std::copy_if(std::make_move_iterator(std::begin(adjacency_list[from])),
                     std::make_move_iterator(std::end(adjacency_list[from])),
                     std::back_inserter(new_edges[from]),
                     [&](auto&& edge) {
                         auto to = edge.getDestination();
                         return node_levels[to] > node_levels[from];
                     });
    }

    *this = UnidirectionGraph{std::move(new_edges)};
}

UnidirectionGraph::UnidirectionGraph(std::vector<NodeOffset> offset_array,
                                     std::vector<Edge> edges)
    : offset_array_(std::move(offset_array)),
      edges_(edges) {}


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
