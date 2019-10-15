#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iterator>
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


auto UnidirectionGraph::rebuild(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                                const std::vector<std::pair<NodeId, Edge>>& needless_edges)
    -> void
{
    std::vector<NodeOffset> offsets(offset_array_.size(), 0);
    std::vector<Edge> edges;

    NodeOffset offset{0};
    for(int i{0}; i < offset_array_.size(); i++) {
        offsets[i] = offset;

        //add all shortcuts of node i
        std::copy_if(std::make_move_iterator(std::begin(shortcuts)),
                     std::make_move_iterator(std::end(shortcuts)),
                     std::back_inserter(edges),
                     [i](const auto& shortcut) {
                         const auto& [node, _] = shortcut;
                         return node == i;
                     });

        //add all edges which were not replaced by a shortcut
        auto known_edges = getEdgesOf(i);
        std::copy_if(std::make_move_iterator(std::begin(known_edges)),
                     std::make_move_iterator(std::end(known_edges)),
                     std::back_inserter(edges),
                     [&](const auto& known_edge) {
                         return std::any_of(std::cbegin(needless_edges),
                                            std::cend(needless_edges),
                                            [&](const auto& pair) {
                                                const auto& [node, needless_edge] = pair;
                                                return node == i
                                                    && known_edge == needless_edge;
                                            });
                     });

        offset = edges.size();
    }


    offsets[offset_array_.size()] = edges.size();

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}
