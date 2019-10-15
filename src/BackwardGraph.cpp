#include <BackwardGraph.hpp>
#include <GraphEssentials.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iterator>
#include <span.hpp>

using datastructure::Edge;
using datastructure::BackwardGraph;

BackwardGraph::BackwardGraph(std::vector<std::vector<Edge>> adjacency_list)
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

BackwardGraph::BackwardGraph(std::vector<std::vector<Edge>> adjacency_list,
                             const std::vector<NodeLevel>& node_levels)
{
    std::vector new_edges(adjacency_list.size(),
                          std::vector<Edge>{});

    //erase edges where level[source] >= level[target]
    for(int to{0}; to < adjacency_list.size(); to++) {
        std::copy_if(std::make_move_iterator(std::begin(adjacency_list[to])),
                     std::make_move_iterator(std::end(adjacency_list[to])),
                     std::back_inserter(new_edges[to]),
                     [&](auto&& edge) {
                         auto from = edge.getDestination();
                         return node_levels[to] > node_levels[from];
                     });
    }

    *this = BackwardGraph{std::move(new_edges)};
}

auto BackwardGraph::getEdgesOf(const NodeId& node) const
    -> tcb::span<const Edge>
{
    auto start_offset = offset_array_[node];
    auto end_offset = offset_array_[node + 1];
    auto* start = &edges_[start_offset];
    auto* end = &edges_[end_offset];

    return {start, end};
}

auto BackwardGraph::getEdgesOf(const NodeId& node)
    -> tcb::span<Edge>
{
    auto start_offset = offset_array_[node];
    auto end_offset = offset_array_[node + 1];
    auto* start = &edges_[start_offset];
    auto* end = &edges_[end_offset];

    return {start, end};
}


auto BackwardGraph::rebuild(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                            const std::vector<std::pair<NodeId, Edge>>& needless_edges)
    -> void
{
    std::vector<NodeOffset> offsets(offset_array_.size(), 0);
    std::vector<Edge> edges;

    std::vector<std::pair<NodeId, Edge>> backward_shortcuts;
    std::transform(std::cbegin(shortcuts),
                   std::cend(shortcuts),
                   std::back_inserter(backward_shortcuts),
                   [](const auto& pair) {
                       auto [node, edge] = pair;
                       auto to = edge.getDestination();
                       edge.setDestination(node);
                       return std::pair{to, edge};
                   });

    NodeOffset offset{0};
    for(int i{0}; i < offset_array_.size(); i++) {
        offsets[i] = offset;

        //add all shortcuts of node i
        std::copy_if(std::make_move_iterator(std::begin(backward_shortcuts)),
                     std::make_move_iterator(std::end(backward_shortcuts)),
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
                                                //needs to be this way because this is a backwards graph
                                                return node == needless_edge.getDestination()
                                                    && known_edge.getDestination() == i;
                                            });
                     });

        offset = edges.size();
    }

    offsets[offset_array_.size()] = edges.size();

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}
