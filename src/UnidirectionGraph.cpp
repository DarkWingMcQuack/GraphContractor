#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <iterator>
#include <span.hpp>

using datastructure::Edge;
using datastructure::NodeId;
using datastructure::UnidirectionGraph;

UnidirectionGraph::UnidirectionGraph(std::vector<std::vector<Edge>> adjacency_list)
{
    if(adjacency_list.empty()) {
        return;
    }

    std::vector<NodeOffset> offsets(adjacency_list.size() + 1, 0);
    std::vector<Edge> edges;

    NodeOffset offset{0};
    for(size_t i{0}; i < adjacency_list.size(); i++) {
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
    for(size_t from{0}; from < adjacency_list.size(); from++) {
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
    auto end_offset = offset_array_.at(node + 1);
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

auto UnidirectionGraph::rebuild(const std::unordered_map<NodeId, std::vector<Edge>>& shortcuts,
                                const std::vector<NodeId>& contracted)
    -> void
{
    std::vector<NodeOffset> offsets(offset_array_.size(), 0);
    std::vector<Edge> edges;
    edges.reserve(edges_.size());

    NodeOffset offset{0};
    for(size_t i{0}; i < offset_array_.size() - 1; i++) {
        std::vector<Edge> new_edges;
        offsets[i] = offset;

        auto is_contracted =
            std::binary_search(std::cbegin(contracted),
                               std::cend(contracted),
                               i);

        if(is_contracted) {
            continue;
        }

        auto map_iter = shortcuts.find(i);
        if(map_iter != shortcuts.end()) {
            std::copy(std::begin(map_iter->second),
                      std::end(map_iter->second),
                      std::back_inserter(new_edges));
        }

        //add all edges which were not replaced by a shortcut
        auto known_edges = getEdgesOf(i);
        std::copy_if(std::begin(known_edges),
                     std::end(known_edges),
                     std::back_inserter(new_edges),
                     [&](const auto& known_edge) {
                         const auto& target = known_edge.getDestination();

                         return !std::binary_search(std::cbegin(contracted),
                                                    std::cend(contracted),
                                                    target);
                     });

        std::sort(std::begin(new_edges),
                  std::end(new_edges),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.getDestination() < rhs.getDestination();
                  });

        Edge best = new_edges.front();
        for(size_t j{0}; j < new_edges.size(); ++j) {
            auto edge = new_edges[j];

            if(edge.getDestination() != best.getDestination()) {
                edges.emplace_back(std::move(best));
                best = edge;
            } else {
                if(best.getCost() > edge.getCost()) {
                    best = edge;
                }
            }
            if(j == new_edges.size() - 1) {
                edges.emplace_back(std::move(edge));
            }
        }

        offset = edges.size();
    }

    offsets[offset_array_.size() - 1] = edges.size();

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}

namespace {

auto switch_direction(const std::unordered_map<NodeId, std::vector<Edge>>& shortcuts)
{
    std::unordered_map<datastructure::NodeId, std::vector<Edge>> ret_map;

    for(const auto& [to, neigs] : shortcuts) {
        for(const auto& edge : neigs) {
            const auto& from = edge.getDestination();
            const auto& cost = edge.getCost();

            ret_map[from].emplace_back(cost, to);
        }
    }

    return ret_map;
}

} // namespace

auto UnidirectionGraph::rebuildBackward(const std::unordered_map<NodeId, std::vector<Edge>>& shortcuts,
                                        const std::vector<NodeId>& contracted)
    -> void
{
    std::vector<NodeOffset> offsets(offset_array_.size(), 0);
    std::vector<Edge> edges;

    auto switched_edges = switch_direction(shortcuts);

	rebuild(switched_edges, contracted);
}
