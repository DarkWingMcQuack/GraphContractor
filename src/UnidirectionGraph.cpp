#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <span.hpp>

using datastructure::Edge;
using datastructure::UnidirectionGraph;

UnidirectionGraph::UnidirectionGraph(std::vector<std::pair<NodeId, Edge>> node_edges)
{
    if(node_edges.empty()) {
        return;
    }

    std::sort(std::begin(node_edges),
              std::end(node_edges),
              [](auto&& lhs, auto&& rhs) {
                  return lhs.first < rhs.first;
              });

    std::vector<NodeOffset> offsets(node_edges.back().first + 1, 0);
    std::vector<Edge> edges;
    edges.reserve(node_edges.size());

    NodeOffset offset{0};
    for(auto&& [node, edge] : node_edges) {
        offsets[node] = ++offset;
        edges.push_back(std::move(edge));
    }

    for(int i{1}; i < offsets.size(); i++) {
        offsets[i] = std::max(offsets[i], offsets[i - 1]);
    }

    edges_ = std::move(edges);
    offset_array_ = std::move(offsets);
}

auto UnidirectionGraph::getEdgesOf(const NodeId& node) const
    -> tcb::span<const Edge>
{
    auto number_of_edges = getNumberOfEdgesOf(node);

    auto offset = [&]() {
        if(__builtin_expect((node == 0), 0)) {
            return 0l;
        }

        return offset_array_[node - 1];
    }();

    auto* start = &edges_[offset];

    return {start, number_of_edges};
}

auto UnidirectionGraph::getEdgesOf(const NodeId& node)
    -> tcb::span<Edge>
{
    auto number_of_edges = getNumberOfEdgesOf(node);

    auto offset = [&]() {
        if(__builtin_expect((node == 0), 0)) {
            return 0l;
        }

        return offset_array_[node - 1];
    }();

    auto* start = &edges_[offset];

    return {start, number_of_edges};
}

auto UnidirectionGraph::getOffsetArray() const
    -> const std::vector<NodeOffset>&
{
    return offset_array_;
}

auto UnidirectionGraph::getEdges() const
    -> const std::vector<Edge>&
{
    return edges_;
}

auto UnidirectionGraph::getNumberOfEdgesOf(const NodeId& node) const
    -> std::int_fast32_t
{
    if(__builtin_expect((node == 0), 0)) {
        return offset_array_[1];
    }

    return offset_array_[node]
        - offset_array_[node - 1];
}
auto UnidirectionGraph::sortEdgesByNodeLevel(const std::vector<NodeLevel>& node_levels)
    -> void
{
    for(NodeId node{0}; node < node_levels.size(); node++) {
        auto edges = getEdgesOf(node);
        std::sort(
            std::begin(edges),
            std::end(edges),
            [&](const auto& lhs, const auto& rhs) {
                auto lhs_dest = lhs.getDestination();
                auto rhs_dest = rhs.getDestination();
                return node_levels[lhs_dest] < node_levels[rhs_dest];
            });
    }
}
