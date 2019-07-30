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


auto datastructure::createBackwardGraphpart(const UnidirectionGraph& forward)
    -> UnidirectionGraph
{
    std::vector<std::pair<NodeId, Edge>> backward_edges;
    backward_edges.reserve(forward.getEdges().size());

    auto number_of_nodes = forward.getOffsetArray().size();

    for(int i{0}; i < number_of_nodes; i++) {
        auto span = forward.getEdgesOf(i);
        for(auto edge : span) {
            auto from = i;
            auto to = edge.getDestination();
            auto cost = edge.getCost();

            Edge backward_edge{cost,
                               static_cast<NodeId>(from)};
            backward_edges.push_back({to, backward_edge});
        }
    }

    UnidirectionGraph backward_graph{backward_edges};

    return backward_graph;
}
