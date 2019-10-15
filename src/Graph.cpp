#include <Graph.hpp>
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fstream>
#include <future>
#include <span.hpp>

using datastructure::Edge;
using datastructure::UnidirectionGraph;
using datastructure::Graph;

Graph::Graph(UnidirectionGraph&& forward_graph,
             UnidirectionGraph&& backward_graph,
             std::vector<NodeLevel>&& levels)
    : forward_graph_(std::move(forward_graph)),
      backward_graph_(std::move(backward_graph)),
      node_levels_(std::move(levels)) {}


auto Graph::getForwardEdgesOf(const NodeId& node) const
    -> tcb::span<const Edge>
{
    return forward_graph_.getEdgesOf(node);
}

auto Graph::getBackwardEdgesOf(const NodeId& node) const
    -> tcb::span<const Edge>
{
    return backward_graph_.getEdgesOf(node);
}

auto Graph::getLevelOf(const NodeId& node) const
    -> NodeLevel
{
    return node_levels_[node];
}

auto Graph::setLevelOf(NodeId node, NodeLevel level)
    -> void
{
    node_levels_[node] = level;
}

auto Graph::rebuild(const std::vector<std::pair<NodeId, Edge>>& shortcuts,
                    const std::vector<std::pair<NodeId, Edge>>& needless_edges,
                    NodeLevel level)
    -> void
{
    auto forward_fut = std::async([&]() {
        forward_graph_.rebuild(shortcuts,
                               needless_edges);
    });

    auto backward_fut = std::async([&]() {
        backward_graph_.rebuildBackward(shortcuts,
                                        needless_edges);
    });

    forward_fut.get();
    backward_fut.get();

    for(const auto& [node, _] : shortcuts) {
        node_levels_[node] = level;
    }
}


auto datastructure::readFromAllreadyContractedFile(std::string_view path)
    -> std::optional<Graph>
{
    // Open the File
    std::ifstream in{path.data()};

    // Check if object is valid
    if(!in.is_open()) {
        fmt::print("unable to open file {}\n", path);
        return std::nullopt;
    }

    std::string str;
    //skip the comments
    while(std::getline(in, str)) {
        if(str[0] == '#') {
            continue;
        } else {
            break;
        }
    }

    std::uint_fast32_t number_of_nodes;
    std::uint_fast32_t number_of_edges;

    in >> number_of_nodes >> number_of_edges;

    fmt::print("number of nodes {}\nnumber of edges {}\n",
               number_of_nodes,
               number_of_edges);

    std::vector<NodeLevel> node_levels(number_of_nodes, 0);

    NodeId node;
    NodeId id2;
    double latitude, longitude;
    int elevation;
    NodeLevel level;

    for(int i{0}; i < number_of_nodes; i++) {
        in >> node >> id2 >> latitude >> longitude >> elevation >> level;
        node_levels[node] = level;
    }

    NodeId from;
    NodeId to;
    Distance cost;
    int speed;
    int type;
    NodeId child1;
    NodeId child2;

    std::vector forward_edges(number_of_nodes,
                              std::vector<Edge>{});
    std::vector backward_edges(number_of_nodes,
                               std::vector<Edge>{});

    for(int i{0}; i < number_of_edges; i++) {
        in >> from >> to >> cost >> speed >> type >> child1 >> child2;

        forward_edges[from].emplace_back(cost, to);
        backward_edges[to].emplace_back(cost, from);
    }

    auto forward_future = std::async(
        std::launch::async,
        [](const auto& edges, const auto& levels) {
            return UnidirectionGraph{edges, levels};
        },
        std::cref(forward_edges),
        std::cref(node_levels));

    auto backward_future = std::async(
        std::launch::async,
        [](const auto& edges, const auto& levels) {
            return UnidirectionGraph{edges, levels};
        },
        std::cref(backward_edges),
        std::cref(node_levels));

    return Graph{forward_future.get(),
                 backward_future.get(),
                 std::move(node_levels)};
}

auto datastructure::readFromNonContractedFile(std::string_view path)
    -> std::optional<Graph>
{
    // Open the File
    std::ifstream in{path.data()};

    // Check if object is valid
    if(!in) {
        fmt::print("unable to open file {}\n", path);
        return std::nullopt;
    }

    std::string str;
    //skip the comments
    while(std::getline(in, str)) {
        if(str[0] == '#') {
            continue;
        } else {
            break;
        }
    }

    std::uint_fast32_t number_of_nodes;
    std::uint_fast32_t number_of_edges;

    in >> number_of_nodes >> number_of_edges;

    fmt::print("number of nodes {}\nnumber of edges {}\n",
               number_of_nodes,
               number_of_edges);

    std::vector<NodeLevel> node_levels(number_of_nodes, 0);

    NodeId node;
    NodeId id2;
    double latitude, longitude;
    int elevation;

    for(int i{0}; i < number_of_nodes; i++) {
        in >> node >> id2 >> latitude >> longitude >> elevation;
    }

    NodeId from;
    NodeId to;
    Distance cost;
    int speed;
    int type;

    std::vector forward_edges(number_of_nodes,
                              std::vector<Edge>{});
    std::vector backward_edges(number_of_nodes,
                               std::vector<Edge>{});

    for(int i{0}; i < number_of_edges; i++) {
        in >> from >> to >> cost >> speed >> type;

        forward_edges[from].emplace_back(cost, to);
        backward_edges[to].emplace_back(cost, from);
    }

    auto forward_future = std::async(
        std::launch::async,
        [](const auto& edges) {
            return UnidirectionGraph{edges};
        },
        std::cref(forward_edges));

    auto backward_future = std::async(
        std::launch::async,
        [](const auto& edges) {
            return UnidirectionGraph{edges};
        },
        std::cref(backward_edges));

    return Graph{forward_future.get(),
                 backward_future.get(),
                 std::move(node_levels)};
}

auto Graph::getNumberOfNodes() const
    -> std::uint_fast32_t
{
    return node_levels_.size();
}


auto Graph::getLevels() const
    -> const std::vector<NodeLevel>&
{
    return node_levels_;
}
