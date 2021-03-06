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

auto Graph::rebuild(const std::unordered_map<NodeId, std::vector<Edge>>& shortcuts,
                    const std::vector<NodeId>& contracted_nodes,
                    NodeLevel level)
    -> void
{
    forward_graph_.rebuild(shortcuts,
                           contracted_nodes);

    backward_graph_.rebuildBackward(shortcuts,
                                    contracted_nodes);

    for(auto node : contracted_nodes) {
        node_levels_[node] = level;
    }
}


auto Graph::addEdges(std::unordered_map<NodeId, std::vector<Edge>> new_edges)
    -> void
{
    std::unordered_map<NodeId, std::vector<Edge>> forward_edges;
    std::unordered_map<NodeId, std::vector<Edge>> backward_edges;

    fmt::print("building edges\n");
    for(auto&& [from, edges] : std::move(new_edges)) {
        for(auto&& edge : std::move(edges)) {
            const auto& target = edge.getDestination();
            if(getLevelOf(from) < getLevelOf(target)) {
                forward_edges[from].emplace_back(std::move(edge));
            } else {
                backward_edges[from].emplace_back(std::move(edge));
            }
        }
    }

	new_edges.clear();

    fmt::print("edges build\n");

    fmt::print("rebuilding forward graph\n");
    forward_graph_.rebuild(forward_edges, {});
    fmt::print("forward graph done\n");

    fmt::print("rebuild back graph\n");

    backward_graph_.rebuildBackward(backward_edges, {});
    fmt::print("back graph done\n");
}

auto Graph::getNumberOfNodes() const
    -> std::uint_fast32_t
{
    return node_levels_.size();
}

auto Graph::getNumberOfEdges() const
    -> std::uint_fast32_t
{
    return forward_graph_.edges_.size();
}

auto Graph::getLevels() const
    -> const std::vector<NodeLevel>&
{
    return node_levels_;
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

    for(size_t i{0}; i < number_of_nodes; i++) {
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

    for(size_t i{0}; i < number_of_edges; i++) {
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

    for(size_t i{0}; i < number_of_nodes; i++) {
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

    for(size_t i{0}; i < number_of_edges; i++) {
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
