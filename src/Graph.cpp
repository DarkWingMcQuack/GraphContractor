#include <Graph.hpp>
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fstream>
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


auto datastructure::readFromAllreadyContractedFile(std::string_view path)
    -> std::optional<Graph>
{
    // Open the File
    std::ifstream in{path.data()};

    // Check if object is valid
    if(!in) {
        fmt::print("unable to open file", path);
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
    EdgeCost cost;
    int speed;
    int type;
    NodeId child1;
    NodeId child2;

    std::vector<std::pair<NodeId, Edge>> forward_edges;
    std::vector<std::pair<NodeId, Edge>> backward_edges;
    forward_edges.reserve(number_of_edges);
    backward_edges.reserve(number_of_edges);

    for(int i{0}; i < number_of_edges; i++) {
        in >> from >> to >> cost >> speed >> type >> child1 >> child2;
        Edge forward_edge{to, cost};
        forward_edges.emplace_back(from, forward_edge);

        Edge backward_edge{from, cost};
        backward_edges.emplace_back(to, backward_edge);
    }

    fmt::print("make forward graph\n");
    UnidirectionGraph forward_graph{forward_edges};
    fmt::print("make backward graph\n");
    UnidirectionGraph backward_graph{backward_edges};

    return Graph{std::move(forward_graph),
                 std::move(backward_graph),
                 std::move(node_levels)};
}

auto readFromNonContractedFile(std::string_view path)
    -> std::optional<Graph>
{
    return std::nullopt;
}
