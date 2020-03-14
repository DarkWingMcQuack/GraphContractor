#include "Graph.hpp"
#include "MultiTargetDijkstra.hpp"
#include <GraphContractor.hpp>
#include <GraphEssentials.hpp>
#include <ParallelTransform.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <future>
#include <iterator>
#include <numeric>
#include <parallel/algorithm>
#include <span.hpp>
#include <vector>

using datastructure::Edge;
using datastructure::NodeId;
using algorithm::transform_par;
using datastructure::UnidirectionGraph;
using datastructure::GraphContractor;
using pathfinding::MultiTargetDijkstra;


GraphContractor::GraphContractor(Graph graph)
    : graph_(std::move(graph)) {}

auto GraphContractor::contractGraph()
    -> void
{
    while(!graphFullContracted()) {
        current_level++;

        fmt::print("build independent set...\n");
        auto independent_set = constructIndependentSet();

        fmt::print("calculate contractions...\n");
        auto [shortcuts, nodes] =
            getBestContractions(std::move(independent_set));

        fmt::print("rebuild graph...\n");
        graph_.rebuild(shortcuts, nodes, current_level);

        fmt::print("done with level {}\n", current_level);
    }

    fmt::print("REEEEEE: {}\n", deleted_edges_.size());
    graph_.addEdges(std::move(deleted_edges_));
    deleted_edges_.clear();
}

auto GraphContractor::getGraph()
    -> Graph&
{
    return graph_;
}


auto GraphContractor::graphFullContracted() const
    -> bool
{
    const auto& levels = graph_.getLevels();
    return std::all_of(std::cbegin(levels),
                       std::cend(levels),
                       [](auto level) {
                           return level != 0;
                       });
}


auto GraphContractor::numberOfIngoingEdges(NodeId node) const
    -> std::int64_t
{
    return graph_.getBackwardEdgesOf(node).size();
}

auto GraphContractor::numberOfOutgoingEdges(NodeId node) const
    -> std::int64_t
{
    return graph_.getForwardEdgesOf(node).size();
}

auto GraphContractor::constructIndependentSet() const
    -> std::vector<NodeId>
{
    auto nodes = getEdgeDegreeSortedNodes();
    std::vector<bool> visited(nodes.size(), false);
    std::vector<NodeId> independent_set;

    for(auto node : nodes) {
        if(!visited[node] && graph_.getLevelOf(node) == 0) {
            visited[node] = true;
            independent_set.emplace_back(node);

            auto forward_edges = graph_.getForwardEdgesOf(node);
            auto backward_edges = graph_.getBackwardEdgesOf(node);

            for(const auto& edge : forward_edges) {
                auto target = edge.getDestination();
                visited[target] = true;
            }

            for(const auto& edge : backward_edges) {
                auto target = edge.getDestination();
                visited[target] = true;
            }
        }
    }

    return independent_set;
}

auto GraphContractor::getEdgeDegreeSortedNodes() const
    -> std::vector<NodeId>
{
    std::vector<NodeId> nodes(graph_.getNumberOfNodes());
    std::iota(std::begin(nodes),
              std::end(nodes),
              0);

    std::sort(std::begin(nodes),
              std::end(nodes),
              [this](const auto& lhs, const auto& rhs) {
                  return getDegreeOf(lhs) < getDegreeOf(rhs);
              });

    return nodes;
}

auto GraphContractor::getDegreeOf(NodeId node) const
    -> std::int64_t
{
    auto forward = graph_.getForwardEdgesOf(node).size();
    auto backward = graph_.getBackwardEdgesOf(node).size();

    return forward + backward;
}

auto GraphContractor::contract(NodeId node) const
    -> std::pair<std::vector<std::pair<NodeId, Edge>>,
                 int>
{
    MultiTargetDijkstra dijkstra{graph_};

    auto source_edges = graph_.getBackwardEdgesOf(node);
    auto target_edges = graph_.getForwardEdgesOf(node);

    std::vector<std::pair<NodeId, Edge>> shortcuts;

    for(auto source_edge : source_edges) {
        auto source = source_edge.getDestination();
        auto source_node_cost = source_edge.getCost();

        for(auto target_edge : target_edges) {
            auto target = target_edge.getDestination();
            auto node_target_cost = target_edge.getCost();

            auto distance_over_node =
                source_node_cost + node_target_cost;

            auto shortest_distance =
                dijkstra.shortestDistanceForContracion(source,
                                                       target,
                                                       node,
                                                       distance_over_node);


            if(shortest_distance == distance_over_node) {
                Edge shortcut{shortest_distance, target};
                shortcuts.emplace_back(source, shortcut);
            }
        }

        dijkstra.cleanup();
    }

    auto edge_diff = shortcuts.size()
        - (source_edges.size() + target_edges.size());

    return {shortcuts, edge_diff};
}

auto GraphContractor::getBestContractions(std::vector<NodeId> independent_set)
    -> std::pair<
        std::vector<std::pair<NodeId, Edge>>, // shortcuts
        std::vector<NodeId>> //contracted nodes
{
    // auto result_vec = transform_par(std::cbegin(independent_set),
    //                                 std::cend(independent_set),
    //                                 [this](auto node) {
    //                                     return contract(node);
    //                                 });

    using ContractionInfo =
        std::pair<std::vector<std::pair<NodeId, //source
                                        Edge>>,
                  int>; //shortcuts

    std::vector<ContractionInfo> result_vec;
    std::transform(std::cbegin(independent_set),
                   std::cend(independent_set),
                   std::back_inserter(result_vec),
                   [this](auto node) {
                       return contract(node);
                   });

    auto sum =
        std::accumulate(std::cbegin(result_vec),
                        std::cend(result_vec),
                        0.,
                        [](auto current, const auto& next) {
                            return current + next.second;
                        });


    auto average = static_cast<double>(sum)
        / static_cast<double>(result_vec.size());

    fmt::print("AVERAGE: {}\n", average);

    std::vector<std::pair<NodeId,
                          Edge>>
        shortcuts;

    std::vector<NodeId> contracted_nodes;

    int counter{0};
    for(auto& [shortcut, edgediff] : result_vec) {
        auto current_node = independent_set[counter++];

        if(edgediff <= average) {
            std::move(std::begin(shortcut),
                      std::end(shortcut),
                      std::back_inserter(shortcuts));

            contracted_nodes.emplace_back(current_node);
        }
    }

    std::sort(std::begin(contracted_nodes),
              std::end(contracted_nodes));

    for(auto node : contracted_nodes) {
        auto source_edges = graph_.getForwardEdgesOf(node);
        auto target_edges = graph_.getBackwardEdgesOf(node);

        std::transform(std::cbegin(source_edges),
                       std::end(source_edges),
                       std::back_inserter(deleted_edges_),
                       [&](auto edge) {
                           return std::pair{node, std::move(edge)};
                       });

        std::transform(std::cbegin(target_edges),
                       std::end(target_edges),
                       std::back_inserter(deleted_edges_),
                       [&](auto back_edge) {
                           auto from = back_edge.getDestination();
                           Edge edge{back_edge.getCost(), node};
                           return std::pair{from, std::move(edge)};
                       });
    }

    fmt::print("contracted: {}\n", contracted_nodes.size());
    fmt::print("SHORTCUTS: {}\n", shortcuts.size());

    return {shortcuts,
            contracted_nodes};
}
