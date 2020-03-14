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
        auto [shortcuts, useless, nodes] =
            getBestContractions(std::move(independent_set));

        fmt::print("rebuild graph...\n");
        graph_.rebuild(shortcuts, useless, nodes, current_level);

        std::move(std::begin(useless),
                  std::end(useless),
                  std::back_inserter(deleted_edges_));

        fmt::print("done with level {}\n", current_level);
    }

    graph_.addEdges(std::move(deleted_edges_));
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
                 std::vector<std::pair<NodeId, Edge>>>
{
    MultiTargetDijkstra dijkstra{graph_};

    auto source_edges = graph_.getBackwardEdgesOf(node);
    auto target_edges = graph_.getForwardEdgesOf(node);

    std::vector<std::pair<NodeId, Edge>> unnecessary;
    std::vector<std::pair<NodeId, Edge>> shortcuts;

	unnecessary.reserve(source_edges.size());
	shortcuts.reserve(target_edges.size());

    std::transform(std::cbegin(source_edges),
                   std::cend(source_edges),
                   std::back_inserter(unnecessary),
                   [node](const auto& backwards_edge) {
                       auto source = backwards_edge.getDestination();
                       Edge to_node{backwards_edge.getCost(), node};
                       return std::pair{source, to_node};
                   });

    std::transform(std::cbegin(target_edges),
                   std::cend(target_edges),
                   std::back_inserter(unnecessary),
                   [node](auto edge) {
                       return std::pair{node, edge};
                   });

    for(auto source_edge : source_edges) {
        auto source = source_edge.getDestination();
        for(auto target_edge : target_edges) {
            auto target = target_edge.getDestination();

            auto shortest_distance =
                dijkstra.shortestDistanceFromTo(source, target);

            auto distance_over_node =
                target_edge.getCost() + source_edge.getCost();

            if(shortest_distance == distance_over_node) {
                Edge shortcut{shortest_distance, target};
                shortcuts.emplace_back(source, shortcut);
            }
        }

        dijkstra.cleanup();
    }

    return {unnecessary,
            shortcuts};
}

auto GraphContractor::getBestContractions(std::vector<NodeId> independent_set)
    -> std::tuple<
        std::vector<std::pair<NodeId, // source
                              Edge>>, //edges to delete
        std::vector<std::pair<NodeId, //source
                              Edge>>,
        std::vector<NodeId>> //edges to add
{
    auto result_vec = transform_par(std::cbegin(independent_set),
                                    std::cend(independent_set),
                                    [this](auto node) {
                                        return contract(node);
                                    });

    auto sum =
        std::accumulate(std::cbegin(result_vec),
                        std::cend(result_vec),
                        0.,
                        [](auto current, const auto& next) {
                            auto edgediff = static_cast<double>(next.second.size())
                                - static_cast<double>(next.first.size());
                            return current + edgediff;
                        });

    auto average = sum / static_cast<double>(result_vec.size());

    std::vector<std::pair<NodeId,
                          Edge>>
        shortcuts;
    std::vector<std::pair<NodeId,
                          Edge>>
        useless_edges;

    std::vector<NodeId> contracted_nodes;

    int counter{0};
    for(auto [to_delete, to_add] : result_vec) {
        auto current_node = independent_set[counter++];

        auto edgediff = static_cast<double>(to_add.size())
            - static_cast<double>(to_delete.size());

        if(edgediff <= average) {
            std::move(std::begin(to_add),
                      std::end(to_add),
                      std::back_inserter(shortcuts));

            std::move(std::begin(to_delete),
                      std::end(to_delete),
                      std::back_inserter(useless_edges));

            contracted_nodes.emplace_back(current_node);
        }
    }

    return {shortcuts,
            useless_edges,
            contracted_nodes};
}
