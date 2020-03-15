#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <GraphEssentials.hpp>
#include <MultiTargetDijkstra.hpp>
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
    : graph_(std::move(graph)),
      dijkstra_(graph_) {}

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


    fmt::print("REEEEEEEEEEEEE: {}\n",
               std::accumulate(std::begin(deleted_edges_),
                               std::end(deleted_edges_),
                               0,
                               [](auto current, auto next) {
                                   return current + next.second.size();
                               }));

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

auto GraphContractor::contract(NodeId node)
    -> std::pair<std::unordered_map<NodeId, std::vector<Edge>>,
                 int>
{
    auto source_edges = graph_.getBackwardEdgesOf(node);
    auto target_edges = graph_.getForwardEdgesOf(node);

    std::unordered_map<NodeId, std::vector<Edge>> shortcuts;

    std::size_t number_of_shortcuts{0};

    for(auto source_edge : source_edges) {
        dijkstra_.cleanup();

        auto source = source_edge.getDestination();
        auto source_node_cost = source_edge.getCost();

        for(auto target_edge : target_edges) {
            auto target = target_edge.getDestination();
            auto node_target_cost = target_edge.getCost();

            auto distance_over_node =
                source_node_cost + node_target_cost;

            auto shortest_distance =
                dijkstra_.shortestDistanceFromTo(source,
                                                 target);

            // fmt::print("shortest: {}\n", shortest_distance);
            // fmt::print("over node: {}\n", distance_over_node);

            if(shortest_distance >= distance_over_node) {
                // Edge shortcut{distance_over_node, target};
                // shortcuts.emplace_back(source, shortcut);
                shortcuts[source].emplace_back(distance_over_node,
                                               target);
                number_of_shortcuts++;
            }
        }
    }

    fmt::print("REEEEEEEEEEEEE: {}\n",
               std::accumulate(std::begin(shortcuts),
                               std::end(shortcuts),
                               0,
                               [](auto current, auto next) {
                                   return current + next.second.size();
                               }));


    auto edge_diff = number_of_shortcuts
        - (source_edges.size() + target_edges.size());

    return {shortcuts, edge_diff};
}

auto GraphContractor::getBestContractions(std::vector<NodeId> independent_set)
    -> std::pair<
        std::unordered_map<NodeId, std::vector<Edge>>,
        std::vector<NodeId>>
{
    using ContractionInfo =
        std::pair<std::unordered_map<NodeId, //source
                                     std::vector<Edge>>,
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

    std::unordered_map<NodeId,
                       std::vector<Edge>>
        shortcuts;

    std::vector<NodeId> contracted_nodes;

    int counter{0};
    for(auto& [shortcut, edgediff] : result_vec) {
        auto current_node = independent_set[counter++];

        if(edgediff <= average) {
            // shortcuts[current_node] = std::move(shortcut[current_node]);
            shortcuts.insert({current_node, std::move(shortcut[current_node])});
            contracted_nodes.emplace_back(current_node);
        }
    }

    std::sort(std::begin(contracted_nodes),
              std::end(contracted_nodes));

    for(auto node : contracted_nodes) {
        auto source_edges = graph_.getForwardEdgesOf(node);
        auto target_edges = graph_.getBackwardEdgesOf(node);

        deleted_edges_[node]
            .insert(std::end(deleted_edges_[node]),
                    std::begin(source_edges),
                    std::end(source_edges));

        for(const auto& edge : target_edges) {
            const auto& cost = edge.getCost();
            const auto& from = edge.getDestination();

            deleted_edges_[from].emplace_back(cost, node);
        }
    }

    return {shortcuts,
            contracted_nodes};
}
