#include "MultiTargetDijkstra.hpp"
#include <ContractionGraph.hpp>
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <future>
#include <iterator>
#include <numeric>
#include <span.hpp>
#include <vector>

using datastructure::Edge;
using datastructure::NodeId;
using datastructure::UnidirectionGraph;
using datastructure::ContractionGraph;
using pathfinding::MultiTargetDijkstra;

auto ContractionGraph::areIndependent(NodeId first, NodeId second) const
    -> bool
{
    auto forward_edges = graph_.getForwardEdgesOf(first);
    auto backward_edges = graph_.getBackwardEdgesOf(first);

    auto is_successor =
        std::find_if(std::cbegin(backward_edges),
                     std::cend(backward_edges),
                     [second](const auto& edge) {
                         return edge.getDestination() == second;
                     })
        != std::cend(backward_edges);

    if(is_successor) {
        return false;
    }

    auto is_predecessor =
        std::find_if(std::cbegin(forward_edges),
                     std::cend(forward_edges),
                     [second](const auto& edge) {
                         return edge.getDestination() == second;
                     })
        != std::cend(forward_edges);

    return !is_predecessor;
}


auto ContractionGraph::numberOfIngoingEdges(NodeId node) const
    -> std::int64_t
{
    return graph_.getBackwardEdgesOf(node).size();
}

auto ContractionGraph::numberOfOutgoingEdges(NodeId node) const
    -> std::int64_t
{
    return graph_.getForwardEdgesOf(node).size();
}

auto ContractionGraph::constructIndependentSet() const
    -> std::vector<NodeId>
{
    auto nodes = getEdgeDegreeSortedNodes();
    std::vector<bool> visited(nodes.size(), false);
    std::vector<NodeId> independent_set;

    for(auto node : nodes) {
        if(!visited[node] && graph_.getLevelOf(node) != 0) {
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

auto ContractionGraph::getEdgeDegreeSortedNodes() const
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


auto ContractionGraph::getDegreeOf(NodeId node) const
    -> std::int64_t
{
    auto forward = graph_.getForwardEdgesOf(node).size();
    auto backward = graph_.getBackwardEdgesOf(node).size();

    return forward + backward;
}


auto ContractionGraph::contract(NodeId node) const
    -> std::pair<std::vector<std::pair<NodeId, Edge>>,
                 std::vector<std::pair<NodeId, Edge>>>
{
    MultiTargetDijkstra dijkstra{graph_};

    auto source_edges = graph_.getBackwardEdgesOf(node);
    auto target_edges = graph_.getForwardEdgesOf(node);

    std::vector<std::pair<NodeId, Edge>> to_delete_edges;
    std::vector<std::pair<NodeId, Edge>> to_add_edges;

    for(auto source_edge : source_edges) {
        for(auto target_edge : target_edges) {
            auto source = source_edge.getDestination();
            auto target = target_edge.getDestination();

            auto shortest_distance =
                dijkstra.shortestDistanceFromTo(source, target);

            auto distance_over_node =
                target_edge.getCost() + source_edge.getCost();

            if(shortest_distance == distance_over_node) {
                //edge repressenting the edge from the source to the
                //currently contracted node
                Edge to_contracted{node, source_edge.getCost()};
                Edge shortcut{target, shortest_distance};

                to_delete_edges.emplace_back(source, to_contracted);
                to_delete_edges.emplace_back(node, target_edge);
                to_add_edges.emplace_back(source, shortcut);
            }
        }
        dijkstra.cleanup();
    }

    return {to_delete_edges,
            to_add_edges};
}

namespace {
//the stl does not provide transform_if
template<class InputIt, class OutputIt, class Pred, class Fct>
void transform_if(InputIt first, InputIt last, OutputIt dest, Pred pred, Fct transform)
{
    while(first != last) {
        if(pred(*first))
            *dest++ = transform(*first);

        ++first;
    }
}
} // namespace

auto ContractionGraph::getBestContractions(std::vector<NodeId> independent_set)
    -> std::pair<
        std::vector<std::pair<NodeId, // source
                              Edge>>, //edges to delete
        std::vector<std::pair<NodeId, //source
                              Edge>>> //edges to add
{
    using ContractionInfo =
        std::pair<std::vector<std::pair<NodeId, // source
                                        Edge>>, //edges to delete
                  std::vector<std::pair<NodeId, //source
                                        Edge>>>;

    using SortableContractionInfo =
        std::pair<std::int64_t,
                  ContractionInfo>;

    std::vector<std::future<SortableContractionInfo>> future_vec;

    std::transform(std::cbegin(independent_set),
                   std::cend(independent_set),
                   std::back_inserter(future_vec),
                   [this](auto node) {
                       return std::async([this, node]() {
                           auto [to_delete, to_add] = contract(node);
                           auto edge_difference =
                               to_add.size() - to_delete.size();

                           ContractionInfo info{std::move(to_delete),
                                                std::move(to_add)};

                           return SortableContractionInfo{edge_difference,
                                                          std::move(info)};
                       });
                   });

    std::vector<SortableContractionInfo> result_vec;

    std::transform(std::make_move_iterator(std::begin(future_vec)),
                   std::make_move_iterator(std::end(future_vec)),
                   std::back_inserter(result_vec),
                   [](auto&& future) {
                       return future.get();
                   });

    auto average =
        std::accumulate(std::cbegin(result_vec),
                        std::cend(result_vec),
                        0,
                        [](auto current, const auto& next) {
                            return current + next.first;
                        })
        / result_vec.size();

    std::vector<std::pair<NodeId,
                          Edge>>
        shortcuts;
    std::vector<std::pair<NodeId,
                          Edge>>
        useless_edges;

    for(auto [edgecount, edges] : result_vec) {
        auto [to_add, to_delete] = std::move(edges);

        std::copy_if(std::make_move_iterator(std::begin(to_add)),
                     std::make_move_iterator(std::end(to_add)),
                     std::back_inserter(shortcuts),
                     [&](const auto&) {
                         return edgecount < average;
                     });

        std::copy_if(std::make_move_iterator(std::begin(to_delete)),
                     std::make_move_iterator(std::end(to_delete)),
                     std::back_inserter(useless_edges),
                     [&](const auto&) {
                         return edgecount < average;
                     });
    }


    return {shortcuts, useless_edges};
}
