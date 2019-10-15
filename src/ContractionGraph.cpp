#include "MultiTargetDijkstra.hpp"
#include <ContractionGraph.hpp>
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <future>
#include <iterator>
#include <span.hpp>
#include <vector>

using datastructure::Edge;
using datastructure::NodeId;
using datastructure::UnidirectionGraph;
using datastructure::ContractionGraph;
using pathfinding::MultiTargetDijkstra;

auto ContractionGraph::isContracted(NodeId node) const
    -> bool
{
    return contracted_nodes[node];
}

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
    // std::vector<bool> markedNodes;
    // std::vector<NodeId> independent_set;
    // independent_set.reserve(count);

    // auto number_of_nodes = forward_offset_array_.size() - 1;

    // NodeId first_node = std::rand() % forward_offset_array_.size() - 1;

    // for(auto i = first_node; i < number_of_nodes; i++) {
    //     NodeId id = graph.nodes[i].id;
    //     if(!markedNodes[id]) {
    //         markedNodes[id] = true;
    //         independentSet.emplace_back(id);
    //         // mark all outgoing nodes
    //         for(auto j = graph.offset[id]; j < graph.offset[id + 1]; ++j) {
    //             markedNodes[graph.edges[j].target] = true;
    //         }
    //         for(auto j = backgraph.offset[id]; j < backgraph.offset[id + 1]; ++j) {
    //             markedNodes[backgraph.edges[j].target] = true;
    //         }
    //     }
    // }
}


auto ContractionGraph::contract(NodeId node) const
    -> std::pair<std::vector<std::pair<NodeId, Edge>>,
                 std::vector<std::pair<NodeId, Edge>>>
{
    const auto& graph = graph_opt_.value();
    MultiTargetDijkstra dijkstra{graph};

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

auto ContractionGraph::getNSmallestEdgeDifferenceContractions(std::vector<NodeId> independent_set,
                                                              std::int64_t number_of_maximal_contractions)
    -> std::vector<
        std::pair<std::vector<std::pair<NodeId, // source
                                        Edge>>, //edges to delete
                  std::vector<std::pair<NodeId, //source
                                        Edge>>>> //edges to add
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

    std::vector<ContractionInfo> ret_vec;

    if(number_of_maximal_contractions > result_vec.size()) {
        std::transform(std::make_move_iterator(std::begin(result_vec)),
                       std::make_move_iterator(std::end(result_vec)),
                       std::back_inserter(ret_vec),
                       [](auto&& result) {
                           return result.second;
                       });
    } else {
        auto max_elem_iter = std::begin(result_vec)
            + number_of_maximal_contractions - 1;

        std::nth_element(std::begin(result_vec),
                         max_elem_iter,
                         std::end(result_vec),
                         [](const auto& lhs, const auto& rhs) {
                             return lhs.first < rhs.first;
                         });

        std::transform(std::make_move_iterator(std::begin(result_vec)),
                       std::make_move_iterator(max_elem_iter),
                       std::back_inserter(ret_vec),
                       [](auto&& result) {
                           return result.second;
                       });
    }

    return ret_vec;
}
