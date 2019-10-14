#include "MultiTargetDijkstra.hpp"
#include <ContractionGraph.hpp>
#include <GraphEssentials.hpp>
#include <UnidirectionGraph.hpp>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iterator>
#include <span.hpp>
#include <vector>

using datastructure::Edge;
using datastructure::NodeId;
using datastructure::UnidirectionGraph;
using datastructure::ContractionGraph;
using pathfinding::MultiTargetDijkstra;

auto ContractionGraph::getForwardEdgesOf(NodeId node) const
    -> tcb::span<const Edge>
{
    auto start_offset = forward_offset_array_[node];
    auto end_offset = forward_offset_array_[node + 1];
    auto* start = &forward_edges_[start_offset];
    auto* end = &forward_edges_[end_offset];

    return {start, end};
}

auto ContractionGraph::getForwardEdgesOf(NodeId node)
    -> tcb::span<Edge>
{
    auto start_offset = forward_offset_array_[node];
    auto end_offset = forward_offset_array_[node + 1];
    auto* start = &forward_edges_[start_offset];
    auto* end = &forward_edges_[end_offset];

    return {start, end};
}

auto ContractionGraph::getBackwardEdgesOf(NodeId node) const
    -> tcb::span<const Edge>
{
    auto start_offset = backward_offset_array_[node];
    auto end_offset = backward_offset_array_[node + 1];
    auto* start = &backward_edges_[start_offset];
    auto* end = &backward_edges_[end_offset];

    return {start, end};
}

auto ContractionGraph::getBackwardEdgesOf(NodeId node)
    -> tcb::span<Edge>
{
    auto start_offset = backward_offset_array_[node];
    auto end_offset = backward_offset_array_[node + 1];
    auto* start = &backward_edges_[start_offset];
    auto* end = &backward_edges_[end_offset];

    return {start, end};
}

auto ContractionGraph::getForwardReachableNodes(NodeId node) const
    -> std::vector<NodeId>
{
    auto edges = getForwardEdgesOf(node);
    std::vector<NodeId> reachable_nodes;

    std::transform(std::cbegin(edges),
                   std::cend(edges),
                   std::back_inserter(reachable_nodes),
                   [](const auto& edge) {
                       return edge.getDestination();
                   });

    return reachable_nodes;
}

auto ContractionGraph::isContracted(NodeId node) const
    -> bool
{
    return contracted_nodes[node];
}

auto ContractionGraph::areIndependent(NodeId first, NodeId second) const
    -> bool
{
    auto first_reachable = getForwardReachableNodes(first);
    auto second_reachable = getForwardReachableNodes(second);

    std::sort(std::begin(first_reachable),
              std::end(first_reachable));
    std::sort(std::begin(second_reachable),
              std::end(second_reachable));

    std::vector<NodeId> intersection;

    std::set_intersection(std::cbegin(first_reachable),
                          std::cend(first_reachable),
                          std::cbegin(second_reachable),
                          std::cend(second_reachable),
                          std::back_inserter(intersection));

    return intersection.empty();
}


auto ContractionGraph::numberOfIngoingEdges(NodeId node) const
    -> std::int64_t
{
    auto edges = getBackwardEdgesOf(node);
    return edges.size();
}

auto ContractionGraph::numberOfOutgoingEdges(NodeId node) const
    -> std::int64_t
{
    auto edges = getForwardEdgesOf(node);
    return edges.size();
}

auto ContractionGraph::constructIndependentSet() const
    -> std::vector<NodeId>
{
}


auto ContractionGraph::contract(NodeId node) const
    -> std::pair<std::vector<std::pair<NodeId, Edge>>,
                 std::vector<std::pair<NodeId, Edge>>>
{
    const auto& graph = graph_opt_.value();
    MultiTargetDijkstra dijkstra{graph};

    auto source_edges = getBackwardEdgesOf(node);
    auto target_edges = getForwardEdgesOf(node);

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

    return {
        to_delete_edges,
        to_add_edges};
}

auto ContractionGraph::computeEdgeDifference(NodeId node) const
    -> std::int64_t
{
    auto [add, del] = contract(node);

    return add.size() - del.size();
}
