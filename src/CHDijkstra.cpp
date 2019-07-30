#include <CHDijkstra.hpp>
#include <Graph.hpp>
#include <GraphEssentials.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <future>
#include <numeric>

using datastructure::Graph;
using datastructure::NodeId;
using datastructure::Edge;
using datastructure::EdgeCost;
using datastructure::NodeLevel;
using pathfinding::CHDijkstra;


CHDijkstra::CHDijkstra(const datastructure::Graph& graph)
    : graph_(graph),
      forward_shortest_distances_(graph_.getNumberOfNodes(),
                                  std::numeric_limits<EdgeCost>::max()),
      backward_shortest_distances_(graph_.getNumberOfNodes(),
                                   std::numeric_limits<EdgeCost>::max()) {}

auto CHDijkstra::shortestDistanceFromTo(const NodeId& source,
                                        const NodeId& target)
    -> EdgeCost
{
    //cleanup touched nodes
    cleanup();
    // auto fut1 = std::async(fillForwardInfo(source)
    // auto fut1 = std::async(std::launch::async,
    //                        [&]() { fillForwardInfo(source); });
    // auto fut2 = std::async(std::launch::async,
    //                        [&]() { fillBackwardInfo(target); });
    // fut1.get();
    // fut2.get();

    fillForwardInfo(source);
    fillBackwardInfo(target);

    return findShortestPathInSettledNodes();
}

auto CHDijkstra::fillForwardInfo(const datastructure::NodeId& source)
    -> void
{
    MinHeap queue(QueueElemCmp);

    queue.push({0, source});

    forward_shortest_distances_[source] = 0;

    while(!queue.empty()) {
        auto [cost_to_current,
              current_node] = queue.top();
        queue.pop();

        auto node_level = graph_.getLevelOf(current_node);

        forward_settled_nodes_.push_back(current_node);

        auto edges = graph_.getForwardEdgesOf(current_node,
                                              node_level + 1);

        for(auto edge : edges) {
            auto weight = edge.getCost();
            auto dest = edge.getDestination();
            auto new_cost = weight + cost_to_current;


            if(new_cost < forward_shortest_distances_[dest]) {
                queue.push({new_cost, dest});
                forward_shortest_distances_[dest] = new_cost;
                forward_touched_nodes_.push_back(current_node);
            }
        }
    }

    std::sort(std::begin(forward_settled_nodes_),
              std::end(forward_settled_nodes_));
}

auto CHDijkstra::fillBackwardInfo(const datastructure::NodeId& target)
    -> void
{
    MinHeap queue(QueueElemCmp);

    queue.push({0, target});

    backward_shortest_distances_[target] = 0;

    while(!queue.empty()) {
        auto [cost_to_current,
              current_node] = queue.top();
        queue.pop();

        auto node_level = graph_.getLevelOf(current_node);

        backward_settled_nodes_.push_back(current_node);

        auto edges = graph_.getBackwardEdgesOf(current_node,
                                               node_level + 1);

        for(auto edge : edges) {
            auto weight = edge.getCost();
            auto dest = edge.getDestination();
            auto new_cost = weight + cost_to_current;


            if(new_cost < backward_shortest_distances_[dest]) {
                queue.push({new_cost, dest});
                backward_shortest_distances_[dest] = new_cost;
                backward_touched_nodes_.push_back(current_node);
            }
        }
    }

    std::sort(std::begin(backward_settled_nodes_),
              std::end(backward_settled_nodes_));
}

auto CHDijkstra::cleanup()
    -> void
{
    for(auto&& idx : backward_touched_nodes_) {
        backward_shortest_distances_[idx] =
            std::numeric_limits<EdgeCost>::max();
    }
    backward_settled_nodes_.clear();
    backward_touched_nodes_.clear();

    for(auto&& idx : forward_touched_nodes_) {
        forward_shortest_distances_[idx] =
            std::numeric_limits<EdgeCost>::max();
    }
    forward_settled_nodes_.clear();
    forward_touched_nodes_.clear();
}

auto CHDijkstra::findShortestPathInSettledNodes()
    -> datastructure::EdgeCost
{
    std::vector<NodeId> common_nodes;
    std::set_intersection(std::cbegin(forward_settled_nodes_),
                          std::cend(forward_settled_nodes_),
                          std::cbegin(backward_settled_nodes_),
                          std::cend(backward_settled_nodes_),
                          std::back_inserter(common_nodes));

    return std::accumulate(
        std::cbegin(common_nodes),
        std::cend(common_nodes),
        std::numeric_limits<EdgeCost>::max(),
        [&](auto&& current, auto&& current_common_node) {
            auto up_cost = forward_shortest_distances_[current_common_node];
            auto down_cost = backward_shortest_distances_[current_common_node];
            return std::min(up_cost + down_cost, current);
        });
}
