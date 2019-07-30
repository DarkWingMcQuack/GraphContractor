#include <Graph.hpp>
#include <GraphEssentials.hpp>
#include <MultiTargetDijkstra.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>

using datastructure::Graph;
using datastructure::NodeId;
using datastructure::Edge;
using datastructure::EdgeCost;
using datastructure::NodeLevel;
using pathfinding::MultiTargetDijkstra;


MultiTargetDijkstra::MultiTargetDijkstra(const datastructure::Graph& graph)
    : graph_(graph),
      shortest_distances_(graph_.getNumberOfNodes(),
                          std::numeric_limits<EdgeCost>::max()) {}

auto MultiTargetDijkstra::shortestDistanceFromTo(const NodeId& source,
                                                 const std::vector<NodeId>& targets)
    -> std::vector<EdgeCost>
{
    //cleanup touched nodes
    cleanup();

    MinHeap queue(QueueElemCmp);
    queue.push({0, source});

    std::vector costs(targets.size(),
                      std::numeric_limits<EdgeCost>::max());

    auto keep_inserting = true;

    shortest_distances_[source] = 0;

    while(!queue.empty()) {
        auto [cost_to_current, current_node] = queue.top();
        queue.pop();

        settled_nodes_.push_back(current_node);

        auto edges = graph_.getForwardEdgesOf(current_node);

        for(auto edge : edges) {
            auto weight = edge.getCost();
            auto dest = edge.getDestination();
            auto new_cost = weight + cost_to_current;


            if(new_cost < shortest_distances_[dest]) {
                if(keep_inserting) {
                    queue.push({new_cost, dest});
                }
                shortest_distances_[dest] = new_cost;
            }

            for(int i{0}; i < targets.size(); i++) {
                const auto target = targets[i];
                costs[i] = std::min(shortest_distances_[target],
                                    costs[i]);
            }

            keep_inserting =
                std::any_of(
                    std::cbegin(costs),
                    std::cend(costs),
                    [](auto&& c) {
                        return c == std::numeric_limits<EdgeCost>::max();
                    });
        }
    }

    return costs;
}


auto MultiTargetDijkstra::cleanup()
    -> void
{
    for(auto&& idx : touched_nodes_) {
        shortest_distances_[idx] = std::numeric_limits<EdgeCost>::max();
    }
}
