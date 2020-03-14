#include <DijkstraQueue.hpp>
#include <Graph.hpp>
#include <GraphEssentials.hpp>
#include <MultiTargetDijkstra.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>

using datastructure::Graph;
using datastructure::NodeId;
using datastructure::Edge;
using datastructure::Distance;
using datastructure::NodeLevel;
using pathfinding::MultiTargetDijkstra;


MultiTargetDijkstra::MultiTargetDijkstra(const datastructure::Graph& graph)
    : graph_(graph),
      settled_(graph_.getNumberOfNodes(), false),
      shortest_distances_(graph_.getNumberOfNodes(),
                          std::numeric_limits<Distance>::max()) {}

auto MultiTargetDijkstra::shortestDistanceFromTo(const NodeId& source,
                                                 const std::vector<NodeId>& targets)
    -> std::vector<Distance>
{
    if(source != last_source_) {
        last_source_ = source;
        cleanup();
        queue_.emplace(0, source);
        shortest_distances_[source] = 0;
        touched_nodes_.push_back(source);
    }

    //count how many targets are already settled
    auto settled_target_count =
        std::count_if(std::cbegin(targets),
                      std::cend(targets),
                      [&](auto&& target) {
                          return settled_[target];
                      });


    //while queue not empty and not all targets sattled
    while(!queue_.empty()
          && settled_target_count < targets.size()) {

        auto [cost_to_current, current_node] = queue_.top();

        //check if current node is a target and if it was not
        //settled until now
        if(std::find(std::cbegin(targets),
                     std::cend(targets),
                     current_node)
               != std::cend(targets)
           && !settled_[current_node]
           && ++settled_target_count == targets.size()) {
            break;
        }

        queue_.pop();

        settled_[current_node] = true;

        auto edges = graph_.getForwardEdgesOf(current_node);

        //visit all the neigbors
        for(auto edge : edges) {
            //calculate new new possible shortest distance
            auto weight = edge.getCost();
            auto dest = edge.getDestination();
            auto new_cost = weight + cost_to_current;


            //update shortest distances
            if(new_cost < shortest_distances_[dest]) {
                queue_.emplace(new_cost, dest);
                shortest_distances_[dest] = new_cost;
                touched_nodes_.push_back(dest);
            }
        }
    }

    //get the shortest path distances of all the targets
    std::vector<Distance> costs;
    costs.reserve(targets.size());

    std::transform(std::cbegin(targets),
                   std::cend(targets),
                   std::back_inserter(costs),
                   [&](auto&& target) {
                       return shortest_distances_[target];
                   });

    return costs;
}


auto MultiTargetDijkstra::shortestDistanceFromTo(const datastructure::NodeId& source,
                                                 const datastructure::NodeId& target)
    -> datastructure::Distance
{
    if(source != last_source_) {
        last_source_ = source;
        cleanup();
        queue_.emplace(0, source);
        shortest_distances_[source] = 0;
        touched_nodes_.push_back(source);
    } else if(settled_[target]) {
        return shortest_distances_[target];
    }

    while(!queue_.empty()) {

        //get next element from queue
        auto [cost_to_current,
              current_node] = queue_.top();

        if(current_node == target) {
            return cost_to_current;
        }

        queue_.pop();

        //add the node to settled nodes
        settled_[current_node] = true;

        //get all edges
        auto edges = graph_.getForwardEdgesOf(current_node);

        //iterate over all neigbours of current_node
        for(auto edge : edges) {
            auto weight = edge.getCost();
            auto dest = edge.getDestination();
            auto new_cost = weight + cost_to_current;


            if(new_cost < shortest_distances_[dest]) {
                queue_.emplace(new_cost, dest);
                shortest_distances_[dest] = new_cost;
                touched_nodes_.push_back(dest);
            }
        }
    }

    return shortest_distances_[target];
}


auto MultiTargetDijkstra::shortestDistanceForContracion(const datastructure::NodeId& source,
                                                        const datastructure::NodeId& target,
                                                        std::size_t cost_limit)
    -> datastructure::Distance
{
    if(source != last_source_) {
        last_source_ = source;
        cleanup();
        queue_.emplace(0, source);
        shortest_distances_[source] = 0;
        touched_nodes_.push_back(source);
    } else if(settled_[target]) {
        return shortest_distances_[target];
    }

    while(!queue_.empty()) {

        //get next element from queue
        auto [cost_to_current,
              current_node] = queue_.top();

        if(cost_to_current >= cost_limit) {
            return cost_limit;
        }

        if(current_node == target) {
            return cost_to_current;
        }

        queue_.pop();

        //add the node to settled nodes
        settled_[current_node] = true;

        //get all edges
        auto edges = graph_.getForwardEdgesOf(current_node);

        //iterate over all neigbours of current_node
        for(auto edge : edges) {
            auto weight = edge.getCost();
            auto dest = edge.getDestination();
            auto new_cost = weight + cost_to_current;

            if(new_cost < shortest_distances_[dest]) {
                queue_.emplace(new_cost, dest);
                shortest_distances_[dest] = new_cost;
                touched_nodes_.push_back(dest);
            }
        }
    }

    return shortest_distances_[target];
}

auto MultiTargetDijkstra::cleanup()
    -> void
{
    for(auto&& idx : touched_nodes_) {
        shortest_distances_[idx] =
            std::numeric_limits<Distance>::max();
        settled_[idx] = false;
    }
    touched_nodes_.clear();

    MinHeap new_queue;
    queue_ = std::move(new_queue);
}
