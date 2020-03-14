#include <CHDijkstra.hpp>
#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <MultiTargetDijkstra.hpp>
#include <Timer.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iostream>

using namespace datastructure;
using namespace pathfinding;


auto main() -> int
{
    Timer t;
    // fmt::print("read CH Graph...\n");
    auto ch_graph = readFromAllreadyContractedFile("/home/lukas/Projects/GraphContractor/data/stgtregbz_ch.fmi").value();
    fmt::print("read non-CH Graph...\n");
    auto graph = readFromNonContractedFile("/home/lukas/Projects/GraphContractor/data/stgtregbz.fmi").value();
    fmt::print("graph build in: {}s\n", t.elapsed());

    // GraphContractor contractor{graph};

    t.reset();
    fmt::print("contracting graph ...\n");
    // contractor.contractGraph();
    fmt::print("graph contracted in: {}s\n", t.elapsed());
    // auto own_ch_graph = std::move(contractor.getGraph());


    NodeId from;
    NodeId to;
    CHDijkstra ch_pathfinder{ch_graph};
    // CHDijkstra own_ch_pathfinder{own_ch_graph};
    MultiTargetDijkstra pathfinder{graph};
    while(true) {

        fmt::print("from: ");
        std::cin >> from;
        fmt::print("to: ");
        std::cin >> to;


        t.reset();
        auto ch_distance = ch_pathfinder.shortestDistanceFromTo(from,
                                                                to);


        auto ch_time = t.elapsed();
        fmt::print("ch_distance: {}s\n", ch_distance);
        fmt::print("calculated ch_distance in: {}s\n", ch_time);

        // t.reset();
        // auto own_ch_distance = own_ch_pathfinder.shortestDistanceFromTo(from,
        //                                                                 to);
        // auto own_ch_time = t.elapsed();
        // fmt::print("own_ch_distance:\t{}\n", own_ch_distance);
        // fmt::print("calculated Own Ch distance in: {}s\n", own_ch_time);


        t.reset();
        auto distance = pathfinder.shortestDistanceFromTo(from, to);
        auto normal_time = t.elapsed();
        fmt::print("normal dijkstra distance:\t{}\n", distance);
        fmt::print("calculated normal dijkstra distances in: {}s\n", normal_time);

        // fmt::print("speedup:\t{}\n", normal_time / own_ch_time);
    }
}
