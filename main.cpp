#include <CHDijkstra.hpp>
#include <CLI/CLI.hpp>
#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <MultiTargetDijkstra.hpp>
#include <Timer.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iostream>

using datastructure::readFromNonContractedFile;
using datastructure::GraphContractor;
using datastructure::NodeId;
using datastructure::NodeId;
using pathfinding::CHDijkstra;
using pathfinding::MultiTargetDijkstra;


auto main(int argc, char **argv) -> int
{
    CLI::App app{"GraphContractor - Schein für ALgEng"};
    std::string graph_file;

    app.add_option("-f,--file",
                   graph_file,
                   "Graph which will be contracted")
        ->required()
        ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    Timer t;
    fmt::print("read non-CH Graph...\n");
    auto graph = readFromNonContractedFile(graph_file).value();
    fmt::print("graph build in: {}s\n", t.elapsed());

    GraphContractor contractor{graph};

    t.reset();
    contractor.contractGraph();
    fmt::print("graph contracted in: {}s\n", t.elapsed());
    auto own_ch_graph = std::move(contractor.getGraph());

    NodeId from;
    NodeId to;
    // CHDijkstra ch_pathfinder{ch_graph};
    CHDijkstra own_ch_pathfinder{own_ch_graph};
    MultiTargetDijkstra pathfinder{graph};
    while(true) {

        fmt::print("from: ");
        std::cin >> from;
        fmt::print("to: ");
        std::cin >> to;

        t.reset();
        auto own_ch_distance = own_ch_pathfinder.shortestDistanceFromTo(from,
                                                                        to);
        auto own_ch_time = t.elapsed();
        fmt::print("CH distance:\t{}\n", own_ch_distance);
        fmt::print("calculated CH distance in: {}s\n", own_ch_time);

        t.reset();
        auto distance = pathfinder.shortestDistanceFromTo(from, to);
        auto normal_time = t.elapsed();
        fmt::print("normal dijkstra distance:\t{}\n", distance);
        fmt::print("calculated normal dijkstra distances in: {}s\n", normal_time);

        fmt::print("speedup:\t{}\n", normal_time / own_ch_time);
    }
}
