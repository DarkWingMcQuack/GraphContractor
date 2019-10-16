#include "GraphContractor.hpp"
#include <CHDijkstra.hpp>
#include <Graph.hpp>
#include <MultiTargetDijkstra.hpp>
#include <chrono>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iostream>

using namespace datastructure;
using namespace pathfinding;

class Timer
{
public:
    Timer()
        : beg_(clock_::now()) {}
    void reset()
    {
        beg_ = clock_::now();
    }
    double elapsed() const
    {
        return std::chrono::duration_cast<second_>(clock_::now() - beg_).count();
    }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1>> second_;
    std::chrono::time_point<clock_> beg_;
};

auto main() -> int
{
    Timer t;
    fmt::print("read CH Graph...\n");
    auto ch_graph = readFromAllreadyContractedFile("/home/lukas/Downloads/stgtregbz_ch.fmi").value();
    fmt::print("read non-CH Graph...\n");
    auto graph = readFromNonContractedFile("/home/lukas/Downloads/stgtregbz.fmi").value();
    fmt::print("graph build in: {}s\n", t.elapsed());

    GraphContractor contractor{graph};

    t.reset();
    fmt::print("contracting graph ...\n");
    contractor.contractGraph();
    fmt::print("graph contracted in: {}s\n", t.elapsed());
    auto own_ch_graph = std::move(contractor.getGraph());


    NodeId from;
    NodeId to;
    CHDijkstra ch_pathfinder{ch_graph};
    CHDijkstra own_ch_pathfinder{own_ch_graph};
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
        fmt::print("calculated ch_distance in: {}s\n", ch_time);
        t.reset();
        auto own_ch_distance = own_ch_pathfinder.shortestDistanceFromTo(from,
                                                                        to);



        auto own_ch_time = t.elapsed();
        fmt::print("calculated own_ch_distance in: {}s\n", ch_time);
        t.reset();


        auto distance = pathfinder.shortestDistanceFromTo(from, to);
        auto normal_time = t.elapsed();
        t.reset();
        auto distances = pathfinder.shortestDistanceFromTo(from, {to, ++to, ++to, ++to, ++to, ++to, ++to, ++to, ++to});
        auto multi_time = t.elapsed();
        fmt::print("calculated distances in: {}s\n", normal_time);
        fmt::print("calculated multi distances in: {}s\n", multi_time);
        fmt::print("ch_distance:\t{}\n", ch_distance);
        fmt::print("own_ch_distance:\t{}\n", own_ch_distance);
        fmt::print("distance:\t{}\n", distance);
        fmt::print("distances:\t{}\n", distances);
        fmt::print("speedup:\t{}\n", normal_time / ch_time);
    }
}
