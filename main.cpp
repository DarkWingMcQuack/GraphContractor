#include <BidirectionalDijkstra.hpp>
#include <Graph.hpp>
#include <MultiTargetDijkstra.hpp>
#include <chrono>
#include <fmt/core.h>
#include <fmt/ranges.h>

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
    auto ch_graph = datastructure::readFromAllreadyContractedFile("/home/lukas/Downloads/stgtregbz_ch.fmi").value();
    fmt::print("read non-CH Graph...\n");
    auto graph = datastructure::readFromNonContractedFile("/home/lukas/Downloads/stgtregbz.fmi").value();
    fmt::print("graph build in: {}s\n", t.elapsed());

    pathfinding::BidirectionalDijkstra ch_pathfinder{ch_graph};
    pathfinding::MultiTargetDijkstra pathfinder{ch_graph};

    t.reset();
    auto ch_distance = ch_pathfinder.shortestDistanceFromTo(12221,
                                                            218323);



    auto ch_time = t.elapsed();
    fmt::print("calculated ch_distance in: {}s\n", ch_time);
    t.reset();


    auto distance = pathfinder.shortestDistanceFromTo(12221, {218323});
    auto normal_time = t.elapsed();
    fmt::print("calculated distance in: {}s\n", normal_time);
    fmt::print("ch_distance:\t{}\n", ch_distance);
    fmt::print("distance:\t{}\n", distance[0]);
    fmt::print("speedup:\t{}\n", normal_time / ch_time);
}
