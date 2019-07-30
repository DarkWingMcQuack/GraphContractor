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
    auto graph = datastructure::readFromNonContractedFile("/home/lukas/Downloads/germany.fmi").value();
    fmt::print("graph build in: {}s\n", t.elapsed());

    pathfinding::MultiTargetDijkstra pathfinder{graph};

    t.reset();
    auto distance = pathfinder.shortestDistanceFromTo(8371825,
                                                      {16743651,
                                                       16743652,
                                                       16743653,
                                                       16743654,
                                                       16743655,
                                                       16743656,
                                                       16743657,
                                                       16743658,
                                                       16743659,
                                                       16743660});




    fmt::print("calculated distance in: {}s\n", t.elapsed());
    std::vector should{648681l, 649433l, 666379l, 648777l, 649372l, 649304l, 648885l, 649227l, 649163l, 648996l};

    fmt::print("distances correct? {}\n", (should == distance));
    fmt::print("real:\t{}\n", distance);
    fmt::print("should:\t{}\n", should);
}
