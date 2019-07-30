#include <Graph.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>



auto main() -> int
{
    auto graph = datastructure::readFromAllreadyContractedFile("/home/lukas/Downloads/stgtregbz_ch.fmi").value();

    fmt::print("forward:\n");
    fmt::print("{}\n", graph.getForwardOffsetArray());
    fmt::print("backward:\n");
    fmt::print("{}\n", graph.getBackwardOffsetArray());

    for(auto edge : graph.getForwardEdgesOf(1)) {
        fmt::print("({},{}) ", edge.getCost(), edge.getDestination());
    }
    fmt::print("\n");
}
