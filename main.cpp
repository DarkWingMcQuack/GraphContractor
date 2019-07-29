#include <Graph.hpp>
#include <fmt/core.h>



auto main() -> int
{
    auto graph = datastructure::readFromAllreadyContractedFile("/home/lukas/Downloads/toy.fmi").value();

    for(auto edge : graph.getForwardEdgesOf(1)) {
        fmt::print("({},{}) ", edge.getCost(), edge.getDestination());
    }
    fmt::print("\n");
}
