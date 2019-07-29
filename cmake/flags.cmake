#debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -fmax-errors=1 -O0 -g3 -ggdb")

#release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -flto -O3 -lboost_system -march=native -fstack-protector-strong")
