# GraphContractor

## Build

#### Clang
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DUSE_CLANG=1 ..
make -j8
```

#### GCC
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DUSE_CLANG=0 ..
make -j8
```

## Useage

```
./GraphContractor --file <path to graph file>
```
