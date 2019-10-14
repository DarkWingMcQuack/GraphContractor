include(ExternalProject)
include(GNUInstallDirs)

set(CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  -DCMAKE_BUILD_TYPE=Release)

ExternalProject_Add(span-project
  PREFIX deps/span
  DOWNLOAD_DIR ${CMAKE_BINARY_DIR}/downloads
  GIT_REPOSITORY https://github.com/tcbrindle/span
  TIMEOUT 10
  UPDATE_COMMAND ""
  CMAKE_ARGS ${CMAKE_ARGS}
  # Overwtire build and install commands to force Release build on MSVC.
  BUILD_COMMAND cmake --build <BINARY_DIR> --config Release
  INSTALL_COMMAND ""#cmake --build <BINARY_DIR> --config Release --target install
  )

ExternalProject_Get_Property(span-project INSTALL_DIR)
set(SPAN_INCLUDE_DIR ${INSTALL_DIR}/include)
add_library(span STATIC IMPORTED)
set_property(TARGET span PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${SPAN_INCLUDE_DIR})

unset(INSTALL_DIR)
unset(CMAKE_ARGS)
