## SPDX-License-Identifier: BSD-3-Clause
## Copyright (c) 2025, The OpenROAD Authors

# Define the rl_eco library target here (before subdirectories)
add_library(rl_eco STATIC)  # Or SHARED if you prefer a .so

# Set properties for the target (e.g., C++ standard, includes)
target_include_directories(rl_eco
  PUBLIC
  ../include
  ${PROJECT_SOURCE_DIR}/include
  PRIVATE
    # Unfortunate side-effect of swig include StaTcl.i
    ${OPENSTA_HOME}/include/sta
    ${OPENSTA_HOME}
)

target_compile_features(rl_eco PUBLIC cxx_std_17)  # Or cxx_std_20 if needed

# Link dependencies (LibTorch, OpenROAD internals)
target_link_libraries(rl_eco
    PRIVATE
    ${TORCH_LIBRARIES}  # From find_package(Torch)
    OpenSTA
    odb
    utl_lib
    dbSta_lib  
)

# Now add the src/ subdirectory (where sources will be added to the target)
add_subdirectory(src)
