## SPDX-License-Identifier: BSD-3-Clause
## Copyright (c) 2025, The OpenROAD Authors

# Add sources to the existing rl_eco target (defined in parent)
target_sources(rl_eco
  PRIVATE
  EcoEnvironment.cpp
  EcoAgent.cpp
    # Add all your .cpp files here
    # If you have mains for executables, see below
)

target_include_directories(rl_eco
  PUBLIC
    ../include
    ${PROJECT_SOURCE_DIR}/include

  PRIVATE
    # Unfortunate side-effect of swig include StaTcl.i
    ${OPENSTA_HOME}/include/sta
    ${OPENSTA_HOME}
)

# Optional: If you want standalone executables (e.g., for training/inference)
#add_executable(train_eco main_train.cpp)
#target_link_libraries(train_eco PRIVATE rl_eco)  # Links to your library

#add_executable(infer_eco main_infer.cpp)
#target_link_libraries(infer_eco PRIVATE rl_eco)

# If you have more subdirs or custom compile options, add them here
