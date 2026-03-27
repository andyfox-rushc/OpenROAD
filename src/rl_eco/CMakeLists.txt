# src/rl_eco/CMakeLists.txt

include("openroad")

swig_lib(NAME      rl_eco
         NAMESPACE eco
         I_FILE    src/RlEco.i
         SCRIPTS   src/RlEco.tcl
)

target_sources(rl_eco
  PRIVATE
  src/EcoDesignManager.cpp
  src/EcoRLEnvironment.cpp
  src/EcoQLearningTrainer.cpp
  src/DQNAgent.cpp
  src/EcoNeuralNetwork.cpp
  src/RlEco.cpp
  src/MakeRlEco.cpp
  )
  

target_include_directories(rl_eco
  PUBLIC
   include
  PRIVATE
  ${ODB_HOME}/include
  ${OPENSTA_HOME}
  utl
)

target_link_libraries(rl_eco
  PRIVATE
    odb
    OpenSTA
    dbSta
    rsz
    grt
    utl
)

# Register the module initialization
target_compile_definitions(rl_eco
  PRIVATE
    ECO_HOME="${CMAKE_CURRENT_SOURCE_DIR}"
)

# Optional: for debuggingmessage(STATUS "RL_ECO sources: ${CMAKE_CURRENT_SOURCE_DIR}")

