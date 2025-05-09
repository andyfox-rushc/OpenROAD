# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

add_library(defin
    definNet.cpp 
    definSNet.cpp 
    definComponent.cpp 
    definComponentMaskShift.cpp
    definVia.cpp 
    definPin.cpp 
    definBlockage.cpp 
    definFill.cpp 
    definTracks.cpp 
    definGCell.cpp 
    definRow.cpp 
    definRegion.cpp 
    definGroup.cpp 
    definNonDefaultRule.cpp 
    definReader.cpp 
    definBase.cpp 
    create_box.cpp 
    defin.cpp 
    definPolygon.cpp 
    definPropDefs.cpp 
    definPinProps.cpp 
)

target_include_directories(defin
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${TCL_INCLUDE_PATH}
)

target_link_libraries(defin
    PUBLIC
        db
        zutil
        def
        defzlib
        utl_lib
)

set_target_properties(defin
  PROPERTIES
    # python requirement
    POSITION_INDEPENDENT_CODE ON
)

