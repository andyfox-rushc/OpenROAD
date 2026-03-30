%module rl_eco

%{
#include "rl_eco/RlEco.h"
#include "ord/OpenRoad.hh"

namespace ord {
eco::RlEco* getRlEco();
OpenRoad* getOpenRoad(); 
}
%}

// Include exception handling
%include "../../Exception.i"

// Include std::vector support for setHiddenLayers
%include "std_vector.i"
%template(IntVector) std::vector<int>;
%template(FloatVector) std::vector<float>;

// Include the main header. Swig will generate
// tcl for every public member.
%include "rl_eco/RlEco.h"


// Tell SWIG about the getter function
namespace ord {
  eco::RlEco* getRlEco();
}

%extend eco::RlEco {
  void init(ord::OpenRoad* openroad) {
    printf("Extended self init with open road\n");
    self->init(openroad);
 }

  
}
