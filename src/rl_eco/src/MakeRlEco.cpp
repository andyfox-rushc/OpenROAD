
#include "ord/OpenRoad.hh"
#include "rl_eco/MakeRlEco.h"
#include "rl_eco/RlEco.h"
#include <tcl.h>

// SWIG-generated init function - OpenROAD style
extern "C" {
extern int Rl_eco_Init(Tcl_Interp* interp);
}
 
namespace ord {

static eco::RlEco* rl_eco = nullptr;
 
  eco::RlEco* getRlEco()
{
  if (!rl_eco) {
    rl_eco = new eco::RlEco();
  }
  if (!rl_eco -> successfullyInit()){
    ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
    rl_eco -> init(openroad);
  }
  
  return rl_eco;
}

void initRlEco(OpenRoad* openroad)
{
  printf("Init rl eco in MakeRlEco.cpp\n");
  // Initialize the C++ object
  getRlEco()->init(openroad);
  
  // Initialize the SWIG/TCL interface
  Tcl_Interp* tcl_interp = openroad->tclInterp();

  if (Rl_eco_Init(tcl_interp) != TCL_OK) {
    printf("Failed to initialize rl_eco TCL interface\n");
  } else {
    printf("Successfully initialized rl_eco TCL interface\n");
    printf("Pointer for eco_rl is %p\n",
	   getRlEco());
  }
  

}
 
void deleteRlEco(eco::RlEco* eco)
{
  delete eco;
}
 
} // namespace ord
