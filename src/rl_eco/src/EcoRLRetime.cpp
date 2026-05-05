/*
Routines to perform retiming.
*/
#include "ord/OpenRoad.hh"
#include "rl_eco/EcoTypes.h" 
#include "rl_eco/EcoRLEnvironment.h"


#include "rl_eco/EcoTypes.h" //for path info
#include "rl_eco/EcoDesignManager.h"

#include "odb/db.h"
#include "odb/dbTypes.h"

#include "sta/Sta.hh"
#include "sta/Network.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/Graph.hh"
#include "sta/Sdc.hh"
#include "sta/Units.hh"

#include "grt/GlobalRouter.h"
#include "rsz/Resizer.hh"

#include "utl/Logger.h"

#include "sta/VerilogWriter.hh"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>
#include <set>
#include <map>
#include <string>
#include <iostream>
#include <sstream>

//#define DEBUG_RETIME 
using namespace odb;
using namespace sta;

namespace eco {

  

  EcoDesignManager::MoveResult EcoDesignManager::previewBackwardRetimeMove(
									     odb::dbInst* flop,
									     odb::dbITerm* d,
									     odb::dbITerm* q
									     )
  {
    MoveResult result;
    //get the matching spares
    std::vector<std::shared_ptr<SpareCell> > dff_spares;
    getUnusedMatchingFlopSpares(flop,dff_spares);
    
#ifdef DEBUG_RETIME
    printf("Backward retime for flop %s (type %s) d %s q%s\n",
	   flop -> getName().c_str(),
	   flop -> getMaster() -> getName().c_str(),
	   d -> getMTerm() -> getName().c_str(),
	   q -> getMTerm() -> getName().c_str());
    odb::dbObject* driver_object = d -> getNet() -> getFirstDriverTerm();
    if (driver_object && driver_object -> getObjectType() == odb::dbITermObj){
      odb::dbITerm* driver_iterm = static_cast<odb::dbITerm*>(driver_object);
      printf("D input driver %s/%s (on gate of type %s)\n",
	     driver_iterm ->getInst() -> getName().c_str(),
	     driver_iterm -> getMTerm() -> getName().c_str(),
	     driver_iterm -> getInst() -> getMaster() -> getName().c_str());      
    }
    else if (driver_object && driver_object -> getObjectType() == odb::dbBTermObj){
      printf("D input driven by pad !\n");
    }

    printf("Flop %s Matches %d\n",
	   flop -> getMaster() -> getName().c_str(),
	   dff_spares.size());
#endif
    static int debug;
    debug++;
    std::vector<std::pair<odb::dbITerm*, std::shared_ptr<SpareCell> > > backward_retime_assignment;
    //make a feasible retiming assignment, if possible
    if (makeBackwardRetimeAssignment(flop,d,dff_spares,
				     backward_retime_assignment)){
#ifdef DEBUG_RETIME
      printf("D %d Spare cell assignment for input flops\n",debug);
      for (auto p: backward_retime_assignment){
	printf("Fanin ip %s Instance %s (type %s)  Spare Cell Inst %s\n",
	       p.first -> getMTerm() -> getName().c_str(),
	       p.first -> getInst() -> getName().c_str(),
	       p.first -> getInst() -> getMaster() -> getName().c_str(),	       
	       p.second -> instance -> getName().c_str());
	       
      }
#endif
      
      resizer_ -> journalBegin();
      //measure retime move
      result = performBackwardRetimeMove(flop,backward_retime_assignment);
      resizer_ -> journalEnd();
      //free up the spares uses.
      for (auto retime_assignment: backward_retime_assignment){
#ifdef DEBUG_RETIME
	printf("Freeing up spare cell %s\n",
	       retime_assignment.second -> instance -> getName().c_str());
#endif	
	retime_assignment.second -> is_used = false;
      }
      //return the timing numbers
      return result;
    }
    result.error_message = "No feasible spare assignment";
    result.timing_improvement = -0.1;
    result.success = false;
    return result;
  }
  
  
  bool EcoDesignManager::makeBackwardRetimeAssignment(odb::dbInst* dff,
						      odb::dbITerm* d,
						      std::vector<std::shared_ptr<SpareCell> >& dff_spares,
						      std::vector<std::pair<odb::dbITerm*,
						      std::shared_ptr<SpareCell> > >& backward_retime_assignment){

    //get the fanin 1 level
    //get unique fanin pins
    //construct the iterm <-> spare cell correlation.

    if (dff && d && d -> getNet()){
      
      std::vector<odb::dbITerm*> fanin_pins_to_retime;
      if (getFanin1LevelBackwardRetime(d,fanin_pins_to_retime)){
#ifdef DEBUG_RETIME
      printf("MakeBackwardRetimeAssignment: pin %s #fanins %d\n",
	     d -> getName().c_str(),
	     fanin_pins_to_retime.size());
#endif      
      std::map<odb::dbNet*, std::vector<dbITerm*> > unique_net_drivers;
      std::map<odb::dbNet*, std::vector<dbITerm*> >::iterator und_it;
      for (auto fanin_pin: fanin_pins_to_retime){
	odb::dbNet* fanin_pin_net = fanin_pin -> getNet();
#ifdef DEBUG_RETIME
	printf("Fanin inst %s pin %s to retime \n",
	       fanin_pin -> getInst() -> getName().c_str(),
	       fanin_pin -> getName().c_str())
#endif	
	und_it = unique_net_drivers.find(fanin_pin_net);
	if (und_it != unique_net_drivers.end()){
	  (*und_it).second.push_back(fanin_pin);
	}
	else{
	  std::vector<dbITerm*> vec_drvr;
	  vec_drvr.push_back(fanin_pin);
	  unique_net_drivers[fanin_pin_net]=vec_drvr;
	}
      }
      if (dff_spares.size() >= unique_net_drivers.size()){
	for (auto und: unique_net_drivers){
	  for (auto i: und.second){
	    std::shared_ptr<SpareCell> dff_spare = findClosestSpare(i -> getInst(),
								    dff_spares);
	    std::pair<odb::dbITerm*, std::shared_ptr<SpareCell> >
	      retime_assignment(i, dff_spare);
	    dff_spare -> is_used=true;
	    //note that two (or more) iterms might share the same spare cell.
	    //This is to handle case of shared fanin drivers.
	    backward_retime_assignment.push_back(retime_assignment);
	  }
	}
	return true;
      }
      }
    }
    return false;
  }

  
  
  
  bool EcoDesignManager:: identifyRetimingMoves(std::vector<std::tuple<odb::dbInst*,
						odb::dbITerm*, //d input
						odb::dbITerm*, //q output
						bool  //direction of the move
						//true forwards, false backwards
						> >
						& candidates){

  //go through all the flops and identify those with 
  //positive slack on one side and negative slack on the other
  //These are our candidates

  for (odb::dbInst* inst : block_->getInsts()) {

    odb::dbITerm* d=nullptr;
    odb::dbITerm* q=nullptr;
    if (isDFFRS(inst,d,q)){
      sta::Pin* sta_d_pin = resizer_->getDbNetwork()->dbToSta(d);
      odb::dbNet* d_net = d -> getNet();
      if (!(d_net && singletonNet(d_net))){
	continue;
      }
      sta::Pin* sta_q_pin = resizer_->getDbNetwork()->dbToSta(q);            
      double d_slack = sta_ -> pinSlack(sta_d_pin,sta::MinMax::max());
      double q_slack = sta_ -> pinSlack(sta_q_pin,sta::MinMax::max());
      if (d_slack == sta::INF ||
	  q_slack == sta::INF){
	continue;
      }
      sta::Unit *unit = Sta::sta()->units()->timeUnit();
      double d_slack_scaled = unit -> staToUser(d_slack);
      double q_slack_scaled = unit -> staToUser(q_slack);
#ifdef DEBUG_RETIME      
      printf("State %s D slack %.10f Q Slack %.10f\n",
	     inst -> getName().c_str(),
	     d_slack_scaled,
	     q_slack_scaled);
      printf("TNS %.10f\n", initial_tns);
#endif      
      //
      //TODO: sort retiming by gain
      //
      if (d_slack_scaled > 0.0 && q_slack_scaled < 0.0){
	//A forward move, push into fanout
	candidates.push_back(std::tuple<odb::dbInst*, odb::dbITerm*, odb::dbITerm*, bool >
			     (inst,d,q,true));
      }
      else if (q_slack_scaled > 0.0 && d_slack_scaled < 0.0){
	//A backward move, push into fanin
#ifdef DEBUG_RETIME
	printf("Backward move (Q=%s/%s D=%s/%s).  slack %.10f D slack %.10f\n",
	       q -> getInst() -> getName().c_str(),
	       q -> getName().c_str(),
	       
	       d -> getInst() -> getName().c_str(),
	       d -> getName().c_str(),
	       
	       q_slack_scaled,
	       d_slack_scaled);
#endif	
	candidates.push_back(std::tuple<odb::dbInst*, odb::dbITerm*, odb::dbITerm*, bool >
			     (inst,d,q,false));
      }
    }
  }
  if (candidates.size() > 0){
    return true;
  }
  return false;
}

  

  bool EcoDesignManager::performForwardRetime(odb::dbNet* clk_net,
					      odb::dbNet* reset_net,
					      odb::dbNet* set_net,
					      
					      odb::dbITerm* op_pin,
					      std::vector<std::pair<odb::dbITerm*,
					      std::shared_ptr<SpareCell> > >
					      &fanout_op_pins
					      ){

    return false;
  }

  EcoDesignManager::MoveResult EcoDesignManager::performBackwardRetimeMove(
					       odb::dbInst* orig_flop,
					       
					       std::vector<std::pair<odb::dbITerm*,
					       std::shared_ptr<SpareCell> > >
					       &fanin_ip_pins //ok to have duplicate spare cells.
					      ){
#ifdef DEBUG_RETIME
    const std::string before_move_verilog = "retimebefore.v";
    sta::writeVerilog(before_move_verilog.c_str(), true, false, {},
		      dbsta_ -> getDbNetwork());
		      //sta_->network());    
#endif    
    MoveResult     result;
    result.timing_improvement = 0.0;

    double initial_wns,initial_tns, initial_area, initial_wire;    
    capturePreMoveMetrics(initial_wns,initial_tns, initial_area, initial_wire);

    printf("backward retime: Initial tns %.10f\n", initial_tns);
    
    std::map<odb::dbNet*,
	     std::pair< std::vector<odb::dbITerm*>, //destination
			std::shared_ptr<SpareCell> >
	     > net2spare;

    std::map<odb::dbNet*,
	     std::pair< std::vector<odb::dbITerm*>, //destination
			std::shared_ptr<SpareCell> >
	     >::iterator net2spare_it;
    

    //
    //Note that different destinations could
    //be driven by the same driver so we guarantee the uniqueness
    //using the hash. We introduce one new flop per net. But we
    //note that there coule be multiple input pins on that net.
    //
    
    for (auto fanin_ip_iter : fanin_ip_pins){
      odb::dbITerm* ip_pin = fanin_ip_iter.first;
      std::shared_ptr<SpareCell> spare_cell = fanin_ip_iter.second;
      odb::dbNet* ip_pin_net = ip_pin -> getNet();
      if (ip_pin_net){
	int inst_x;
	int inst_y;
	ip_pin -> getInst() ->getLocation(inst_x, inst_y);
	/*
	printf("Ip pin %s location x=%d y= %d  spare cell inst %s cell %s location x=%d y=%d\n",
	       ip_pin -> getMTerm() -> getName().c_str(),
	       inst_x,
	       inst_y,
	       spare_cell -> instance -> getName().c_str(),
	       spare_cell -> master -> getName().c_str(),
	       spare_cell -> x,
	       spare_cell -> y
	       );
	*/     
	net2spare_it = net2spare.find(ip_pin_net);
	if (net2spare_it != net2spare.end()){
	  //multiple net-> pin case, stash additional ip_pin in vector
	  (*net2spare_it).second.first.push_back(ip_pin);
	}
	else{
	  //single net -> pin case
	  std::vector<odb::dbITerm*> vec;
	  vec.push_back(ip_pin);
	  net2spare[ip_pin_net] =
	    std::pair<std::vector<odb::dbITerm*>,
		      std::shared_ptr<SpareCell> >(vec,spare_cell);
	}
      }
    }

    /*
      Short out original flop: d -> q
    */

    odb::dbITerm *orig_flop_clk=nullptr, *orig_flop_r=nullptr,
      *orig_flop_s=nullptr, *orig_flop_d=nullptr, *orig_flop_q=nullptr, *orig_flop_qn=nullptr;
    
    getFlopPins(orig_flop,
		orig_flop_d,
		orig_flop_q,
		orig_flop_qn,		
		orig_flop_clk,
		orig_flop_r,
		orig_flop_s
		);

    assert(orig_flop_d && orig_flop_d -> getNet());
    assert(orig_flop_q && orig_flop_q -> getNet());    
    assert(orig_flop_clk && orig_flop_clk -> getNet());

    odb::dbNet* orig_flop_q_net = orig_flop_q -> getNet();    
    odb::dbNet* orig_flop_d_net = orig_flop_d -> getNet();
    assert(singletonNet(orig_flop_d_net));

    odb::dbNet* orig_flop_r_net = orig_flop_r ? orig_flop_r -> getNet(): nullptr;
    odb::dbNet* orig_flop_s_net = orig_flop_s ? orig_flop_s -> getNet(): nullptr;
    odb::dbNet* orig_flop_clk_net = orig_flop_clk ? orig_flop_clk -> getNet(): nullptr;    
    
    //hook up the destinations
    for (auto net2spare_iter: net2spare){

      //hooked to fanin net, goes to d of new flop
      odb::dbNet* orig_ip_net = net2spare_iter.first; 

      //disconnect the destinations
      for (auto d: net2spare_iter.second.first){
	d -> disconnect();
      }

      //make a new net for the flop output
      odb::dbNet* new_flop_op_net = mkFlatNet();

      //connect new net to all the destinations
      for (auto d: net2spare_iter.second.first){
	d -> connect(new_flop_op_net);
      }

      //Set up the new flop
      odb::dbInst* new_flop = net2spare_iter.second.second -> instance;
      net2spare_iter.second.second -> is_used=true; //mark spare flop as used.

      //get the new flop pins and wire up the new flop
      //d -> original net driving destination
      //r,s,clk as per original.
      
      odb::dbITerm* new_flop_clk=nullptr, *new_flop_r=nullptr,
	*new_flop_s=nullptr, *new_flop_d=nullptr, *new_flop_q=nullptr, *new_flop_qn;
      
      getFlopPins(new_flop,
		  new_flop_d,
		  new_flop_q,
		  new_flop_qn,		  
		  new_flop_clk,
		  new_flop_r,
		  new_flop_s);
      if (orig_flop_r_net && new_flop_r){
	new_flop_r -> connect(orig_flop_r_net);
      }
      if (orig_flop_clk_net && new_flop_clk){
	new_flop_clk -> connect(orig_flop_clk_net);
      }
      if (orig_flop_s_net && new_flop_s){
	new_flop_s -> connect(orig_flop_s_net);
      }
      if (new_flop_q && new_flop_op_net){
	new_flop_q -> connect(new_flop_op_net);
      }
      if (new_flop_d && orig_ip_net){
	new_flop_d -> connect(orig_ip_net);
      }
    }

    //free up the original flop
    //and short its d to q.
    //Because we require the d input net to have 
    if (orig_flop_d_net && orig_flop_q_net){
      orig_flop_q  -> disconnect();
      odb::dbObject* driver_object = orig_flop_d_net -> getFirstDriverTerm();
      orig_flop_d -> disconnect();
      if (driver_object && driver_object -> getObjectType() == odb::dbITermObj){
	odb::dbITerm* driver_iterm = static_cast<odb::dbITerm*>(driver_object);
	driver_iterm -> disconnect();
	driver_iterm -> connect(orig_flop_q_net);
      }
    }
    
    if (orig_flop_clk && orig_flop_clk_net){
      orig_flop_clk -> disconnect();
    }
    if (orig_flop_r && orig_flop_r_net){
      orig_flop_r -> disconnect();
    }
    if (orig_flop_s && orig_flop_s_net){
      orig_flop_s -> disconnect();
    }
    std::stringstream retime_move ;
    retime_move << "Retime " << "Flop " << orig_flop->getName() << "."; 
    result = calculateMoveImpact(initial_wns,initial_tns, initial_area, initial_wire,retime_move.str());


#ifdef DEBUG_RETIME    
    const std::string after_move_verilog = "retimeafter.v";
    sta::writeVerilog(after_move_verilog.c_str(), true, false, {},
		      dbsta_ -> getDbNetwork());
		      //sta_->network());
#endif    
    return result;
  }
  
 
}
