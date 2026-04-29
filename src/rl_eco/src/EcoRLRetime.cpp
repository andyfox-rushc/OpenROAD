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



#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>
#include <set>
#include <map>
#include <string>
#include <iostream>
#include <sstream>

using namespace odb;
using namespace sta;

namespace eco {

  

  EcoDesignManager::MoveResult EcoDesignManager::previewBackwardRetimingMove(
									     odb::dbInst* flop,
									     odb::dbITerm* d,
									     odb::dbITerm* q
									     )
  {
    MoveResult result;
    return result;
    /*
    std::vector<std::shared_ptr<SpareCell> > spares;
    
    resizer_ -> journalBegin();
    MoveResult result = performBackwardRetimeMove(flop,d,q,spares);
    resizer_ -> journalEnd();
    //free up the spares uses.
    for (auto spare: spares){
      spare -> is_used = false;
    }
    return result;
    */
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
    odb::dbMaster* master = inst -> getMaster();
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
      printf("State %s D slack %.10f Q Slack %.10f\n",
	     inst -> getName().c_str(),
	     d_slack_scaled,
	     q_slack_scaled);
      
      if (d_slack_scaled > 0.0 && q_slack_scaled < 0.0){
	//A forward move, push into fanout
	candidates.push_back(std::tuple<odb::dbInst*, odb::dbITerm*, odb::dbITerm*, bool >
			     (inst,d,q,true));
      }
      else if (q_slack_scaled > 0.0 && d_slack_scaled < 0.0){
	//A backward move, push into fanin
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

  EcoDesignManager::MoveResult EcoDesignManager::performBackwardRetime(
					       odb::dbInst* orig_flop,
					       
					       std::vector<std::pair<odb::dbITerm*,
					       std::shared_ptr<SpareCell> > >
					       &fanin_ip_pins //ok to have duplicate spare cells.
					      ){
    MoveResult     result;
    result.timing_improvement = 0.0;

    double initial_tns, initial_area, initial_wire;    
    capturePreMoveMetrics(initial_tns, initial_area, initial_wire);    
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
    result = calculateMoveImpact(initial_tns, initial_area, initial_wire);
    return result;
  }
  
 
}
