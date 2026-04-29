/*
Utility routines for interacting with openroad database
used by EcoDesignManager
*/
#include "ord/OpenRoad.hh"
#include "rl_eco/EcoTypes.h" 
#include "rl_eco/EcoRLEnvironment.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>
#include <set>
#include <map>
#include <string>
#include <iostream>
#include <sstream>

namespace eco {

  //
  //Is this a gate we can push a flop through ?
  //TODO: remove hardcoded names, look for logic.
  //
  bool EcoDesignManager::RetimeableGate(odb::dbInst* inst) const{
    if (inst){
      odb::dbMaster* master = inst -> getMaster();
      std::string master_name = master -> getName();
      std::transform(master_name.begin(), master_name.end(), master_name.begin(), ::toupper);
      
          if (
	      (master_name.find("BUF") != std::string::npos) ||
	      (master_name.find("INV") != std::string::npos) ||
	      (master_name.find("NAND") != std::string::npos)||
	      (master_name.find("NOR") != std::string::npos) ||
	      (master_name.find("XOR") != std::string::npos) ||
	      (master_name.find("MUX") != std::string::npos) ||
	      (master_name.find("AND") != std::string::npos) ||
	      (master_name.find("OR") != std::string::npos)){
	    return true;
	  }
    }
    return false;
    }

  
  bool EcoDesignManager::instConnectedToIos(odb::dbInst* cur_inst) const {
    for (odb::dbITerm* cur_pin : cur_inst ->  getITerms()) {
      odb::dbNet* cur_net = cur_pin -> getNet();
      if (cur_net -> getBTerms().size() >0){
	return true;
      }
    }
    return false;
  }

  

  bool EcoDesignManager::getFanin1Level(odb::dbITerm* ip_pin,
					std::vector<odb::dbITerm*>& ip_pins){

    int feasible_ip_pin_count=0;
    if (ip_pin){
      odb::dbInst* cur_inst  = ip_pin -> getInst();

      if (instConnectedToIos(cur_inst)){
	return false;
      }
      
      for (odb::dbITerm* cur_pin : cur_inst ->  getITerms()) {
	odb::dbNet* ip_net = cur_pin -> getNet();
	//cannot handle anything connected to a pad
	if (ip_net -> getBTerms().size() >0){
	  return false;
	}
	odb::dbObject* driver_object = ip_net -> getFirstDriverTerm();
	if (driver_object ->getObjectType() == odb::dbITermObj){
	  odb::dbITerm* driver_iterm = static_cast<odb::dbITerm*>(driver_object);
	  odb::dbInst* driver_inst = driver_iterm -> getInst();
	  
	  //we cannot handle cycles. instance looping back on itself..
	  if (driver_inst == cur_inst){
	    return false;
	  }
	  if (!RetimeableGate(driver_inst)){
	    return false;
	  }
	  if (instConnectedToIos(driver_inst)){
	    return false;
	  }
	  
	  for (odb::dbITerm* fanin_gate_pin : driver_inst  -> getITerms()){
	    if (fanin_gate_pin != driver_iterm){
	      if (fanin_gate_pin -> isInputSignal(false)){
		ip_pins.push_back(fanin_gate_pin);
		feasible_ip_pin_count++;
	      }
	    }
	  }
	}
      }
    }
    if (feasible_ip_pin_count >0){
      return true;
    }
    return false;
  }



  bool EcoDesignManager::getFanout1Level(odb::dbITerm* op_pin,
					 std::vector<odb::dbITerm*>& op_pins){

    int feasible_op_pin_count=0;
    if (op_pin){
      odb::dbInst* cur_inst  = op_pin -> getInst();

      if (instConnectedToIos(cur_inst)){
	return false;
      }
      
      for (odb::dbITerm* cur_pin : cur_inst ->  getITerms()) {

	odb::dbNet* cur_net = cur_pin -> getNet();
	
	if (cur_pin -> isOutputSignal(false)){
	  odb::dbNet* op_net = cur_pin -> getNet();
	  
	  for (auto iterm : op_net -> getITerms()){
	    if ((iterm != cur_pin) && iterm -> getInst() == cur_inst){
	      return false;
	    }
	    odb::dbInst* fanout_inst = iterm -> getInst();

	    //avoid any feedback loops
	    if (fanout_inst == cur_inst){
	      return false;
	    }
	    //avoid any gates we cannot push flops through
	    if (!RetimeableGate(fanout_inst)){
	      return false;
	    }
	    //avoid any gates connected to ios.
	    if (instConnectedToIos(fanout_inst)){
	      return false;
	    }
	    for (auto fanout_gate_iterm: fanout_inst -> getITerms()){
	      if (fanout_gate_iterm -> isOutputSignal(false)){
		op_pins.push_back(fanout_gate_iterm);
		feasible_op_pin_count++;
	      }
	    }
	  }
	}
      }
    }
    if (feasible_op_pin_count >0){
      return true;
    }
    return false;
  }



  
  void EcoDesignManager::updateTiming(bool full){
    sta_ -> ensureGraph();
    sta_ -> searchPreamble();
    sta_ -> updateTiming(full);
  }

  
  bool EcoDesignManager::areCompatibleMasters(const std::string& spare_master,
					      const std::string& inst_master) const{
    if (spare_master == inst_master){
      return true;
    }
    std::string spare_base_name = extractBaseName(spare_master); 
    std::string inst_base_name = extractBaseName(inst_master);
    if (spare_base_name == inst_base_name){
      if (arePinCompatible(spare_base_name, inst_base_name)){
	return true;
      }
    }
    return false;
  }

  bool EcoDesignManager::arePinCompatible(const std::string& master1_name, 
					  const std::string& master2_name) const {
    odb::dbMaster* master1 = db_->findMaster(master1_name.c_str());
    odb::dbMaster* master2 = db_->findMaster(master2_name.c_str());
    if (!master1 || !master2) {
      return false;
    }
    // Check same number of pins
    if (master1->getMTermCount() != master2->getMTermCount()) {
      return false;
    }
    // Check pin names and directions match
    std::map<std::string, odb::dbSigType> pins1, pins2;
    for (odb::dbMTerm* mterm : master1->getMTerms()) {
      pins1[mterm->getName()] = mterm->getSigType();
    }
    for (odb::dbMTerm* mterm : master2->getMTerms()) {
      pins2[mterm->getName()] = mterm->getSigType();
    }
    return pins1 == pins2;
  }


  odb::dbITerm* EcoDesignManager::getPinByMTermName(odb::dbInst* inst,
						    const std::string& mterm_name){

    for (odb::dbITerm* iterm : inst-> getITerms()){
      auto mterm = iterm -> getMTerm();
      if (mterm -> getName() == mterm_name){
	return iterm;
      }
    }
    printf("Failed to find pin \n");
    return nullptr;
  }


  /*
    Todo: remove hardcoding of names !
  */

  bool EcoDesignManager::isDFFRS(odb::dbInst* db_inst,
				 odb::dbITerm* &d_in,
				 odb::dbITerm* &q_out){

    d_in = nullptr;
    q_out = nullptr;
    if (db_inst ){
      std::string master_name = db_inst -> getMaster() -> getName();
      std::transform(master_name.begin(), master_name.end(), master_name.begin(), ::toupper);      
      if (master_name.find("DFFS") != std::string::npos ||
	  master_name.find("DFFR") != std::string::npos 
	  ){
	for (auto iterm : db_inst -> getITerms()){
	  auto mterm = iterm -> getMTerm();
	  if (mterm -> getName() == "D"){
	    d_in = iterm;
	  }
	  if (mterm -> getName() == "Q"){
	    q_out = iterm;
	  }
	}
      }
      if (d_in && q_out){
	return true;
      }
    }
    return false;
  }


  bool EcoDesignManager::getSpareMux2(
				      std::shared_ptr<SpareCell>& mux_spare,
				      odb::dbITerm*& mux2xn_d0,
				      odb::dbITerm*& mux2xn_d1,
				      odb::dbITerm*& mux2xn_sel,
				      odb::dbITerm*& mux2xn_op){
    
    std::vector<std::shared_ptr<SpareCell> > spare_cells =
      getAvailableSpares("MUX2_X2");

    //
    //Mux equation (hardcoded!)
    //op  S&B | ~S&A
    //select high -> B (mux2xn_d1)
    //select low -> A (mux2xn_d0)
    //select -> S (mux2xn_sel)
    //op -> Z (mux2xn_op)
    //
    
    if (spare_cells.size() >0){
      mux_spare = spare_cells[0];
      odb::dbInst* mux_inst = spare_cells[0] -> instance;
      for (auto iterm: mux_inst -> getITerms()){
	auto mterm =  iterm -> getMTerm();
	if (mterm -> getName() == "A"){
	  mux2xn_d0 = iterm;
	}
	if (mterm -> getName() == "B"){
	  mux2xn_d1 = iterm;
	}
	if (mterm -> getName() == "S"){
	  mux2xn_sel = iterm;
	}
	if (mterm -> getName() == "Z"){
	  mux2xn_op = iterm;
	}
      }
      if (mux2xn_d0 && mux2xn_d1 && mux2xn_sel && mux2xn_op){
	return true;
      }
    }
    printf("Failed to find spare mux cell for shannon splitting !\n");
    return false;
  }
				      

  odb::dbNet* EcoDesignManager::findOrCreateConstantNet(bool constant_phase,
							sta::dbNetwork* db_network){
    odb::dbNet* ret=nullptr;

    if (constant_phase == false &&
	gnd_net_ != nullptr){
      return gnd_net_;
    }
    if (constant_phase == true &&
	pwr_net_ != nullptr){
      return pwr_net_;
    }
    for (odb::dbNet* net: block_ -> getNets()){
      if (constant_phase == false){
	if (net -> getSigType() == odb::dbSigType::GROUND){
	  gnd_net_ = net;
	  return net;
	}
      }
      if (constant_phase == true){
	if (net -> getSigType() == odb::dbSigType::POWER){
	  pwr_net_ = net;
	  return net;
	}
      }
    }
    //TODO: create net if missing.
    printf("Weird, no power/ground net. TODO: build net\n");
    return ret;
    }

					    
  odb::dbNet* EcoDesignManager::mkFlatNet(){
    sta::dbNetwork* db_network = resizer_ -> getDbNetwork();
    odb::dbNet* ret =  db_network -> flatNet(dbnetwork_ ->
							makeNet(dbnetwork_ -> topInstance()));
    return ret;
  }

  /*
    A singleton net has one driver and one sink
  */
  
  bool EcoDesignManager::singletonNet(odb::dbNet* cur_net) const {
    if (cur_net) {
      //singleton cannot drive any ios
      if ((cur_net -> getBTerms()).size() > 0)
	return false;

      int ip_count=0;
      int op_count=0;
      for (auto term: cur_net -> getITerms()){
	if (term -> isInputSignal(false)){
	  ip_count++;
	}
	else if (term -> isOutputSignal(false)){
	  op_count++;
	}
      }
      if (ip_count == 1 && op_count == 1){
	return true;
      }
    }
    return false;
  }
  
  void EcoDesignManager::getFlopPins(odb::dbInst* flop,
				     
				     odb::dbITerm* &d_pin,
				     odb::dbITerm* &q_pin,
				     odb::dbITerm* &qn_pin,
				     odb::dbITerm* &clk_pin,
				     odb::dbITerm* &r_pin,
				     odb::dbITerm* &s_pin){

    d_pin = nullptr;
    q_pin = nullptr;
    clk_pin = nullptr;
    r_pin = nullptr;
    s_pin = nullptr;
    
    if (flop){
      for (odb::dbITerm* cur_pin : flop ->  getITerms()) {
	auto mterm = cur_pin -> getMTerm();
	if (mterm -> getName() == "CK"){
	  clk_pin = cur_pin;
	}
	else if (mterm -> getName() == "D"){
	  d_pin = cur_pin;
	}
	else if (mterm -> getName() == "Q"){
	  q_pin = cur_pin;
	}
	else if (mterm -> getName() == "QN"){
	  qn_pin = cur_pin;
	}
	else if (mterm -> getName() == "CK"){
	  clk_pin = cur_pin;
	}
	else if (mterm -> getName() == "RN"){
	  r_pin = cur_pin;
	}
	else if (mterm -> getName() == "SN"){
	  s_pin = cur_pin;
	}
      }
    }
  }

  
}
