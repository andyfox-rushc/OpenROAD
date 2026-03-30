/*
The ECO Design Manager
----------------------

Performs ECO moves. Where possible using the resizer infra structure.
The resizer offers journalling for easy do/undo and also buffer
insertion, resizing and other moves.

*/
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
#include <iostream>
#include <sstream>

using namespace odb;
using namespace sta;
using namespace grt;

namespace eco {
EcoDesignManager::EcoDesignManager(dbDatabase* db,
				   Sta* sta, 
				   GlobalRouter* router,
				   rsz::Resizer* resizer,
				   utl::Logger* logger)
  : db_(db), sta_(sta), router_(router), resizer_(resizer), logger_(logger){
    
    if (!db_ || !sta_ || !resizer_) {
        throw std::runtime_error("Database, STA, and Resizer objects must not be null");
    }
    
    block_ = db_->getChip()->getBlock();
    if (!block_) {
        throw std::runtime_error("No block found in database");
    }
    dbsta_= dynamic_cast<dbSta*>(sta);
    if (!dbsta_){
      throw std::runtime_error("No dbsta found in sta");
    }
    // Initialize spare cell inventory
    identifySpareCells();

    //needed for estimation
    resizer -> getEstimateParasitics() -> setIncrementalParasiticsEnabled(true);
}

EcoDesignManager::~EcoDesignManager() {
}


  std::vector<std::shared_ptr<SpareCell> > EcoDesignManager::getSpareCells(){
    return spare_cells_;
  }


  dbInst* EcoDesignManager::findInst(std::string& inst_name){
    return (block_ -> findInst(inst_name.c_str()));
  }
  
void EcoDesignManager::identifySpareCells() {
    spare_cells_.clear();
    spare_cells_by_type_.clear();
    
    // Scan all instances for spare cells
    for (dbInst* inst : block_->getInsts()) {
        if (isSpareCell(inst)) {
	  std::shared_ptr<SpareCell> spare = std::make_shared<SpareCell>();
            spare -> instance = inst;
            spare -> master = inst->getMaster();
            
            // Get location
            int x, y;
            inst->getLocation(x, y);
            spare -> x = x;
            spare -> y = y;
            
            // Check if it's already connected (used)
            spare -> is_used = false;
            for (dbITerm* iterm : inst->getITerms()) {
                if (iterm->getNet() != nullptr) {
		  if (!(iterm -> getNet() -> isSpecial())){
                    spare -> is_used = true;
                    break;
		  }
                }
            }
            
            // Determine type based on master cell name
            spare -> type = getSpareType(spare -> master);
            
            if (!spare -> is_used) {
                spare_cells_.push_back(spare);
                spare_cells_by_type_[spare-> type].push_back(spare_cells_.back());
            }
        }
    }
    
    std::cout << "Identified " << spare_cells_.size() << " available spare cells" << std::endl;
    // Print summary by type
    for (const auto& [type, cells] : spare_cells_by_type_) {
        std::cout << "  Type " << type << ": " << cells.size() << " cells" << std::endl;
    }
}

  

  
bool EcoDesignManager::isSpareCell(dbInst* inst) const {
    std::string inst_name = inst->getName();
    // Common patterns: SPARE_*, FILL_*, spare_*, eco_*
    return (inst_name.find("SPARE_") == 0 || 
            inst_name.find("FILL_") == 0 ||
            inst_name.find("spare_") == 0 ||
            inst_name.find("testspare_") == 0 ||	    
            inst_name.find("eco_") == 0);
}

  
std::set<std::string> EcoDesignManager::getCompatibleMasters(
    const std::string& master_name) const {
    
    std::set<std::string> compatible_masters;
    
    // Extract the base cell type (remove drive strength suffix)
    std::string base_name = extractBaseName(master_name);
    
    // Get all masters in the library
    
    std::vector<dbMaster*> masters;
    block_ -> getMasters(masters);
    for (dbMaster* master : masters){
        std::string candidate_name = master->getName();
        
        // Skip if same as current
        if (candidate_name == master_name) {
            continue;
        }
        
        // Check if same base type
        if (extractBaseName(candidate_name) == base_name) {
            // Verify same number of pins and pin compatibility
            if (arePinCompatible(master_name, candidate_name)) {
                compatible_masters.insert(candidate_name);
            }
        }
    }
    
    return compatible_masters;
}

std::string EcoDesignManager::extractBaseName(const std::string& master_name) const {
    // Remove drive strength suffix (e.g., _X1, _X2, _X4, etc.)
    size_t pos = master_name.rfind("_X");
    if (pos != std::string::npos) {
        return master_name.substr(0, pos);
    }
    return master_name;
}

bool EcoDesignManager::arePinCompatible(const std::string& master1_name, 
                                        const std::string& master2_name) const {

  dbMaster* master1 = db_->findMaster(master1_name.c_str());
  dbMaster* master2 = db_->findMaster(master2_name.c_str());
    
  if (!master1 || !master2) {
    return false;
  }
    
    // Check same number of pins
  if (master1->getMTermCount() != master2->getMTermCount()) {
    return false;
  }
    
    // Check pin names and directions match
    std::map<std::string, dbSigType> pins1, pins2;
    
    for (dbMTerm* mterm : master1->getMTerms()) {
        pins1[mterm->getName()] = mterm->getSigType();
    }
    
    for (dbMTerm* mterm : master2->getMTerms()) {
        pins2[mterm->getName()] = mterm->getSigType();
    }
    
    return pins1 == pins2;
}

  

  /*
    TODO fix this hardcoded stuff
  */
std::string EcoDesignManager::getSpareType(dbMaster* master) const {
    std::string master_name = master->getName();

    // Convert to uppercase for comparison
    std::transform(master_name.begin(), master_name.end(), master_name.begin(), ::toupper);
    if (master_name.find("BUF") != std::string::npos) {
        return "BUF";
    } else if (master_name.find("INV") != std::string::npos) {
        return "INV";
    } else if (master_name.find("AND") != std::string::npos) {
        return "AND";
    } else if (master_name.find("OR") != std::string::npos) {
        return "OR";
    } else if (master_name.find("NAND") != std::string::npos) {
        return "NAND";
    } else if (master_name.find("NOR") != std::string::npos) {
        return "NOR";
    } else if (master_name.find("XOR") != std::string::npos) {
        return "XOR";
    } else if (master_name.find("MUX") != std::string::npos) {
        return "MUX";
    } else {
        return "GENERIC";
    }
}

  std::vector<std::shared_ptr<SpareCell> > EcoDesignManager::getAvailableSpares(
    const std::string& type) const {
    std::vector<std::shared_ptr<SpareCell> > available;
    
    for (const auto& spare : spare_cells_) {
        if (!spare -> is_used && (type.empty() || spare->type == type)) {
            available.push_back(spare);
        }
    }
    
    return available;
}

  
SpareGateSummary EcoDesignManager::getSpareGateSummary() const {
    SpareGateSummary summary;
    
    summary.total_count = 0;
    summary.used_count = 0;
    
    summary.buffers = 0;
    summary.inverters = 0;
    summary.logic_gates = 0;
    summary.sequentials = 0;

    // Count available (unused) spare cells by type
    for (const auto& spare : spare_cells_) {
      if (spare->is_used){
	summary.used_count++;
      }
      summary.total_count++;       

      sta::LibertyCell* lib_cell = dbsta_ -> getDbNetwork() -> findLibertyCell(spare->master -> getName().c_str());
      assert(lib_cell);
      if (isBuffer(lib_cell)){
	summary.buffers++;
      } else if (isInverter(lib_cell)){
	summary.inverters++;
      }
      else if (isSequential(lib_cell)){
	summary.sequentials++;
      }
      else if (spare->type == "AND" || 
	       spare->type == "OR" || 
	       spare->type == "NAND" || 
	       spare->type == "NOR" || 
	       spare->type == "XOR" || 
	       spare->type == "MUX") {
	summary.logic_gates++;
      }
    }
    return summary;
}

  
  long long  EcoDesignManager::getMaxRadius(odb::dbInst* instance){
    //use db units
    int inst_x, inst_y;
    instance->getLocation(inst_x, inst_y);
    odb::dbBox* bbox = instance->getBBox();
    
    if (bbox) {
        inst_x = (bbox->xMin() + bbox->xMax()) / 2;
        inst_y = (bbox->yMin() + bbox->yMax()) / 2;
    }
    long long min_x = inst_x;
    long long max_y = inst_y;
    
    //get maximum radius of any instance
    for (dbInst* cur_inst : block_->getInsts()) {
      if (cur_inst == instance)
	continue;
      int cur_inst_x;
      int cur_inst_y;
      cur_inst ->getLocation(cur_inst_x, cur_inst_y);

      max_y = cur_inst_y > max_y ? cur_inst_y: max_y;      
      min_x = cur_inst_x < min_x ? cur_inst_x: min_x;
    }

    //get square of radius
    long long  radius_squared = ((long long)(min_x)*(long long )(min_x)) +
      ((long long)max_y* (long long)max_y);
    long long unsigned radius_squared_u = min_x*min_x +
      max_y*max_y;

    return radius_squared;
  }

  
std::vector<std::shared_ptr<SpareCell> > EcoDesignManager::findSpareCellsNear(
    dbInst* instance, long long radius_squared_in) {

  //restrict search to nearby cells.
  long long radius_squared = radius_squared_in / 16;
  
    std::vector<std::shared_ptr<SpareCell>> nearby_spares;
    
    if (!instance) {
      printf("Bug -- given null instane !\n");
      exit(0);
      return nearby_spares;
    }
    
    // Get instance location (center point)
    int inst_x, inst_y;
    instance->getLocation(inst_x, inst_y);
    
    // Get instance bounding box to find center
    odb::dbBox* bbox = instance->getBBox();
    if (bbox) {
        inst_x = (bbox->xMin() + bbox->xMax()) / 2;
        inst_y = (bbox->yMin() + bbox->yMax()) / 2;
    }
    
    //    logger_->info(utl::ECO, 100, "Find spare for Instance {} at x {} y {}",
    //		  instance -> getName(),
    //		  inst_x,
    //		  inst_y);

    
    // Search through all spare cells
    // spare cells previously harvested.
    for (auto& spare : spare_cells_) {
        // Get spare cell instance
        dbInst* spare_inst = spare->instance;
        if (!spare_inst) {
            continue;
        }
        
        // Get spare cell location
        int spare_x, spare_y;
        spare_inst->getLocation(spare_x, spare_y);
        
        // Get spare cell center
        odb::dbBox* spare_bbox = spare_inst->getBBox();
        if (spare_bbox) {
            spare_x = (spare_bbox->xMin() + spare_bbox->xMax()) / 2;
            spare_y = (spare_bbox->yMin() + spare_bbox->yMax()) / 2;
        }

	
        // Calculate squared distance (avoid sqrt for performance)
        long long dx = static_cast<long long>(spare_x - inst_x);
        long long dy = static_cast<long long>(spare_y - inst_y);
        long long dist_squared = dx * dx + dy * dy;
        /*
        // Check if within radius
	logger_->info(utl::ECO, 100, 
		      "Seeking spare cell  dist_squared {} radius_squared {}",
		      dist_squared,
		      radius_squared);
	*/

        if (dist_squared <= static_cast<long long>(radius_squared)) {
            nearby_spares.push_back(spare);
	    /*
	    logger_->info(utl::ECO, 100, "Candidate spare {} at dx {} dy {} from instance {}",
			  spare -> instance -> getName(),
			  dx,
			  dy,
			  instance -> getName()
			  );
	    */
        }
    }
    
    // Sort by distance (closest first)
    std::sort(nearby_spares.begin(), nearby_spares.end(),
        [inst_x, inst_y](const std::shared_ptr<SpareCell>& a, 
                         const std::shared_ptr<SpareCell>& b) {
            dbInst* inst_a = a->instance;
            dbInst* inst_b = b->instance;
            
            if (!inst_a || !inst_b) {
                return inst_a != nullptr;  // Put non-null first
            }
            
            // Get centers
            odb::dbBox* bbox_a = inst_a->getBBox();
            odb::dbBox* bbox_b = inst_b->getBBox();
            
            int ax = bbox_a ? (bbox_a->xMin() + bbox_a->xMax()) / 2 : 0;
            int ay = bbox_a ? (bbox_a->yMin() + bbox_a->yMax()) / 2 : 0;
            int bx = bbox_b ? (bbox_b->xMin() + bbox_b->xMax()) / 2 : 0;
            int by = bbox_b ? (bbox_b->yMin() + bbox_b->yMax()) / 2 : 0;
            
            // Calculate distances
            long long dist_a = static_cast<long long>(ax - inst_x) * (ax - inst_x) + 
                              static_cast<long long>(ay - inst_y) * (ay - inst_y);
            long long dist_b = static_cast<long long>(bx - inst_x) * (bx - inst_x) + 
                              static_cast<long long>(by - inst_y) * (by - inst_y);
            
            return dist_a < dist_b;
        });


    return nearby_spares;
}

  
  
  std::shared_ptr<SpareCell> EcoDesignManager::findNearestSpare(
    int x, int y, const std::string& type) {
    
    std::shared_ptr<SpareCell> nearest=nullptr;
    double min_distance = std::numeric_limits<double>::max();
    
    // If type is specified, search only in that type
    if (!type.empty() && spare_cells_by_type_.find(type) != spare_cells_by_type_.end()) {
      for (std::shared_ptr<SpareCell> spare : spare_cells_by_type_[type]) {
            if (!spare->is_used) {
                double dx = spare->x - x;
                double dy = spare->y - y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest = spare;
                }
            }
        }
    } else {
        // Search all spare cells
        for (auto spare : spare_cells_) {
            if (!spare->is_used && (type.empty() || spare->type == type)) {
                double dx = spare->x - x;
                double dy = spare->y - y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest = spare;
                }
            }
        }
    }
    
    return nearest;
}


void EcoDesignManager::capturePreMoveMetrics(double& tns, double& area, double& wire_length) {
    tns = evaluateTotalNegativeSlack();
    area = evaluateDesignArea();
    wire_length = evaluateTotalWireLength();
}


EcoDesignManager::MoveResult EcoDesignManager::calculateMoveImpact(
    double pre_tns, double pre_area, double pre_wire) {
    
    MoveResult result;
    double post_tns = evaluateTotalNegativeSlack();
    double post_area = evaluateDesignArea();
    double post_wire = evaluateTotalWireLength();

    result.timing_improvement = post_tns - pre_tns;  // Positive is good


    result.area_delta = post_area - pre_area;
    result.wire_length_delta = post_wire - pre_wire;
    result.success = true;
    return result;
}


  /*
    Preview the resizing
  */
  
  EcoDesignManager::MoveResult EcoDesignManager::previewResize(
							       odb::dbInst* inst,
							       odb::dbInst* spare_inst
					     ){
    // Use Resizer's journaling
    resizer_->journalBegin();
    
    // Perform the resize
    MoveResult result = performInstanceSwap(inst, spare_inst);
    
    // Restore original state
    resizer_->journalRestore();
    
    // Result contains timing_improvement without permanent changes
    return result;
  }

  
  EcoDesignManager::MoveResult EcoDesignManager::performInstanceSwap(
							     odb::dbInst* inst,
							     odb::dbInst* spare_inst){

    /*    printf("Swapping Instance %s (cell %s)  with spare instance %s (cell %s)\n",
	   inst -> getName().c_str(),
	   inst -> getMaster() -> getName().c_str(),
	   spare_inst -> getName().c_str(),
	   spare_inst -> getMaster() -> getName().c_str()
	   );
    */
    // Record initial metrics
    double initial_tns, initial_area, initial_wire;
    capturePreMoveMetrics(initial_tns, initial_area, initial_wire);


    // Find the instance
    if (inst == nullptr) {
      return {false, 0.0, 0.0, 0.0, "Instance not found: " + inst-> getName()};
    }
    if (spare_inst == nullptr) {
      return {false, 0.0, 0.0, 0.0, "Instance not found: " + spare_inst-> getName()};
    }

    //disconnect the inst
    std::map<std::string,odb::dbNet*> pin2net;
    for (auto iterm: inst->getITerms()){
      pin2net[iterm -> getMTerm() -> getName()] = iterm -> getNet();
      iterm -> disconnect();
    }
    
    //connect up the spare inst
    for (auto iterm: spare_inst -> getITerms()){
      odb::dbNet* cur_net = pin2net[iterm -> getMTerm() -> getName()];
      assert(cur_net);
      iterm -> connect(cur_net);
    }
    
    // Update timing
    sta_->updateTiming(false);
    
    // Calculate impact
    MoveResult result = calculateMoveImpact(initial_tns, initial_area, initial_wire);

    return result;
  }
  
  
EcoDesignManager::MoveResult EcoDesignManager::performResize(
							     odb::dbInst* inst,
							     odb::dbInst* spare_inst){
    
    // Record initial metrics
    double initial_tns, initial_area, initial_wire;
    capturePreMoveMetrics(initial_tns, initial_area, initial_wire);
    
    // Find the instance
    if (!inst) {
      return {false, 0.0, 0.0, 0.0, "Instance not found: " + inst-> getName()};
    }

    
    /*
    // Check if we need to use a spare cell
    if (isSpareCell(inst)) {
    
      printf("Not sure what the hell this does!\n");

        // Handle spare cell replacement
      std::string new_master_name = spare_inst-> getName();
        dbMaster* new_master = db_->findMaster(new_master_name.c_str());
        if (!new_master) {
	  return {false, 0.0, 0.0, 0.0, "Master cell not found: " + new_master_name};
        }
        
        std::string spare_type = getSpareType(new_master);
	std::shared_ptr<SpareCell> spare = findNearestSpare(target_inst->getBBox()->xMin(), 
					    target_inst->getBBox()->yMin(), 
					    spare_type);
        if (!spare) {
	  return {false, 0.0, 0.0, 0.0, "No suitable spare cell available"};
        }
        // Perform the spare cell swap
        performSpareCellSwap(target_inst, spare, new_master);

    } else {
        // For regular cells, use Resizer's functionality
        sta::Instance* sta_inst = dbsta_->getDbNetwork()->dbToSta(inst);
        sta::LibertyCell* new_cell = dbsta_ -> getDbNetwork() ->
	  findLibertyCell(spare_inst -> getMaster() -> getName().c_str());
        
        if (!new_cell) {
	  return {false, 0.0, 0.0, 0.0, "New master cell not found: " + new_master_name};
        }
        
        // Check if this is a valid swap using Resizer's logic
        sta::LibertyCell* current_cell = resizer_->getDbNetwork()->libertyCell(sta_inst);
        auto swappable_cells = resizer_->getSwappableCells(current_cell);

	printf("Source cell type %s\n", inst -> getMaster() -> getName().c_str());
	
        bool is_valid_swap = false;
        for (auto cell : swappable_cells) {
	  printf("Allowed swappable cell %s\n", cell -> name());
	  if (cell == new_cell) {
	    is_valid_swap = true;
	    break;
	  }
        }
        
        if (!is_valid_swap) {
	  //
	  //try a spare cell ???
	  //
	  return {false, 0.0, 0.0, 0.0, "Cell swap not valid according to Resizer rules"};
        }
        
        // Use Resizer's replaceCell 
        if (!resizer_->replaceCell(sta_inst, new_cell, true)) {
	  return {false, 0.0, 0.0, 0.0, "Failed to replace cell"};
        }
    }
    */

    // Update timing
    sta_->updateTiming(false);
    
    // Calculate impact
    MoveResult result = calculateMoveImpact(initial_tns, initial_area, initial_wire);

    return result;
}



  
void EcoDesignManager::performSpareCellSwap(dbInst* target_inst, 
					    std::shared_ptr<SpareCell> spare,
					    dbMaster* new_master) {
    // Save connections from target
    std::unordered_map<std::string, dbNet*> pin_connections;
    for (dbITerm* iterm : target_inst->getITerms()) {
        if (iterm->getNet()) {
            pin_connections[iterm->getMTerm()->getName()] = iterm->getNet();
        }
    }
    
    // Swap the spare cell master
    spare->instance->swapMaster(new_master);
    spare->is_used = true;
    
    // Transfer connections
    for (const auto& [pin_name, net] : pin_connections) {
        dbITerm* spare_iterm = spare->instance->findITerm(pin_name.c_str());
        if (spare_iterm) {
            spare_iterm->connect(net);
        }
    }
    
    // Disconnect original instance
    for (dbITerm* iterm : target_inst->getITerms()) {
        iterm->disconnect();
    }
}


  

bool EcoDesignManager::routeNet(dbNet* net) {
    if (!net || !router_) {
        return false;
    }
    
    // For actual routing, integrate with GlobalRouter
    // This is a placeholder - actual implementation would call:
    // router_->routeNet(net);
    printf("Unsupported route\n");
    return true;
}

bool EcoDesignManager::unrouteNet(dbNet* net) {
    if (!net) {
        return false;
    }
    // Remove routing for the net (segments)
    net->destroySWires();
    return true;
}

dbInst* EcoDesignManager::insertSpareCell(const std::string& spare_master_name,
                                         const std::string& instance_name,
                                         int x, int y) {
    dbMaster* master = db_->findMaster(spare_master_name.c_str());
    if (!master) {
        return nullptr;
    }
    
    // Create new instance
    dbInst* inst = dbInst::create(block_, master, instance_name.c_str());
    if (!inst) {
        return nullptr;
    }
    
    // Set location
    inst->setLocation(x, y);
    inst->setPlacementStatus(dbPlacementStatus::PLACED);
    
    // Add to spare cell tracking
    std::shared_ptr<SpareCell> spare = std::make_shared<SpareCell>();
    spare -> instance = inst;
    spare -> master = master;
    spare -> x = x;
    spare -> y = y;
    spare -> is_used = false;
    spare -> type = getSpareType(master);
    
    spare_cells_.push_back(spare);
    // Update spare cells by type
    spare_cells_by_type_[spare->type].push_back(spare_cells_.back());
    
    return inst;
}


  /* 
     Timing apis
   */

std::vector<PathInfo> EcoDesignManager::getCriticalPaths(int path_count) {
    std::vector<PathInfo> critical_paths;
    
    if (!sta_) {
        logger_->error(utl::ECO, 1, "STA not initialized");
        return critical_paths;
    }
    
    // Update timing to ensure we have current data
    sta_->updateTiming(true);
    
    // Get worst paths from all path groups
    sta::PathEndSeq path_ends = sta_->findPathEnds(
        nullptr,  // from (nullptr = any)
        nullptr,  // thrus 
        nullptr,  // to (nullptr = any)
        false,    // unconstrained
        nullptr,  // corner (nullptr = all corners)
        sta::MinMaxAll::max(),  // max for setup checks
        path_count,  // group_count - paths per group
        path_count,  // endpoint_count - paths per endpoint
        true,     // unique_pins
        false,    // unique_edges
        -sta::INF,  // slack_min
        sta::INF,   // slack_max
        true,     // sort_by_slack
        nullptr,  // group_names (nullptr = all groups)
        true,     // setup
        false,    // hold
        false,    // recovery
        false,    // removal
        false,    // clk_gating_setup
        false     // clk_gating_hold
    );
    
    if (path_ends.empty()) {
        logger_->info(utl::ECO, 100, "No critical paths found");
        return critical_paths;
    }
    
    // Convert STA paths to our PathInfo structure
    int path_index = 0;
    for (sta::PathEnd* path_end : path_ends) {
        if (!path_end) continue;
        
        PathInfo info;
        info.slack = path_end->slack(sta_);
        info.index = path_index++;


        // Skip paths with positive slack if we have enough critical ones
        if (info.slack > 0 && critical_paths.size() >= static_cast<size_t>(path_count / 2)) {
            continue;
        }
        
        // Store the path
        info.sta_path = path_end->path();
        
        // Extract instances and nets along the path
        extractPathElements(path_end->path(), info.instances, info.nets);
        
        critical_paths.push_back(info);
        
        // Stop if we have enough paths
        if (critical_paths.size() >= static_cast<size_t>(path_count)) {
            break;
        }
    }
    
    // Log summary
    if (!critical_paths.empty()) {
        logger_->info(utl::ECO, 101, 
            "Found {} critical paths, WNS = {:.2f} ps", 
            critical_paths.size(), 
            critical_paths[0].slack * 1e12);  // Convert to ps
    }
    
    return critical_paths;
}
  
  

// Helper method to extract instances and nets from a path
void EcoDesignManager::extractPathElements(sta::Path* path,
                                         std::vector<odb::dbInst*>& instances,
                                         std::vector<odb::dbNet*>& nets) {
    instances.clear();
    nets.clear();
    
    if (!path) return;
    
    // Use PathExpanded to get the full path
    sta::PathExpanded expanded(path, sta_);
    
    for (size_t i = 0; i < expanded.size(); i++) {
        const sta::Path* path_ref = expanded.path(i);
        sta::Pin* pin = path_ref->pin(sta_);
        
        if (pin) {
            // Get instance from pin
            odb::dbITerm* iterm = nullptr;
            odb::dbBTerm* bterm = nullptr;
	    odb::dbModITerm* moditerm=nullptr;
	    
            // Convert sta Pin to db pin - need to use the network adapter
	    sta::dbNetwork* db_network = resizer_ -> getDbNetwork();

	    db_network -> staToDb(pin, iterm, bterm, moditerm);
            
            if (iterm) {
                odb::dbInst* inst = iterm->getInst();
                
                // Add instance if not already in list
                if (inst && (instances.empty() || instances.back() != inst)) {
                    instances.push_back(inst);
                }
                
                // Get net connected to this pin
                odb::dbNet* net = iterm->getNet();
                if (net && (nets.empty() || nets.back() != net)) {
                    nets.push_back(net);
                }
            } else if (bterm) {
                // Handle top-level port connections
                odb::dbNet* net = bterm->getNet();
                if (net && (nets.empty() || nets.back() != net)) {
                    nets.push_back(net);
                }
            }
        }
    }
}

std::vector<PathInfo> EcoDesignManager::getPathsThroughInstance(
    odb::dbInst* inst, 
    int max_paths) {
    
    std::vector<PathInfo> paths;
    
    if (!inst || !sta_) return paths;
    
    // Create a pin set for all pins of this instance
    sta::PinSet* through_pins = new sta::PinSet;
    
    for (odb::dbITerm* iterm : inst->getITerms()) {
      sta::Pin* pin = resizer_ -> getDbNetwork()->dbToSta(iterm);
        if (pin) {
            through_pins->insert(pin);
        }
    }
    
    // Create exception through specification
    sta::ExceptionThru* thru = sta_->makeExceptionThru(
        through_pins,
        nullptr,  // nets
        nullptr,  // instances
        sta::RiseFallBoth::riseFall()
    );
    
    sta::ExceptionThruSeq* thrus = new sta::ExceptionThruSeq;
    thrus->push_back(thru);
    
    // Find paths through these pins
    sta::PathEndSeq path_ends = sta_->findPathEnds(
        nullptr,  // from
        thrus,    // through this instance
        nullptr,  // to
        false,    // unconstrained
        nullptr,  // corner
        sta::MinMaxAll::max(),
        max_paths,
        max_paths,
        true,     // unique pins
        false,    // unique edges
        -sta::INF,
        0.0,      // Only negative slack
        true,     // sort_by_slack
        nullptr,  // group_names
        true,     // setup
        false,    // hold
        false,    // recovery
        false,    // removal
        false,    // clk_gating_setup
        false     // clk_gating_hold
    );
    
    // Process the path ends
    for (sta::PathEnd* path_end : path_ends) {
        if (!path_end) continue;
        
        PathInfo info;
        info.sta_path = path_end->path();
        info.slack = path_end->slack(sta_);
        info.index = paths.size();
        
        extractPathElements(info.sta_path, info.instances, info.nets);
        paths.push_back(info);
        
        if (paths.size() >= static_cast<size_t>(max_paths)) {
            break;
        }
    }
    
    // Clean up
    sta_->deleteExceptionThru(thru);
    delete thrus;
    // Note: through_pins is owned by thru and will be deleted with it
    
    return paths;
}

// Similarly for getCriticalPathsThroughNet
std::vector<PathInfo> EcoDesignManager::getCriticalPathsThroughNet(
    odb::dbNet* net, 
    int max_paths) {
    
    std::vector<PathInfo> paths;
    
    if (!net || !sta_) return paths;
    
    // Create a net set for the exception
    sta::NetSet* through_nets = new sta::NetSet;
    sta::Net* sta_net = resizer_ -> getDbNetwork()->dbToSta(net);
    
    if (!sta_net) {
        delete through_nets;
        return paths;
    }
    
    through_nets->insert(sta_net);
    
    // Create exception through specification
    sta::ExceptionThru* thru = sta_->makeExceptionThru(
        nullptr,  // pins
        through_nets,  // this net
        nullptr,  // instances
        sta::RiseFallBoth::riseFall()
    );
    
    sta::ExceptionThruSeq* thrus = new sta::ExceptionThruSeq;
    thrus->push_back(thru);
    
    // Find paths through this net
    sta::PathEndSeq path_ends = sta_->findPathEnds(
        nullptr,  // from
        thrus,    // through this net
        nullptr,  // to
        false,    // unconstrained
        nullptr,  // corner
        sta::MinMaxAll::max(),
        max_paths,
        max_paths,
        true,     // unique pins
        false,    // unique edges
        -sta::INF,
        0.0,      // Only negative slack
        true,     // sort_by_slack
        nullptr,  // group_names
        true,     // setup
        false,    // hold
        false,    // recovery
        false,    // removal
        false,    // clk_gating_setup
        false     // clk_gating_hold
    );
    
    // Process the path ends
    for (sta::PathEnd* path_end : path_ends) {
        if (!path_end) continue;
        
        PathInfo info;
        info.sta_path = path_end->path();
        info.slack = path_end->slack(sta_);
        info.index = paths.size();
        
        extractPathElements(info.sta_path, info.instances, info.nets);
        paths.push_back(info);
        
        if (paths.size() >= static_cast<size_t>(max_paths)) {
            break;
        }
    }
    
    // Clean up
    sta_->deleteExceptionThru(thru);
    delete thrus;
    // Note: through_nets is owned by thru and will be deleted with it
    
    return paths;
}
  


double EcoDesignManager::evaluateTimingSlack(const std::string& pin_name) {
    // Find the pin
    auto parts = splitPinName(pin_name);
    if (parts.first.empty() || parts.second.empty()) {
        return 0.0;
    }
    
    dbInst* inst = block_->findInst(parts.first.c_str());
    if (!inst) {
        return 0.0;
    }
    
    dbITerm* iterm = inst->findITerm(parts.second.c_str());
    if (!iterm) {
        return 0.0;
    }
    
    // Convert to STA pin and get slack
    sta::Pin* sta_pin = resizer_->getDbNetwork()->dbToSta(iterm);
    if (!sta_pin) {
        return 0.0;
    }
    
    return sta_->pinSlack(sta_pin, sta::MinMax::max());
}

double EcoDesignManager::evaluateTotalNegativeSlack() {
    // Use STA functionality to get TNS
    sta_->ensureGraph();
    sta_->searchPreamble();
    sta_->updateTiming(false);    
    double tns = sta_->totalNegativeSlack(sta_->cmdCorner(), sta::MinMax::max());
    sta::Unit *unit = Sta::sta()->units()->timeUnit();
    double tns_out = unit -> staToUser(tns); 
    return tns_out;
}

double EcoDesignManager::evaluateWorstNegativeSlack() {
  
    sta_->ensureGraph();
    sta_->searchPreamble();
    Slack worst_slack;
    Vertex* worst_vertex;
    Sta::sta()->worstSlack(sta::MinMax::max(),worst_slack, worst_vertex);
    sta::Unit *unit = Sta::sta()->units()->timeUnit();
    double worst_slack_out = unit -> staToUser(worst_slack); 
    return worst_slack_out;
}

  double EcoDesignManager::evaluateTotalPower(){
    printf("Evaluate total power todo!\n");
    return 0.1;
  }
  
double EcoDesignManager::evaluateDesignArea() {
    // Use Resizer's area calculation
    return resizer_->designArea();
}

double EcoDesignManager::evaluateTotalWireLength() {
    double total_length = 0.0;
    
    // Calculate actual wire length from routed nets
    for (dbNet* net : block_->getNets()) {
        dbWire* wire = net->getWire();
        if (wire) {
            // Get wire length in DBU and convert to microns
            uint64_t wire_length_dbu = wire->getLength();
            double dbu_per_micron = block_->getDbUnitsPerMicron();
            total_length += wire_length_dbu / dbu_per_micron;
        }
    }
    
    return total_length;
}



// Helper function to split instance_name/pin_name format
std::pair<std::string, std::string> EcoDesignManager::splitPinName(const std::string& pin_name) {
    size_t slash_pos = pin_name.find('/');
    if (slash_pos != std::string::npos) {
        return {pin_name.substr(0, slash_pos), pin_name.substr(slash_pos + 1)};
    }
    return {"", ""};
}


// Buffer insertion using Resizer
EcoDesignManager::MoveResult EcoDesignManager::insertBuffer(
    const std::string& net_name,
    const std::string& buffer_type,
    int x, int y) {
    
    // Record initial metrics
    double initial_tns, initial_area, initial_wire;
    capturePreMoveMetrics(initial_tns, initial_area, initial_wire);
    
    // Find the net
    dbNet* net = block_->findNet(net_name.c_str());
    if (!net) {
        return {false, 0.0, 0.0, 0.0, "Net not found: " + net_name};
    }
    
    
    // Convert to STA net
    sta::Net* sta_net = resizer_->getDbNetwork()->dbToSta(net);
    
    // Find buffer cell
    sta::LibertyCell* buffer_cell = dbsta_ -> getDbNetwork() -> findLibertyCell(buffer_type.c_str());
    if (!buffer_cell) {
        return {false, 0.0, 0.0, 0.0, "Buffer cell not found: " + buffer_type};
    }
    
    // Use Resizer's buffer insertion (which will be journaled)
    Point loc(x, y);
    sta::Instance* buffer = resizer_->insertBufferAfterDriver(
        sta_net, buffer_cell, &loc);
    
    if (!buffer) {
        return {false, 0.0, 0.0, 0.0, "Failed to insert buffer"};
    }
    
    // Update timing
    sta_->updateTiming(false);
    
    // Calculate impact
    MoveResult result = calculateMoveImpact(initial_tns, initial_area, initial_wire);
    
    return result;
}

  /* Helpers */

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

  
  
  //Get the area of a master cell
  double EcoDesignManager::getMasterArea(const std::string& master_name) {
    // First try Liberty cell (has area directly)
    sta::LibertyCell* lib_cell = dbsta_->getDbNetwork()->findLibertyCell(master_name.c_str());
    if (lib_cell) {
        float area = lib_cell->area();
        if (area > 0) {
            return area; // Already in square microns
        }
    }
    
    // Fall back to dbMaster
    odb::dbMaster* master = db_->findMaster(master_name.c_str());
    if (!master) {
        logger_->warn(utl::ECO, 201, "Cannot find master {}", master_name);
        return 0.0;
    }
    
    // Skip non-placeable cells (like pads, macros)
    if (!master->isCoreAutoPlaceable()) {
        return 0.0;
    }
    
    // Method 1: Direct area (if available)
    uint64_t area = master->getArea();
    if (area > 0) {
        double dbu_per_micron = block_->getDbUnitsPerMicron();
        return area / (dbu_per_micron * dbu_per_micron);
    }
    
    // Method 2: Width × Height
    double dbu_per_micron = block_->getDbUnitsPerMicron();
    double width_um = master->getWidth() / dbu_per_micron;
    double height_um = master->getHeight() / dbu_per_micron;
    
    return width_um * height_um;
}

  void EcoDesignManager::reportSpareCells(){
    for (auto spare: spare_cells_){
      logger_->info(utl::ECO, 100, "Spare Cell Instance {} with Master {}  of type {} at x {} y {}",
		    spare -> instance-> getName(),
		    spare -> master -> getName(),
		    spare -> type,
		    spare -> x,
		    spare -> y
		    );
    }
  }
  
}
