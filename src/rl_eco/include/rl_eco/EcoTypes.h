#pragma once

// OpenROAD includes
#include "odb/dbTypes.h"
#include "sta/Sta.hh"
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "sta/Liberty.hh"
#include "sta/NetworkClass.hh"
#include "sta/GraphClass.hh"
#include "sta/Sdc.hh"
#include "rsz/Resizer.hh"
#include "odb/db.h"

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace eco {

// ========== ECO-Specific Types Only ==========

//  
// Spare cell tracking (not in standard OpenROAD)
//


  
  // Spare cell management
    struct SpareCell {
        odb::dbInst* instance;
        odb::dbMaster* master;
        int x, y;
        bool is_used;
        std::string type; // e.g., "BUF", "INV", "AND2", etc.
    };

  
struct SpareGateSummary {
    int total_count = 0;
    int used_count = 0;
    
    // By category
    int buffers = 0;
    int inverters = 0;
    int logic_gates = 0;
    int sequentials = 0;
    
    double utilization_rate() const {
        return total_count > 0 ? double(used_count) / total_count : 0.0;
    }
};


  
// ========== Path Analysis Structure (needs to be public) ==========
struct PathInfo {
    sta::Path* sta_path;
    float slack;
    std::vector<odb::dbInst*> instances;
    std::vector<odb::dbNet*> nets;
    int index;  // Position in critical path list
};

    // Critical path analysis  
struct PathAnalysis {
    std::vector<PathInfo> critical_paths;
    std::unordered_map<std::string, std::vector<int>> instance_to_paths;
    std::unordered_map<std::string, double> instance_criticality;
};

  /*
// Result of an ECO move
struct MoveResult {
    bool success = false;
    std::string error_message;
    
    // Timing impact (extracted from STA after move)
    float timing_improvement = 0.0;  // in ps
    float wns_delta = 0.0;
    float tns_delta = 0.0;
    
    // Resource impact
    float area_delta = 0.0;
    float power_delta = 0.0;
    
    // What changed
    std::vector<odb::dbInst*> modified_instances;
    std::vector<odb::dbNet*> affected_nets;
    std::vector<SpareCell*> spares_consumed;
    
    // For journaling/undo
    int journal_restore_point;
};
  */
// ECO configuration 
struct EcoConfig {
    // Use existing rsz::Resizer parameters where possible
    rsz::Resizer* resizer = nullptr;
    
    // ECO-specific settings
    float critical_slack_threshold = -10.0f;  // ps
    float max_spare_distance = 500.0f;        // um (in DBU)
    
    int max_moves_per_episode = 100;
    int max_spare_usage_percent = 80;
    
    // Which moves are enabled
    bool enable_resize = true;
    bool enable_rebuffer = true;
    bool enable_load_split = true;
    bool enable_retime = false;
    bool enable_pipeline = false;
    bool enable_logic_remap = false;
};

// ========== Utility Functions ==========

// Convert between OpenROAD units
inline float dbuToMicrons(int dbu, odb::dbTech* tech) {
    return static_cast<float>(dbu) / tech->getLefUnits();
}

inline int micronsToDBU(float microns, odb::dbTech* tech) {
    return static_cast<int>(microns * tech->getLefUnits());
}

// Check cell types using Liberty data
inline bool isBuffer(sta::LibertyCell* cell) {
    return cell && cell->isBuffer();
}

inline bool isInverter(sta::LibertyCell* cell) {
    return cell && cell->isInverter();
}

inline bool isSequential(sta::LibertyCell* cell) {
    return cell && cell->hasSequentials();
}

// Get cell master from instance
inline odb::dbMaster* getCellMaster(odb::dbInst* inst) {
    return inst ? inst->getMaster() : nullptr;
}

// Get Liberty cell from master
  inline sta::LibertyCell* getLibertyCell(odb::dbMaster* master, sta::dbNetwork* db_network) {
    sta::Cell* cell = reinterpret_cast<sta::Cell*>(master->staCell());
    return const_cast<sta::LibertyCell*>(db_network-> libertyCell(cell));
}


  typedef std::map<std::string,//instance name of spare cell
		   std::vector<std::tuple <bool, std::shared_ptr<SpareCell> > > //used, reference to spare cell.
		 > SpareCellsDictionary;
  
    

  
} // namespace eco
