#ifndef ECO_DESIGN_MANAGER_H
#define ECO_DESIGN_MANAGER_H

#include <memory>
#include <vector>
#include <set>
#include <stack>
#include <unordered_map>
#include <string>
#include <utility>

// OpenROAD includes
namespace odb {
    class dbDatabase;
    class dbBlock;
    class dbInst;
    class dbNet;
    class dbITerm;
    class dbMaster;
  class dbInst;
}

namespace sta {
  class Sta;
  class dbSta;
}

namespace grt {
    class GlobalRouter;
}

namespace rsz {
    class Resizer;
}

namespace utl {
  class Logger;
};

namespace eco {
  
class EcoDesignManager {
public:
    // Move result structure to capture impact
    struct MoveResult {
        bool success;
        double timing_improvement;  // in picoseconds
        double area_delta;          // in square microns
        double wire_length_delta;   // in microns
        std::string error_message;
    };

    // Move descriptor for undo functionality
    struct Move {
        enum Type {
            RESIZE,
            REBUFFER,
            LOAD_SPLIT,
            RETIME,
            PIPELINE,
            LOGIC_REMAP
        };
        
        Type type;
        std::vector<std::string> affected_instances;
        std::vector<std::string> affected_nets;
        std::unordered_map<std::string, std::string> parameters;
    };

  


    // Constructor/Destructor
    EcoDesignManager(odb::dbDatabase* db,
		     sta::Sta* sta,
                     grt::GlobalRouter* router,
		     rsz::Resizer* resizer,
		     utl::Logger* logger);
    ~EcoDesignManager();


    // Main move operations
  //    MoveResult previewResize(const std::string& instance_name,
  //			     const std::string& new_master_name);
  MoveResult previewResize(odb::dbInst* inst,
			   odb::dbInst* spare_inst);

  //    MoveResult performResize(const std::string& instance_name, 
  //                           const std::string& new_master_name);

  MoveResult performResize(odb::dbInst* inst,
			   odb::dbInst* spare_inst);

  odb::dbInst* findInst(std::string& inst_name);
  
  MoveResult performInstanceSwap(odb::dbInst* inst,
				 odb::dbInst* spare_inst);
    
    // Stubbed move operations
    MoveResult performRebuffer(const std::string& net_name, 
                             const std::string& buffer_location) { 
        return {false, 0.0, 0.0, 0.0, "Rebuffering not implemented yet"}; 
    }
    
    MoveResult performLoadSplit(const std::string& driver_instance) { 
        return {false, 0.0, 0.0, 0.0, "Load splitting not implemented yet"}; 
    }
    
    MoveResult performRetime(const std::string& ff_instance, 
                           const std::string& target_location) { 
        return {false, 0.0, 0.0, 0.0, "Retiming not implemented yet"}; 
    }
    
    MoveResult performPipeline(const std::string& path_start, 
                             const std::string& path_end) { 
        return {false, 0.0, 0.0, 0.0, "Pipelining not implemented yet"}; 
    }
    
    MoveResult performLogicRemap(const std::vector<std::string>& cut_instances) { 
        return {false, 0.0, 0.0, 0.0, "Logic resynthesis not implemented yet"}; 
    }

    // Low-level physical operations
    bool routeNet(odb::dbNet* net);
    bool unrouteNet(odb::dbNet* net);
    odb::dbInst* insertSpareCell(const std::string& spare_master_name,
                                const std::string& instance_name,
                                int x, int y);
    bool removeCell(odb::dbInst* inst) { 
        return false; 
    }

    // Spare cell management
  void identifySpareCells();
  void reportSpareCells();  
  SpareGateSummary getSpareGateSummary() const;
  std::vector<std::shared_ptr<SpareCell> > getSpareCells();  
  std::vector<std::shared_ptr<SpareCell> > getAvailableSpares(const std::string& type = "") const;
  std::shared_ptr<SpareCell> findNearestSpare(int x, int y, const std::string& type);
  std::vector<std::shared_ptr<SpareCell> > findSpareCellsNear(
							      odb::dbInst* instance,
							      long long radius);

  std::set<std::string> getCompatibleMasters(const std::string& master_name) const;
  bool areCompatibleMasters(const std::string& spare_master, const std::string& inst_master) const;
  std::string extractBaseName(const std::string& master_name) const;
  bool arePinCompatible(const std::string& master1_name, const std::string& master_name) const;
  
  

    // Timing and metric evaluation

  // Critical path analysis
  std::vector<PathInfo> getCriticalPaths(int path_count = 10);
  std::vector<PathInfo> getPathsThroughInstance(odb::dbInst* inst, int max_paths = 5);
  std::vector<PathInfo> getCriticalPathsThroughNet(odb::dbNet* net, int max_paths = 5);

  long long getMaxRadius(odb::dbInst* instance);
  double evaluateTimingSlack(const std::string& pin_name);
  double evaluateTotalNegativeSlack();
  double evaluateWorstNegativeSlack();
  double evaluateDesignArea();
  double evaluateTotalWireLength();
  double evaluateTotalPower();
    // Buffer insertion
    MoveResult insertBuffer(const std::string& net_name,
                          const std::string& buffer_type,
                          int x, int y);
  //get the area of a master cell
  double getMasterArea(const std::string& master_name);
  //get the logger
  utl::Logger* logger() {return logger_;}

private:
    // OpenROAD database handles
  odb::dbDatabase* db_;
  odb::dbBlock* block_;
  sta::Sta* sta_;
  sta::dbSta* dbsta_;
  grt::GlobalRouter* router_;
  rsz::Resizer* resizer_;
  utl::Logger* logger_;
  
  // Spare cell tracking
  std::vector<std::shared_ptr<SpareCell> > spare_cells_;
  std::unordered_map<std::string, std::vector<std::shared_ptr<SpareCell>>> spare_cells_by_type_;

  // Checkpoints
  std::unordered_map<std::string, std::pair<double, double>> checkpoints_;
    
  // Helper methods
  void capturePreMoveMetrics(double& tns, double& area, double& wire_length);
  MoveResult calculateMoveImpact(double pre_tns, double pre_area, double pre_wire);
  bool isSpareCell(odb::dbInst* inst) const;
  std::string getSpareType(odb::dbMaster* master) const;
  void performSpareCellSwap(odb::dbInst* target_inst, std::shared_ptr<SpareCell> spare, odb::dbMaster* new_master);
  std::pair<std::string, std::string> splitPinName(const std::string& pin_name);
  
  // Helper for path extraction
  void extractPathElements(sta::Path* path,
                           std::vector<odb::dbInst*>& instances,
                           std::vector<odb::dbNet*>& nets);


  
};

}
#endif // ECO_DESIGN_MANAGER_H
