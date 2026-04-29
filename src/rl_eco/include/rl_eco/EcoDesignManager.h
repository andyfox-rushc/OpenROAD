#ifndef ECO_DESIGN_MANAGER_H
#define ECO_DESIGN_MANAGER_H

#include <memory>
#include <vector>
#include <set>
#include <map>
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
  class dbNetwork;
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


  double deltaRemovedWires();
  double deltaAddedWires();

  // Utilities (EcoRLDbUtil)
  bool isDFFRS(odb::dbInst* db_inst,
	       odb::dbITerm* &d_in,
	       odb::dbITerm* &q_out);
  
  odb::dbITerm* getPinByMTermName(odb::dbInst*, const std::string& pin_name);
  bool getSpareMux2(
		    std::shared_ptr<SpareCell>& mux_spare,
		    odb::dbITerm*& mux2xn_d0,
		    odb::dbITerm*& mux2xn_d1,
		    odb::dbITerm*& mux2xn_sel,
		    odb::dbITerm*& mux2xn_op);

  bool RetimeableGate(odb::dbInst* inst) const;
  bool instConnectedToIos(odb::dbInst* inst) const;
  bool getFanin1Level(odb::dbITerm* ip_pin, std::vector<odb::dbITerm*>& ip_pins);
  bool getFanout1Level(odb::dbITerm* op_pin, std::vector<odb::dbITerm*>& op_pins);
  void updateTiming(bool full);
  bool areCompatibleMasters(const std::string& spare_master, const std::string& inst_master) const;
  bool arePinCompatible(const std::string& master1_name, const std::string& master_name) const;
  odb::dbNet* findOrCreateConstantNet(bool constant_phase,
				      sta::dbNetwork* db_network);
  odb::dbNet* mkFlatNet();
  bool singletonNet(odb::dbNet* cur_net) const;
  void getFlopPins(odb::dbInst* flop,
		   odb::dbITerm* &d_pin,
		   odb::dbITerm* &q_pin,
		   odb::dbITerm* &qn_pin,
		   odb::dbITerm* &clk_pin,
		   odb::dbITerm* &r_pin,
		   odb::dbITerm* &s_pin);

  //Path splitting moves
  
  MoveResult previewPathSplit(odb::dbITerm* start,
			      odb::dbITerm* end,
			      std::shared_ptr<SpareCell> mux_spare,
			      odb::dbITerm* mux2xn_d0,
			      odb::dbITerm* mux2xn_d1,
			      odb::dbITerm* mux2xn_sel,
			      odb::dbITerm* mux2xn_op,			      
			      std::vector<std::pair<odb::dbInst*,
			      std::shared_ptr<SpareCell>> > &gates_to_duplicate);

  MoveResult performPathSplit(odb::dbITerm* start,
			      odb::dbITerm* end,
			      
			      std::shared_ptr<SpareCell> mux_spare,			      
			      odb::dbITerm* mux2xn_d0,
			      odb::dbITerm* mux2xn_d1,
			      odb::dbITerm* mux2xn_sel,
			      odb::dbITerm* mux2xn_op,			      
			      std::vector<std::pair<odb::dbInst*,
			      std::shared_ptr<SpareCell>> > &gates_to_duplicate);

  //Resizing moves
  
  MoveResult previewResize(odb::dbInst* inst,
			   odb::dbInst* spare_inst);


  MoveResult performResize(odb::dbInst* inst,
			   odb::dbInst* spare_inst);

  /*
    Retiming moves
  */






  bool getSpareFlops(std::string master_type, unsigned count,
		     std::vector<std::shared_ptr<SpareCell> >&spare_flops);
  
  bool performForwardRetime(odb::dbNet* clk_net,
			    odb::dbNet* reset_net,
			    odb::dbNet* set_net,
					      
			    odb::dbITerm* op_pin,
			    std::vector<std::pair<odb::dbITerm*,
			    std::shared_ptr<SpareCell> > >
			    &fanout_op_pins
			    );

  MoveResult previewBackwardRetimingMove(odb::dbInst* flop,
					 odb::dbITerm* d,
					 odb::dbITerm* q);

  
  MoveResult performBackwardRetime(
			     odb::dbInst* orig_flop,
			     std::vector<std::pair<odb::dbITerm*,
			     std::shared_ptr<SpareCell> > >
			     &fanin_ip_pins
			     );
  
  bool identifyRetimingMoves(std::vector<
			     std::tuple<
			     odb::dbInst*,
			     odb::dbITerm*,
			     odb::dbITerm* ,
			     bool > >&
			     candidates);
  


  
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
  odb::dbInst* findInst(std::string& inst_name);
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
  std::vector<std::shared_ptr<SpareCell> > findSpareCellsNear(//pass in cells used in prior states here
							      odb::dbInst* instance,
							      long long radius);

  std::set<std::string> getCompatibleMasters(const std::string& master_name) const;

  std::string extractBaseName(const std::string& master_name) const;

  
  

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
  
  bool isFeasibleMaster(const std::string& name, SpareCellsDictionary& spare_cells_dictionary);
  void markUsed(const std::string& name, SpareCellsDictionary& spare_cells_dictionary);
  void populateSpareCellsDictionary(SpareCellsDictionary& spare_cells_dictionary);
  void wireLengthCache(bool value){use_wire_length_cache_=value;}

private:
    // OpenROAD database handles
  odb::dbDatabase* db_;
  odb::dbBlock* block_;
  sta::Sta* sta_;
  sta::dbSta* dbsta_;
  grt::GlobalRouter* router_;
  rsz::Resizer* resizer_;
  utl::Logger* logger_;
  sta::dbNetwork* dbnetwork_;
  
  //Keep track of nets that changed for area calc
  std::map<std::string, odb::dbNet*> pin2net_;
  std::unordered_set<odb::dbInst*> free_set_;
  
  
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

  //routines for managing free cells
  bool freeInst(odb::dbInst*) const;
  void removeFromFreeSet(odb::dbInst*);
  void markInstFree(odb::dbInst*);
  
  
  // Helper for path extraction
  void extractPathElements(sta::Path* path,
                           std::vector<odb::dbInst*>& instances,
                           std::vector<odb::dbNet*>& nets);

  odb::dbNet* gnd_net_=nullptr;
  odb::dbNet* pwr_net_=nullptr;
  
  bool use_wire_length_cache_=false;
  double wire_length_cached_=0.0;
  double removed_wire_cached_=0.0;
  double added_wire_cached_=0.0;
  
  
};

}
#endif // ECO_DESIGN_MANAGER_H
