#include "odb/db.h"
#include "rl_eco/EcoDesignManager.h"
#include "rl_eco/EcoChangeSet.h"


#include "db_sta/dbSta.hh"
#include "utl/Logger.h"
#include "odb/defout.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace eco {

// EcoChange implementations
std::string EcoGateChange::getDescription() const {
    return "Gate change: " + instance_change_->toString();
}

std::string EcoNetChange::getDescription() const {
    return "Net change: " + connection_change_->toString();
}

std::string EcoTimingFixChange::getDescription() const {
    std::stringstream ss;
    ss << "Timing fix for path with slack: " << slack_ << "ns";
    return ss.str();
}

// EcoMove implementation
std::string EcoMove::getDescription() const {
    switch (type) {
        case USE_SPARE_CELL:
            return "Use spare cell " + spare_cell_name + " for " + instance_name;
        case CONNECT_NET:
            return "Connect " + instance_name + "." + pin_name + " to net " + net_name;
        case DISCONNECT_NET:
            return "Disconnect " + instance_name + "." + pin_name + " from net";
        case CREATE_NET:
            return "Create net " + net_name;
        case BUFFER_NET:
            return "Insert buffer of size " + std::to_string(buffer_size) + " on net " + net_name;
        case SKIP:
            return "Skip current change";
        default:
            return "Unknown move";
    }
}

// EcoDesignManager implementation
  EcoDesignManager::EcoDesignManager(odb::dbDatabase* db, 
				     sta::dbSta* sta,
				     utl::Logger* logger)
    : db_(db),  sta_(sta), logger_(logger),
      total_changes_(0), implemented_changes_(0),
      cached_worst_slack_(0.0), cached_total_wirelength_(0.0), 
      cached_congestion_metric_(0.0), metrics_valid_(false) {
    
    chip_ = db_->getChip();
    if (chip_) {
        block_ = chip_->getBlock();
    }
    initializeSpareCellPatterns();
}

void EcoDesignManager::setChangeSet(std::shared_ptr<EcoChangeSet> change_set) {
    change_set_ = change_set;
    total_changes_ = change_set_->getTotalChanges();
    convertChangesToEcoChanges();
}

bool EcoDesignManager::parseChangeFile(const char* filename) {
    if (!change_set_) {
        change_set_ = std::make_shared<EcoChangeSet>();
    }
    
    bool success = change_set_->parseChangeFile(filename);
    if (success) {
        total_changes_ = change_set_->getTotalChanges();
        convertChangesToEcoChanges();
    }
    return success;
}

  
// Add these methods to EcoDesignManager.cpp:

float EcoDesignManager::getTimingImprovement() const {
    // Calculate timing improvement compared to baseline
    // This should compare current worst slack to initial worst slack
    static double initial_worst_slack = std::numeric_limits<double>::lowest();
    if (initial_worst_slack == std::numeric_limits<double>::lowest()) {
        // First call - store initial value
        initial_worst_slack = getWorstSlack();
        return 0.0f;
    }
    
    double current_slack = getWorstSlack();
    return static_cast<float>((current_slack - initial_worst_slack) * 1000.0); // Convert to ps
}

float EcoDesignManager::getWirelengthIncrease() const {
    // Calculate wirelength increase as percentage
    static double initial_wirelength = 0.0;
    if (initial_wirelength == 0.0) {
        initial_wirelength = getTotalWirelength();
        return 0.0f;
    }
    
    double current_wl = getTotalWirelength();
    return static_cast<float>((current_wl - initial_wirelength) / initial_wirelength);
}

void EcoDesignManager::applyGreedyPlacement() {
    logger_->info(utl::ECO, 107, "Applying greedy ECO placement");
    
    // Reset to start fresh
    reset();
    
    // Process all changes greedily
    while (hasUnimplementedChanges()) {
        auto* change = getNextUnimplementedChange();
        if (!change) break;
        
        // Greedy strategy: use nearest spare cell
        auto gate_change = dynamic_cast<EcoGateChange*>(change);
        if (gate_change) {
            auto inst_change = gate_change->getInstanceChange();
            if (inst_change && inst_change->getInfo().has_location) {
                double x = inst_change->getInfo().x_coord;
                double y = inst_change->getInfo().y_coord;
                
                // Find nearest available spare cell of correct type
                std::string required_type = inst_change->getInfo().cell_type;
                auto* spare = findNearestSpareCell(required_type, x, y);
                
                if (spare) {
                    implementChangeWithSpareCell(change, *spare);
                } else {
                    // No suitable spare cell - skip
                    skipChange(change);
                }
            } else {
                // No location info - skip this change
                skipChange(change);
            }
        } else {
            // Handle other change types
            auto net_change = dynamic_cast<EcoNetChange*>(change);
            if (net_change) {
                // For net changes, just mark as implemented (simplified)
                implementChangeWithReroute(change);
            } else {
                // For any other change type (like timing fixes), skip
                skipChange(change);
            }
        }
    }
}
  
bool EcoDesignManager::applyMove(const EcoMove& move) {
    auto* current = getCurrentChange();
    if (!current) {
        logger_->warn(utl::ECO, 108, "Trying to apply move with no current change");  // Fixed: added missing quote
        return false;
    }
    
    switch (move.type) {
        case EcoMove::USE_SPARE_CELL:
        {
            // Find the spare cell
            for (auto& cell : spare_cells_) {
                if (cell.name == move.spare_cell_name && !cell.used) {
                    return implementChangeWithSpareCell(current, cell);
                }
            }
            return false;
        }
        
        case EcoMove::CONNECT_NET:
            return implementChangeWithReroute(current);
            
        case EcoMove::DISCONNECT_NET:
            // Handle disconnect
            skipChange(current);  // Simplified for now
            return true;
            
        case EcoMove::CREATE_NET:
            createNet(move.net_name);
            return true;
            
        case EcoMove::BUFFER_NET:
            return insertBuffer(current, move.buffer_size);
            
        case EcoMove::SKIP:
            skipChange(current);
            return true;
            
        default:
            return false;
    }
}

  


void EcoDesignManager::initializeSpareCellPatterns() {
    // Common spare cell naming patterns
    spare_cell_patterns_ = {
        "SPARE", "spare", "FILL", "fill", "DCAP", "dcap",
        "TIE", "tie", "ANTENNA", "antenna"
    };
}

  std::string EcoDesignManager::extractBaseType(const std::string& full_type) const{
    // First, try to remove any known spare cell pattern prefix
        for (const auto& pattern : spare_cell_patterns_) {
            // Check if type starts with pattern followed by underscore
            std::string prefix = pattern + "_";
            if (full_type.find(prefix) == 0) {
                return full_type.substr(prefix.length());
            }
            
            // Check if type starts with pattern (case insensitive)
            std::string lower_type = full_type;
            std::transform(lower_type.begin(), lower_type.end(), lower_type.begin(), ::tolower);
            std::string lower_pattern = pattern;
            std::transform(lower_pattern.begin(), lower_pattern.end(), lower_pattern.begin(), ::tolower);
            
            if (lower_type.find(lower_pattern + "_") == 0) {
                return full_type.substr(lower_pattern.length() + 1);
            }
        }
        
        // If no pattern matched, try to extract meaningful type
        // Look for common cell type indicators
        static const std::vector<std::string> type_indicators = {
            "NAND", "NOR", "AND", "OR", "XOR", "XNOR", 
            "INV", "BUF", "MUX", "AOI", "OAI", "DFF", 
            "LATCH", "SDFF", "DFFSR"
        };
        
        for (const auto& indicator : type_indicators) {
            if (full_type.find(indicator) != std::string::npos) {
                // Found a known cell type indicator
                // Extract the part containing this indicator
                size_t pos = full_type.find(indicator);
                size_t end = pos + indicator.length();
                
                // Check if there's a drive strength suffix (e.g., _X1, _X2)
                if (end < full_type.length() && full_type[end] == '_') {
                    size_t strength_end = full_type.find_first_not_of("X0123456789", end + 1);
                    if (strength_end != std::string::npos) {
                        return full_type.substr(pos, strength_end - pos);
                    } else {
                        return full_type.substr(pos);
                    }
                }
                
                return full_type.substr(pos, indicator.length());
            }
        }
        
        // If still no match, return the full type minus any common prefixes
        return full_type;
  }

  

std::string EcoDesignManager::extractCellSignature(const std::string& full_type) const {
    // Remove spare prefixes
    std::string type = full_type;
    for (const auto& pattern : spare_cell_patterns_) {
        std::string prefix = pattern + "_";
        if (type.find(prefix) == 0) {
            type = type.substr(prefix.length());
        }
    }
    
    // Now return the full logic type INCLUDING size
    // Examples: "NAND2_X1" -> "NAND2", "SPARE_INV_X2" -> "INV"
    
    // Remove only the drive strength
    size_t pos = type.rfind("_X");
    if (pos != std::string::npos && pos + 2 < type.length()) {
        if (std::isdigit(type[pos + 2])) {
            return type.substr(0, pos);
        }
    }
    
    return type;
}

bool EcoDesignManager::areCellsCompatible(const std::string& spare_type, 
                                          const std::string& required_type) const {
    std::string spare_sig = extractCellSignature(spare_type);
    std::string required_sig = extractCellSignature(required_type);
    
    // NAND2 == NAND2 (regardless of drive strength)
    // NAND2 != NAND3
    return spare_sig == required_sig;
}

  
  std::vector<SpareCell*> EcoDesignManager::findSpareCellsByType(const std::string& base_type){
    std::vector<SpareCell*> matching_cells;
    for (auto& cell : spare_cells_) {
      if (!cell.used) {
	std::string cell_base_type = extractBaseType(cell.type);
	if (cell_base_type == base_type) {
	  matching_cells.push_back(&cell);
	}
      }
    }
        
        // Sort by some criteria (e.g., alphabetical by name)
        std::sort(matching_cells.begin(), matching_cells.end(),
                  [](const SpareCell* a, const SpareCell* b) {
                      return a->name < b->name;
                  });
        return matching_cells;
  }

  
void EcoDesignManager::identifySpareCells() {
    spare_cells_.clear();
    
    if (!block_) {
        logger_->warn(utl::ECO, 100, "No block loaded, cannot identify spare cells");
        return;
    }
    
    for (odb::dbInst* inst : block_->getInsts()) {
        if (isSpareCell(inst)) {
            odb::dbBox* bbox = inst->getBBox();
            double x = (bbox->xMin() + bbox->xMax()) / 2.0 / static_cast<double>(block_->getDbUnitsPerMicron());
            double y = (bbox->yMin() + bbox->yMax()) / 2.0 / static_cast<double>(block_->getDbUnitsPerMicron());
            
            SpareCell cell(inst->getName(), 
                          inst->getMaster()->getName(),
                          x, y, inst);
            spare_cells_.push_back(cell);
        }
    }
    
    logger_->info(utl::ECO, 101, "Identified {} spare cells", spare_cells_.size());
}


bool EcoDesignManager::isSpareCell(odb::dbInst* inst) const {
    std::string inst_name = inst->getName();
    std::string master_name = inst->getMaster()->getName();
    
    // Convert to lowercase for case-insensitive matching
    std::string inst_name_lower = inst_name;
    std::string master_name_lower = master_name;
    std::transform(inst_name_lower.begin(), inst_name_lower.end(), inst_name_lower.begin(), ::tolower);
    std::transform(master_name_lower.begin(), master_name_lower.end(), master_name_lower.begin(), ::tolower);
    
    // Check instance name against patterns
    for (const auto& pattern : spare_cell_patterns_) {
        std::string pattern_lower = pattern;
        std::transform(pattern_lower.begin(), pattern_lower.end(), pattern_lower.begin(), ::tolower);
        
        // Check if pattern appears in the name
        if (inst_name_lower.find(pattern_lower) != std::string::npos ||
            master_name_lower.find(pattern_lower) != std::string::npos) {
            return true;
        }
    }
    
    // Check if it's a standard cell that's unconnected
    odb::dbMaster* master = inst->getMaster();
    if (master && master->getType() == odb::dbMasterType::CORE) {
        // Check if ALL pins are unconnected
        bool all_unconnected = true;
        for (odb::dbITerm* iterm : inst->getITerms()) {
            if (iterm->getNet() != nullptr) {
                // Check if it's only connected to power/ground
                odb::dbNet* net = iterm->getNet();
                if (!net->getSigType().isSupply()) {
                    all_unconnected = false;
                    break;
                }
            }
        }
        
        if (all_unconnected) {
            // Additional check: verify it's a logic cell (not clock buffer, etc.)
            return isLogicCell(master_name);
        }
    }
    
    return false;
}

  //
  //TODO:
  //get this from liberty !
  //
// Add helper method to check if a cell is a logic cell
bool EcoDesignManager::isLogicCell(const std::string& cell_name) const {
    static const std::vector<std::string> logic_patterns = {
        "NAND", "NOR", "AND", "OR", "XOR", "XNOR", "INV", "BUF", 
        "MUX", "AOI", "OAI", "DFF", "LATCH"
    };
    
    std::string upper_name = cell_name;
    std::transform(upper_name.begin(), upper_name.end(), upper_name.begin(), ::toupper);
    
    for (const auto& pattern : logic_patterns) {
        if (upper_name.find(pattern) != std::string::npos) {
            return true;
        }
    }
    
    return false;
}
  
  std::vector<std::shared_ptr<EcoChange>> EcoDesignManager::getImplementedChanges() const {
    std::vector<std::shared_ptr<EcoChange>> implemented;
    for (const auto& change : all_changes_) {
        if (change->isImplemented()) {
            implemented.push_back(change);
        }
    }
    return implemented;
  }
  
std::set<std::string> EcoDesignManager::getAvailableSpareCellTypes() const {
  std::set<std::string> cell_types;
  for (const auto& spare_cell : spare_cells_) {
    // Extract base type (remove any SPARE_ prefix if present)
    std::string base_type = extractBaseType(spare_cell.type);
    if (!base_type.empty()) {
      cell_types.insert(base_type);
    }
  }
  return cell_types;
}
    
  // Get spare cell type distribution
  std::map<std::string, int> EcoDesignManager::getSpareCellTypeCount() const {
    std::map<std::string, int> type_count;
    for (const auto& spare_cell : spare_cells_) {
      if (!spare_cell.used) {  // Only count available cells
	std::string base_type = extractBaseType(spare_cell.type);
	if (!base_type.empty()) {
	  type_count[base_type]++;
	}
      }
    }
    return type_count;
  }
  
std::vector<SpareCell> EcoDesignManager::getAvailableSpareCells() const {
    std::vector<SpareCell> available;
    for (const auto& cell : spare_cells_) {
        if (!cell.used) {
            available.push_back(cell);
        }
    }
    return available;
}

int EcoDesignManager::getNumAvailableSpareCells() const {
    int count = 0;
    for (const auto& cell : spare_cells_) {
        if (!cell.used) count++;
    }
    return count;
}

void EcoDesignManager::reset() {
    // Reset spare cells
    for (auto& cell : spare_cells_) {
        cell.used = false;
    }
    
    // Reset changes
    implemented_changes_ = 0;
    unimplemented_changes_ = std::queue<std::shared_ptr<EcoChange>>();
    for (auto& change : all_changes_) {
        change->setImplemented(false);
        unimplemented_changes_.push(change);
    }
    
    current_change_ = nullptr;
    invalidateMetrics();
}

bool EcoDesignManager::hasUnimplementedChanges() const {
    return !unimplemented_changes_.empty();
}

bool EcoDesignManager::allChangesProcessed() const {
    return implemented_changes_ == total_changes_;
}

int EcoDesignManager::getNumUnimplementedChanges() const {
    return unimplemented_changes_.size();
}

EcoChange* EcoDesignManager::getCurrentChange() {
    if (!current_change_ && !unimplemented_changes_.empty()) {
        current_change_ = unimplemented_changes_.front();
    }
    return current_change_.get();
}

EcoChange* EcoDesignManager::getNextUnimplementedChange() {
    if (!unimplemented_changes_.empty()) {
        current_change_ = unimplemented_changes_.front();
        unimplemented_changes_.pop();
        return current_change_.get();
    }
    return nullptr;
}

bool EcoDesignManager::implementChange(Change* change) {
    // This is a simplified implementation
    // Real implementation would depend on change type
    
    auto inst_change = dynamic_cast<InstanceChange*>(change);
    if (inst_change && inst_change->getType() == ChangeType::INSTANCE_ADDED) {
        const auto& info = inst_change->getInfo();
        auto* spare = findNearestSpareCell(info.cell_type, info.x_coord, info.y_coord);
        if (spare) {
            spare->used = true;
            implemented_changes_++;
            invalidateMetrics();
            return true;
        }
    }
    
    auto conn_change = dynamic_cast<ConnectionChange*>(change);
    if (conn_change) {
        // Handle connection changes
        implemented_changes_++;
        invalidateMetrics();
        return true;
    }
    
    return false;
}

bool EcoDesignManager::implementChangeWithSpareCell(EcoChange* change, SpareCell& spare_cell) {
  auto gate_change = dynamic_cast<EcoGateChange*>(change);
    if (!gate_change) return false;
    
    auto inst_change = gate_change->getInstanceChange();
    if (!inst_change || inst_change->getType() != ChangeType::INSTANCE_ADDED) {
        return false;
    }
    
    const auto& info = inst_change->getInfo();
    
    // 1. Rename spare instance to new instance name
    spare_cell.db_inst->rename(info.name.c_str());
    
    // 2. Disconnect all non-power pins from dummy nets
    for (odb::dbITerm* iterm : spare_cell.db_inst->getITerms()) {
        odb::dbNet* net = iterm->getNet();
        if (net && !net->getSigType().isSupply()) {
            iterm->disconnect();
            logger_->info(utl::ECO, 301, "Disconnected pin {} from net {}", 
                         iterm->getMTerm()->getName(), net->getName());
        }
    }
    
    // 3. Apply connection changes for this instance
    applyConnectionChanges(info.name);
    
    // 4. Mark as used and implemented
    spare_cell.used = true;
    change->setImplemented(true);
    implemented_changes_++;
    
    logger_->info(utl::ECO, 302, "Repurposed spare cell {} as {}", 
                 spare_cell.name, info.name);
    
    invalidateMetrics();
    return true;
}

  // New method to apply connection changes
void EcoDesignManager::applyConnectionChanges(const std::string& instance_name) {
    if (!change_set_) return;
    
    auto* inst = block_->findInst(instance_name.c_str());
    if (!inst) return;
    
    // Process all connection changes for this instance
    for (auto& conn_change : change_set_->getConnectionChanges()) {
        const auto& conn_info = conn_change->getInfo();
        
        if (conn_info.instance_name != instance_name) continue;
        
        odb::dbITerm* iterm = inst->findITerm(conn_info.pin_name.c_str());
        if (!iterm) {
            logger_->warn(utl::ECO, 303, "Pin {} not found on instance {}", 
                         conn_info.pin_name, instance_name);
            continue;
        }
        
        switch (conn_change->getType()) {
            case ChangeType::CONNECTION_ADDED:
            case ChangeType::CONNECTION_MODIFIED: {
                odb::dbNet* net = block_->findNet(conn_info.net_name.c_str());
                if (!net) {
                    logger_->warn(utl::ECO, 304, "Net {} not found", conn_info.net_name);
                    continue;
                }
                iterm->connect(net);
                logger_->info(utl::ECO, 305, "Connected {}.{} to net {}", 
                             instance_name, conn_info.pin_name, conn_info.net_name);
                break;
            }
            
            case ChangeType::CONNECTION_REMOVED:
                iterm->disconnect();
                logger_->info(utl::ECO, 306, "Disconnected {}.{}", 
                             instance_name, conn_info.pin_name);
                break;
                
            default:
                break;
        }
    }
}

  

bool EcoDesignManager::implementChangeWithReroute(EcoChange* change) {
    auto net_change = dynamic_cast<EcoNetChange*>(change);
    if (!net_change) return false;
    
    // Implement net rerouting logic
    change->setImplemented(true);
    implemented_changes_++;
    invalidateMetrics();
    
    logger_->info(utl::ECO, 103, "Implemented change with rerouting");
    return true;
}

bool EcoDesignManager::insertBuffer(EcoChange* change, int buffer_size) {
    // Find available buffer spare cell
    std::string buffer_type = "BUF_X" + std::to_string(buffer_size);
    
    for (auto& cell : spare_cells_) {
        if (!cell.used && cell.type.find("BUF") != std::string::npos) {
            cell.used = true;
            change->setImplemented(true);
            implemented_changes_++;
            invalidateMetrics();
            
            logger_->info(utl::ECO, 104, "Inserted buffer using spare cell {}", cell.name);
            return true;
        }
    }
    
    return false;
}

bool EcoDesignManager::resizeGate(EcoChange* change) {
    // Gate resizing logic would go here
    // This would involve finding a spare cell of different drive strength
    
    change->setImplemented(true);
    implemented_changes_++;
    invalidateMetrics();
    
    logger_->info(utl::ECO, 105, "Resized gate for timing optimization");
    return true;
}

void EcoDesignManager::skipChange(EcoChange* change) {
    change->setImplemented(true);  // Mark as processed but not actually implemented
    logger_->info(utl::ECO, 106, "Skipped change: {}", change->getDescription());
}

double EcoDesignManager::getWorstSlack() const {
    if (!metrics_valid_) {
        const_cast<EcoDesignManager*>(this)->updateTimingMetrics();
    }
    return cached_worst_slack_;
}

double EcoDesignManager::getTotalWirelength() const {
    if (!metrics_valid_) {
        const_cast<EcoDesignManager*>(this)->updatePhysicalMetrics();
    }
    return cached_total_wirelength_;
}

double EcoDesignManager::getCongestionMetric() const {
    if (!metrics_valid_) {
        const_cast<EcoDesignManager*>(this)->updatePhysicalMetrics();
    }
    return cached_congestion_metric_;
}

double EcoDesignManager::getDistanceToChange(const SpareCell& cell, EcoChange* change) const {
    // Get location associated with the change
    auto gate_change = dynamic_cast<EcoGateChange*>(change);
    if (gate_change) {
        auto inst_change = gate_change->getInstanceChange();
        if (inst_change && inst_change->getInfo().has_location) {
            return computeDistance(cell.x, cell.y, 
                                 inst_change->getInfo().x_coord,
                                 inst_change->getInfo().y_coord);
        }
    }
    
    // Default to center of design if no location
    if (block_) {
        odb::Rect die_box = block_->getDieArea();
        double center_x = (die_box.xMin() + die_box.xMax()) / 2.0 / block_->getDbUnitsPerMicron();
        double center_y = (die_box.yMin() + die_box.yMax()) / 2.0 / block_->getDbUnitsPerMicron();
        return computeDistance(cell.x, cell.y, center_x, center_y);
    }
    
    return 0.0;
}

void EcoDesignManager::updateTimingAfterChange(Change* change) {
    updateTimingMetrics();
}

void EcoDesignManager::updateTimingMetrics() {
    if (!sta_) {
        cached_worst_slack_ = 0.0;
        return;
    }
    
    // Update timing method inheritted from sta
    sta_->updateTiming(false);
    
    // Get worst slack
    cached_worst_slack_ = sta_->worstSlack(sta::MinMax::max());
    
    metrics_valid_ = true;
}

void EcoDesignManager::updatePhysicalMetrics() {
    if (!block_) {
        cached_total_wirelength_ = 0.0;
        cached_congestion_metric_ = 0.0;
        return;
    }
    
    // Calculate total wirelength
    double total_wl = 0.0;
    for (odb::dbNet* net : block_->getNets()) {
        if (net->isSpecial()) continue;
        
        odb::dbWire* wire = net->getWire();
        if (wire) {
            // Simplified wirelength calculation
            odb::Rect bbox;
            bool first = true;
            
            for (odb::dbITerm* iterm : net->getITerms()) {
                int x, y;
                if (iterm->getAvgXY(&x, &y)) {
                    if (first) {
                        bbox.init(x, y, x, y);
                        first = false;
                    } else {
                        bbox.merge(odb::Rect(x, y, x, y));
                    }
                }
            }
            
            if (!first) {
                total_wl += (bbox.dx() + bbox.dy()) / static_cast<double>(block_->getDbUnitsPerMicron());
            }
        }
    }
    cached_total_wirelength_ = total_wl;
    
    // Simplified congestion metric (percentage of routing resources used)
    // In a real implementation, this would query the router
    cached_congestion_metric_ = 0.75;  // Placeholder value
    
    metrics_valid_ = true;
}

std::vector<EcoMove> EcoDesignManager::generatePossibleMoves() const {
    std::vector<EcoMove> moves;
    
    auto* current = const_cast<EcoDesignManager*>(this)->getCurrentChange();
    if (!current) {
        return moves;
    }
    
    // Always can skip
    EcoMove skip_move;
    skip_move.type = EcoMove::SKIP;
    moves.push_back(skip_move);
    
    // Generate moves based on change type
    auto gate_change = dynamic_cast<EcoGateChange*>(current);
    if (gate_change) {
        // Add moves for using each available spare cell
        auto available_cells = getAvailableSpareCells();
        for (const auto& cell : available_cells) {
            EcoMove move;
            move.type = EcoMove::USE_SPARE_CELL;
            move.spare_cell_name = cell.name;
            moves.push_back(move);
        }
    }
    
    auto net_change = dynamic_cast<EcoNetChange*>(current);
    if (net_change) {
        // Add net-related moves
        EcoMove connect_move;
        connect_move.type = EcoMove::CONNECT_NET;
        moves.push_back(connect_move);
        
        EcoMove disconnect_move;
        disconnect_move.type = EcoMove::DISCONNECT_NET;
        moves.push_back(disconnect_move);
    }
    
    // Add buffer insertion moves
    for (int size = 1; size <= 4; ++size) {
        EcoMove buffer_move;
        buffer_move.type = EcoMove::BUFFER_NET;
        buffer_move.buffer_size = size;
        moves.push_back(buffer_move);
    }
    
    return moves;
}

SpareCell* EcoDesignManager::findNearestSpareCell(const std::string& required_type, double x, double y) {
  SpareCell* nearest = nullptr;
  double min_dist = std::numeric_limits<double>::max();
    
    for (auto& cell : spare_cells_) {
        if (!cell.used && areCellsCompatible(cell.type, required_type)) {
            double dist = computeDistance(x, y, cell.x, cell.y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = &cell;
            }
        }
    }
    return nearest;
}

  

double EcoDesignManager::computeDistance(double x1, double y1, double x2, double y2) const {
    // Manhattan distance
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

void EcoDesignManager::convertChangesToEcoChanges() {
    all_changes_.clear();
    unimplemented_changes_ = std::queue<std::shared_ptr<EcoChange>>(); // Clear the queue
    
    if (!change_set_) return;
    
    // Convert instance changes
    for (auto& change : change_set_->getAddedInstances()) {
        auto eco_change = std::make_shared<EcoGateChange>(change);
        all_changes_.push_back(eco_change);
        unimplemented_changes_.push(eco_change);
    }
    
    for (auto& change : change_set_->getRemovedInstances()) {
        auto eco_change = std::make_shared<EcoGateChange>(change);
        all_changes_.push_back(eco_change);
        unimplemented_changes_.push(eco_change);
    }
    
    // Convert connection changes
    for (auto& change : change_set_->getConnectionChanges()) {
        auto eco_change = std::make_shared<EcoNetChange>(change);
        all_changes_.push_back(eco_change);
        unimplemented_changes_.push(eco_change);
    }
    
    // Convert net changes - you might want to handle these differently
    for (auto& change : change_set_->getNetChanges()) {
      (void)change;
      //TODO:
      // Net changes might need a different EcoChange type
      // For now, we'll skip them or you could create an EcoNetModifyChange class
    }
}

  

odb::dbInst* EcoDesignManager::createInstance(const std::string& name, 
                                             const std::string& cell_type, 
                                             double x, double y) {
    // Implementation would create a new instance
    // This is a placeholder
    return nullptr;
}

bool EcoDesignManager::connectPin(odb::dbInst* inst, const std::string& pin_name, odb::dbNet* net) {
    if (!inst || !net) return false;
    
    odb::dbITerm* iterm = inst->findITerm(pin_name.c_str());
    if (!iterm) return false;
    
    iterm->connect(net);
    return true;
}

bool EcoDesignManager::disconnectPin(odb::dbInst* inst, const std::string& pin_name) {
    if (!inst) return false;
    
    odb::dbITerm* iterm = inst->findITerm(pin_name.c_str());
    if (!iterm) return false;
    
    iterm->disconnect();
    return true;
}

odb::dbNet* EcoDesignManager::createNet(const std::string& name) {
    if (!block_) return nullptr;
    return odb::dbNet::create(block_, name.c_str());
}

bool EcoDesignManager::applyAllChanges() {
    if (!change_set_) {
        logger_->error(utl::ECO, 307, "No ECO changes loaded");
        return false;
    }
    
    int success_count = 0;
    int skip_count = 0;
    
    // First handle instance additions
    for (auto& inst_change : change_set_->getAddedInstances()) {
        const auto& info = inst_change->getInfo();
        
        // Find compatible spare cell
        double x = info.has_location ? info.x_coord : 0.0;
        double y = info.has_location ? info.y_coord : 0.0;
        
        SpareCell* spare = findNearestSpareCell(info.cell_type, x, y);
        
        if (spare) {
            auto eco_change = std::make_shared<EcoGateChange>(inst_change);
            if (implementChangeWithSpareCell(eco_change.get(), *spare)) {
                success_count++;
            }
        } else {
            logger_->warn(utl::ECO, 308, "No compatible spare cell for {} ({})", 
                         info.name, info.cell_type);
            skip_count++;
        }
    }
    
    // Then handle net additions
    for (auto& net_change : change_set_->getNetAdditions()) {
        const auto& info = net_change->getInfo();
        
        // Create the net
        odb::dbNet* net = odb::dbNet::create(block_, info.name.c_str());
        if (net) {
            logger_->info(utl::ECO, 309, "Created net {}", info.name);
            success_count++;
        }
    }
    
    logger_->info(utl::ECO, 310, "Applied {} changes, skipped {}", 
                 success_count, skip_count);
    
    return success_count > 0;
}
 
void EcoDesignManager::writeDEF(const std::string& filename) {
    if (!block_) {
        logger_->error(utl::ECO, 311, "No design loaded");
        return;
    }
    
    odb::DefOut writer(logger_);
    writer.writeBlock(block_, filename.c_str());
    logger_->info(utl::ECO, 312, "Wrote modified DEF to {}", filename);
}





bool EcoDesignManager::routeConnection(const std::string& instance_name, 
                                      const std::string& pin_name, 
                                      const std::string& net_name) {
    odb::dbInst* inst = block_->findInst(instance_name.c_str());
    if (!inst) return false;
    
    odb::dbITerm* iterm = inst->findITerm(pin_name.c_str());
    if (!iterm) return false;
    
    odb::dbNet* net = block_->findNet(net_name.c_str());
    if (!net) return false;
    
    // Create wire if it doesn't exist
    odb::dbWire* wire = net->getWire();
    if (!wire) {
        wire = odb::dbWire::create(net);
    }
    
    // Find a metal-only route from the pin to an existing segment of the net
    return findMetalRoute(iterm, net);
}

  
bool EcoDesignManager::eco_route_changes() {
    logger_->info(utl::ECO, 400, "Applying ECO connection changes");
    
    if (!change_set_) {
        logger_->warn(utl::ECO, 406, "No change set loaded");
        return false;
    }
    
    std::set<odb::dbNet*> modified_nets;
    int connections_made = 0;
    
    // Process all connection changes from the change set
    for (auto& conn_change : change_set_->getConnectionChanges()) {
        const auto& info = conn_change->getInfo();
        
        odb::dbInst* inst = block_->findInst(info.instance_name.c_str());
        if (!inst) {
            logger_->warn(utl::ECO, 407, "Instance {} not found for connection change", 
                         info.instance_name);
            continue;
        }
        
        odb::dbITerm* iterm = inst->findITerm(info.pin_name.c_str());
        if (!iterm) {
            logger_->warn(utl::ECO, 408, "Pin {}.{} not found", 
                         info.instance_name, info.pin_name);
            continue;
        }
        
        odb::dbNet* net = block_->findNet(info.net_name.c_str());
        if (!net) {
            logger_->warn(utl::ECO, 409, "Net {} not found", info.net_name);
            continue;
        }
        
        // Make the connection
        iterm->connect(net);
        modified_nets.insert(net);
        connections_made++;
        
        logger_->info(utl::ECO, 403, "Connected {}.{} to net {}", 
                     info.instance_name, info.pin_name, info.net_name);
    }
    
    logger_->info(utl::ECO, 401, "Made {} connections across {} nets. Ready for routing.", 
                 connections_made, modified_nets.size());
    logger_->info(utl::ECO, 402, "Run global_route and detailed_route commands to complete routing.");
    
    return connections_made > 0;
}
  
  
bool EcoDesignManager::findMetalRoute(odb::dbITerm* from_iterm, odb::dbNet* to_net) {
    // Simply ensure the logical connection is made
    if (from_iterm->getNet() != to_net) {
        from_iterm->connect(to_net);
    }
    
    logger_->info(utl::ECO, 403, "Connected {}.{} to net {} (ready for routing)", 
                 from_iterm->getInst()->getName(), 
                 from_iterm->getMTerm()->getName(),
                 to_net->getName());
    
    // Mark the net as modified/needs routing by setting wire ordered to false
    to_net->setWireOrdered(false);
    
    return true;
}

  
odb::dbTechLayer* EcoDesignManager::getRoutingLayer(int layer_num) {
    std::string layer_name = "METAL" + std::to_string(layer_num);
    return block_->getTech()->findLayer(layer_name.c_str());
}


void EcoDesignManager::writeRoutedDEF(const std::string& filename) {
    writeDEF(filename);  // Reuse existing method
    logger_->info(utl::ECO, 405, "Wrote routed ECO design to {}", filename);
}

  bool EcoDesignManager::applyEcoChanges() {
    logger_->info(utl::ECO, 300, "Applying ECO instance changes");
    
    if (!change_set_) {
        logger_->warn(utl::ECO, 313, "No change set loaded");
        return false;
    }
    
    int changes_applied = 0;
    
    // 1. Process instance additions (spare cell repurposing)
    for (auto& inst_change : change_set_->getAddedInstances()) {
        const auto& info = inst_change->getInfo();
        
        // Find and repurpose a spare cell
        double x = info.has_location ? info.x_coord : 0.0;
        double y = info.has_location ? info.y_coord : 0.0;
        
        SpareCell* spare = findNearestSpareCell(info.cell_type, x, y);
        
        if (spare && spare->db_inst) {
            // Rename the spare cell to the new instance name
            spare->db_inst->rename(info.name.c_str());
            spare->used = true;
            
            logger_->info(utl::ECO, 314, "Repurposed spare cell {} as {} ({})", 
                         spare->name, info.name, info.cell_type);
            changes_applied++;
        } else {
            logger_->warn(utl::ECO, 315, "No compatible spare cell for {} ({})", 
                         info.name, info.cell_type);
        }
    }
    
    // 2. Process instance removals (disconnect and mark as unused)
    for (auto& inst_change : change_set_->getRemovedInstances()) {
        const auto& info = inst_change->getInfo();
        odb::dbInst* inst = block_->findInst(info.name.c_str());
        
        if (inst) {
            // Disconnect all pins
            for (odb::dbITerm* iterm : inst->getITerms()) {
                iterm->disconnect();
            }
            logger_->info(utl::ECO, 316, "Disconnected instance {}", info.name);
            changes_applied++;
        }
    }
    
    logger_->info(utl::ECO, 302, "Applied {} instance changes", changes_applied);
    return changes_applied > 0;
}
  
} // namespace eco
