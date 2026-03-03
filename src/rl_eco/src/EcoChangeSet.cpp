// EcoChangeSet.cpp
// Implementation of ECO change set classes

#include "rl_eco/EcoChangeSet.h"
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>


namespace eco {

// Utility function implementations
std::string changeTypeToString(ChangeType type) {
    switch (type) {
        case ChangeType::INSTANCE_ADDED: return "INSTANCE_ADDED";
        case ChangeType::INSTANCE_REMOVED: return "INSTANCE_REMOVED";
        case ChangeType::CONNECTION_ADDED: return "CONNECTION_ADDED";
        case ChangeType::CONNECTION_REMOVED: return "CONNECTION_REMOVED";
        case ChangeType::CONNECTION_MODIFIED: return "CONNECTION_MODIFIED";
        case ChangeType::NET_ADDED: return "NET_ADDED";
        case ChangeType::NET_REMOVED: return "NET_REMOVED";
        case ChangeType::NET_MODIFIED: return "NET_MODIFIED";
        default: return "UNKNOWN";
    }
}

// InstanceChange implementation
InstanceChange::InstanceChange(ChangeType type, const Info& info)
    : Change(type), info_(info) {}

std::string InstanceChange::toString() const {
    std::stringstream ss;
    ss << changeTypeToString(type_) << ": " << info_.name 
       << " (" << info_.cell_type << ")";
    if (info_.has_location) {
        ss << " @ (" << std::fixed << std::setprecision(2) 
           << info_.x_coord << ", " << info_.y_coord << ")";
    }
    return ss.str();
}

// ConnectionChange implementation
ConnectionChange::ConnectionChange(ChangeType type, const Info& info)
    : Change(type), info_(info) {}

std::string ConnectionChange::toString() const {
    std::stringstream ss;
    ss << changeTypeToString(type_) << ": " 
       << info_.instance_name << "." << info_.pin_name;
    
    switch (type_) {
        case ChangeType::CONNECTION_ADDED:
            ss << " -> " << info_.net_name;
            break;
        case ChangeType::CONNECTION_REMOVED:
            ss << " -X- " << info_.net_name;
            break;
        case ChangeType::CONNECTION_MODIFIED:
            ss << ": " << info_.old_net_name << " -> " << info_.net_name;
            break;
        default:
            break;
    }
    return ss.str();
}

std::string ConnectionChange::getKey() const {
    return info_.instance_name + "." + info_.pin_name;
}

// NetChange implementation
NetChange::NetChange(ChangeType type, const Info& info)
    : Change(type), info_(info) {}

std::string NetChange::toString() const {
    std::stringstream ss;
    ss << changeTypeToString(type_) << ": " << info_.name 
       << " [" << info_.connections.size() << " connections]";
    return ss.str();
}

// EcoChangeSet implementation

bool EcoChangeSet::parseChangeFile(const char* filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open ECO change file: " << filename << std::endl;
        return false;
    }
    
    clear();  // Clear any existing changes
    
    std::string line;
    int line_number = 0;
    
    while (std::getline(file, line)) {
        line_number++;
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == '/') {
            continue;
        }
        
        // Remove leading/trailing whitespace
        size_t first = line.find_first_not_of(" \t\r\n");
        size_t last = line.find_last_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        line = line.substr(first, last - first + 1);
        
        // Parse the line
        std::istringstream iss(line);
        std::string command;
        iss >> command;
        
        // Convert command to uppercase for case-insensitive matching
        std::transform(command.begin(), command.end(), command.begin(), ::toupper);
        
        try {
            if (command == "ADD_INSTANCE" || command == "INSTANCE_ADD") {
                parseInstanceAdd(iss, line_number);
            }
            else if (command == "REMOVE_INSTANCE" || command == "INSTANCE_REMOVE" || 
                     command == "DELETE_INSTANCE") {
                parseInstanceRemove(iss, line_number);
            }
            else if (command == "CONNECT" || command == "ADD_CONNECTION") {
                parseConnectionAdd(iss, line_number);
            }
            else if (command == "DISCONNECT" || command == "REMOVE_CONNECTION") {
                parseConnectionRemove(iss, line_number);
            }
            else if (command == "MODIFY_CONNECTION" || command == "CHANGE_CONNECTION") {
                parseConnectionModify(iss, line_number);
            }
            else if (command == "ADD_NET" || command == "CREATE_NET") {
                parseNetAdd(iss, line_number);
            }
            else if (command == "REMOVE_NET" || command == "DELETE_NET") {
                parseNetRemove(iss, line_number);
            }
            else if (command == "MODIFY_NET") {
                parseNetModify(iss, line_number);
            }
            else {
                std::cerr << "Warning: Unknown command '" << command 
                         << "' at line " << line_number << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error parsing line " << line_number << ": " 
                     << e.what() << std::endl;
            std::cerr << "Line content: " << line << std::endl;
            file.close();
            return false;
        }
    }
    
    file.close();
    
    // Validate the parsed changes
    if (!validate()) {
        std::cerr << "Error: ECO changes validation failed" << std::endl;
        auto errors = getValidationErrors();
        for (const auto& error : errors) {
            std::cerr << "  - " << error << std::endl;
        }
        return false;
    }
    
    std::cout << "Successfully parsed " << getTotalChanges() 
              << " ECO changes from " << filename << std::endl;
    
    return true;
}

// Helper parsing methods (add these as private methods to EcoChangeSet class)
void EcoChangeSet::parseInstanceAdd(std::istringstream& iss, int line_number) {
    InstanceChange::Info info;
    iss >> info.name >> info.cell_type;
    
    if (info.name.empty() || info.cell_type.empty()) {
        throw std::runtime_error("Missing instance name or cell type");
    }
    
    // Optional coordinates
    std::string x_str, y_str;
    if (iss >> x_str >> y_str) {
        try {
            info.x_coord = std::stod(x_str);
            info.y_coord = std::stod(y_str);
            info.has_location = true;
        } catch (...) {
            // If conversion fails, assume no coordinates provided
            info.has_location = false;
        }
    }
    
    addInstanceAdded(info);
}

void EcoChangeSet::parseInstanceRemove(std::istringstream& iss, int line_number) {
    InstanceChange::Info info;
    iss >> info.name;
    
    if (info.name.empty()) {
        throw std::runtime_error("Missing instance name");
    }
    
    // Cell type is optional for removal
    iss >> info.cell_type;
    
    addInstanceRemoved(info);
}

void EcoChangeSet::parseConnectionAdd(std::istringstream& iss, int line_number) {
    ConnectionChange::Info info;
    std::string pin_spec;
    iss >> pin_spec >> info.net_name;
    
    // Parse instance.pin format
    size_t dot_pos = pin_spec.find('.');
    if (dot_pos == std::string::npos) {
        throw std::runtime_error("Invalid pin specification (expected format: instance.pin)");
    }
    
    info.instance_name = pin_spec.substr(0, dot_pos);
    info.pin_name = pin_spec.substr(dot_pos + 1);
    
    if (info.instance_name.empty() || info.pin_name.empty() || info.net_name.empty()) {
        throw std::runtime_error("Missing connection information");
    }
    
    addConnectionAdded(info);
}

void EcoChangeSet::parseConnectionRemove(std::istringstream& iss, int line_number) {
    ConnectionChange::Info info;
    std::string pin_spec;
    iss >> pin_spec;
    
    // Parse instance.pin format
    size_t dot_pos = pin_spec.find('.');
    if (dot_pos == std::string::npos) {
        throw std::runtime_error("Invalid pin specification (expected format: instance.pin)");
    }
    
    info.instance_name = pin_spec.substr(0, dot_pos);
    info.pin_name = pin_spec.substr(dot_pos + 1);
    
    // Net name is optional for removal (can be determined from current connection)
    iss >> info.net_name;
    
    if (info.instance_name.empty() || info.pin_name.empty()) {
        throw std::runtime_error("Missing connection information");
    }
    
    addConnectionRemoved(info);
}

void EcoChangeSet::parseConnectionModify(std::istringstream& iss, int line_number) {
    ConnectionChange::Info info;
    std::string pin_spec;
    iss >> pin_spec >> info.old_net_name >> info.net_name;
    
    // Parse instance.pin format
    size_t dot_pos = pin_spec.find('.');
    if (dot_pos == std::string::npos) {
        throw std::runtime_error("Invalid pin specification (expected format: instance.pin)");
    }
    
    info.instance_name = pin_spec.substr(0, dot_pos);
    info.pin_name = pin_spec.substr(dot_pos + 1);
    
    if (info.instance_name.empty() || info.pin_name.empty() || 
        info.old_net_name.empty() || info.net_name.empty()) {
        throw std::runtime_error("Missing connection modification information");
    }
    
    addConnectionModified(info);
}


void EcoChangeSet::parseNetAdd(std::istringstream& iss, int line_number) {
    NetChange::Info info;
    iss >> info.name;
    
    if (info.name.empty()) {
        throw std::runtime_error("Missing net name");
    }
    
    // Optional: read connection list
    std::string conn;
    while (iss >> conn) {
        // Parse instance.pin format
        size_t dot_pos = conn.find('.');
        if (dot_pos != std::string::npos) {
            NetChange::PinConnection pin_conn;
            pin_conn.instance_name = conn.substr(0, dot_pos);
            pin_conn.pin_name = conn.substr(dot_pos + 1);
            info.connections.push_back(pin_conn);
        }
    }
    
    addNetAdded(info);
}
  
void EcoChangeSet::parseNetRemove(std::istringstream& iss, int line_number) {
    NetChange::Info info;
    iss >> info.name;
    
    if (info.name.empty()) {
        throw std::runtime_error("Missing net name");
    }
    
    addNetRemoved(info);
}
  
void EcoChangeSet::parseNetModify(std::istringstream& iss, int line_number) {
    NetChange::Info info;
    iss >> info.name;
    
    if (info.name.empty()) {
        throw std::runtime_error("Missing net name");
    }
    
    // Read new connection list
    std::string conn;
    while (iss >> conn) {
        // Parse instance.pin format
        size_t dot_pos = conn.find('.');
        if (dot_pos != std::string::npos) {
            NetChange::PinConnection pin_conn;
            pin_conn.instance_name = conn.substr(0, dot_pos);
            pin_conn.pin_name = conn.substr(dot_pos + 1);
            info.connections.push_back(pin_conn);
        }
    }
    
    addNetModified(info);
}


  
  
void EcoChangeSet::addChange(std::shared_ptr<Change> change) {
    changes_.push_back(change);
    change_index_[change->getKey()] = change;
    invalidateStats();
}

void EcoChangeSet::addInstanceAdded(const InstanceChange::Info& info) {
    addChange(std::make_shared<InstanceChange>(ChangeType::INSTANCE_ADDED, info));
}

void EcoChangeSet::addInstanceRemoved(const InstanceChange::Info& info) {
    addChange(std::make_shared<InstanceChange>(ChangeType::INSTANCE_REMOVED, info));
}

void EcoChangeSet::addConnectionAdded(const ConnectionChange::Info& info) {
    addChange(std::make_shared<ConnectionChange>(ChangeType::CONNECTION_ADDED, info));
}

void EcoChangeSet::addConnectionRemoved(const ConnectionChange::Info& info) {
    addChange(std::make_shared<ConnectionChange>(ChangeType::CONNECTION_REMOVED, info));
}

void EcoChangeSet::addConnectionModified(const ConnectionChange::Info& info) {
    addChange(std::make_shared<ConnectionChange>(ChangeType::CONNECTION_MODIFIED, info));
}

void EcoChangeSet::addNetAdded(const NetChange::Info& info) {
    addChange(std::make_shared<NetChange>(ChangeType::NET_ADDED, info));
}

void EcoChangeSet::addNetRemoved(const NetChange::Info& info) {
    addChange(std::make_shared<NetChange>(ChangeType::NET_REMOVED, info));
}

void EcoChangeSet::addNetModified(const NetChange::Info& info) {
    addChange(std::make_shared<NetChange>(ChangeType::NET_MODIFIED, info));
}

std::vector<std::shared_ptr<Change>> EcoChangeSet::getChangesByType(ChangeType type) const {
    std::vector<std::shared_ptr<Change>> result;
    for (const auto& change : changes_) {
        if (change->getType() == type) {
            result.push_back(change);
        }
    }
    return result;
}

std::shared_ptr<Change> EcoChangeSet::findChangeByKey(const std::string& key) const {
    auto it = change_index_.find(key);
    return (it != change_index_.end()) ? it->second : nullptr;
}

void EcoChangeSet::updateStats() const {
    if (stats_valid_) return;
    
    num_instances_added_ = getChangesByType(ChangeType::INSTANCE_ADDED).size();
    num_instances_removed_ = getChangesByType(ChangeType::INSTANCE_REMOVED).size();
    
    num_connections_changed_ = 
        getChangesByType(ChangeType::CONNECTION_ADDED).size() +
        getChangesByType(ChangeType::CONNECTION_REMOVED).size() +
        getChangesByType(ChangeType::CONNECTION_MODIFIED).size();
    
    num_nets_changed_ = 
        getChangesByType(ChangeType::NET_ADDED).size() +
        getChangesByType(ChangeType::NET_REMOVED).size() +
        getChangesByType(ChangeType::NET_MODIFIED).size();
    
    stats_valid_ = true;
}

size_t EcoChangeSet::getNumInstancesAdded() const {
    updateStats();
    return num_instances_added_;
}

size_t EcoChangeSet::getNumInstancesRemoved() const {
    updateStats();
    return num_instances_removed_;
}

size_t EcoChangeSet::getNumConnectionChanges() const {
    updateStats();
    return num_connections_changed_;
}

size_t EcoChangeSet::getNumNetChanges() const {
    updateStats();
    return num_nets_changed_;
}

std::vector<std::shared_ptr<InstanceChange>> EcoChangeSet::getAddedInstances() const {
    std::vector<std::shared_ptr<InstanceChange>> result;
    for (const auto& change : getChangesByType(ChangeType::INSTANCE_ADDED)) {
        result.push_back(std::dynamic_pointer_cast<InstanceChange>(change));
    }
    return result;
}

std::vector<std::shared_ptr<InstanceChange>> EcoChangeSet::getRemovedInstances() const {
    std::vector<std::shared_ptr<InstanceChange>> result;
    for (const auto& change : getChangesByType(ChangeType::INSTANCE_REMOVED)) {
        result.push_back(std::dynamic_pointer_cast<InstanceChange>(change));
    }
    return result;
}

std::vector<std::shared_ptr<ConnectionChange>> EcoChangeSet::getConnectionChanges() const {
    std::vector<std::shared_ptr<ConnectionChange>> result;
    for (const auto& change : changes_) {
        if (change->getType() == ChangeType::CONNECTION_ADDED ||
            change->getType() == ChangeType::CONNECTION_REMOVED ||
            change->getType() == ChangeType::CONNECTION_MODIFIED) {
            result.push_back(std::dynamic_pointer_cast<ConnectionChange>(change));
        }
    }
    return result;
}

std::vector<std::shared_ptr<NetChange>> EcoChangeSet::getNetChanges() const {
    std::vector<std::shared_ptr<NetChange>> result;
    for (const auto& change : changes_) {
        if (change->getType() == ChangeType::NET_ADDED ||
            change->getType() == ChangeType::NET_REMOVED ||
            change->getType() == ChangeType::NET_MODIFIED) {
            result.push_back(std::dynamic_pointer_cast<NetChange>(change));
        }
    }
    return result;
}
  
std::vector<std::shared_ptr<NetChange>> EcoChangeSet::getNetAdditions() const {
    std::vector<std::shared_ptr<NetChange>> result;
    for (const auto& change : changes_) {
      if (change->getType() == ChangeType::NET_ADDED){
            result.push_back(std::dynamic_pointer_cast<NetChange>(change));
        }
    }
    return result;
}

EcoRequirements EcoChangeSet::analyzeRequirements() const {
    EcoRequirements req;
    
    // Count spare cells needed by type
    for (const auto& inst : getAddedInstances()) {
        req.spare_cells_by_type[inst->getInfo().cell_type]++;
        req.total_spare_cells++;
    }
    
    // Analyze rewiring requirements
    std::unordered_map<std::string, std::string> removed_connections;
    std::unordered_map<std::string, std::string> added_connections;
    
    for (const auto& conn : getConnectionChanges()) {
        const auto& info = conn->getInfo();
        std::string key = info.instance_name + "." + info.pin_name;
        
        if (conn->getType() == ChangeType::CONNECTION_REMOVED) {
            removed_connections[key] = info.net_name;
        } else if (conn->getType() == ChangeType::CONNECTION_ADDED) {
            added_connections[key] = info.net_name;
        } else if (conn->getType() == ChangeType::CONNECTION_MODIFIED) {
            req.rewiring_pairs.push_back({
                key + " : " + info.old_net_name,
                key + " : " + info.net_name
            });
        }
    }
    
    // Match removed and added connections for the same pin
    for (const auto& [key, old_net] : removed_connections) {
        auto it = added_connections.find(key);
        if (it != added_connections.end()) {
            req.rewiring_pairs.push_back({
                key + " : " + old_net,
                key + " : " + it->second
            });
        }
    }
    
    // Estimate routing resources (simplified)
    req.estimated_wire_length = req.rewiring_pairs.size() * 100.0; // 100 units per rewire
    req.estimated_vias = req.rewiring_pairs.size() * 2; // 2 vias per rewire
    
    // Assume we might need metal layers 1-3 for ECO routing
    if (!req.rewiring_pairs.empty()) {
        req.metal_layers_needed = {1, 2, 3};
    }
    
    return req;
}

std::vector<std::string> EcoChangeSet::getRequiredCellTypes() const {
    std::unordered_set<std::string> cell_types;
    for (const auto& inst : getAddedInstances()) {
        cell_types.insert(inst->getInfo().cell_type);
    }
    return std::vector<std::string>(cell_types.begin(), cell_types.end());
}

bool EcoChangeSet::validate() const {
    return checkConnectionConsistency() && checkNetConsistency();
}

bool EcoChangeSet::checkConnectionConsistency() const {
    // Check that we don't have conflicting connection changes
    std::unordered_map<std::string, int> connection_ops;
    
    for (const auto& conn : getConnectionChanges()) {
        std::string key = conn->getKey();
        connection_ops[key]++;
        
        // Multiple operations on the same pin might indicate an issue
        if (connection_ops[key] > 2) {
            return false;
        }
    }
    
    return true;
}

bool EcoChangeSet::checkNetConsistency() const {
    // Check that net changes are consistent with connection changes
    // This is a simplified check - a full implementation would be more thorough
    return true;
}

std::vector<std::string> EcoChangeSet::getValidationErrors() const {
    std::vector<std::string> errors;
    
    if (!checkConnectionConsistency()) {
        errors.push_back("Inconsistent connection changes detected");
    }
    
    if (!checkNetConsistency()) {
        errors.push_back("Net changes inconsistent with connection changes");
    }
    
    return errors;
}

std::string EcoChangeSet::generateSummary() const {
    std::stringstream ss;
    ss << "ECO Change Set Summary\n";
    ss << "=====================\n";
    ss << "Total changes: " << getTotalChanges() << "\n";
    ss << "  - Instances added: " << getNumInstancesAdded() << "\n";
    ss << "  - Instances removed: " << getNumInstancesRemoved() << "\n";
    ss << "  - Connection changes: " << getNumConnectionChanges() << "\n";
    ss << "  - Net changes: " << getNumNetChanges() << "\n";
    
    auto req = analyzeRequirements();
    ss << "\nImplementation Requirements:\n";
    ss << "  - Total spare cells needed: " << req.total_spare_cells << "\n";
    if (!req.spare_cells_by_type.empty()) {
        ss << "  - Spare cells by type:\n";
        for (const auto& [type, count] : req.spare_cells_by_type) {
            ss << "    * " << type << ": " << count << "\n";
        }
    }
    ss << "  - Rewiring operations: " << req.rewiring_pairs.size() << "\n";
    
    return ss.str();
}

std::string EcoChangeSet::generateDetailedReport() const {
    std::stringstream ss;
    ss << generateSummary();
    ss << "\nDetailed Changes:\n";
    ss << "-----------------\n";
    
    // Group changes by type
    for (int i = 0; i < 8; ++i) {
        ChangeType type = static_cast<ChangeType>(i);
        auto changes = getChangesByType(type);
        if (!changes.empty()) {
            ss << "\n" << changeTypeToString(type) << " (" << changes.size() << "):\n";
            for (const auto& change : changes) {
                ss << "  - " << change->toString() << "\n";
            }
        }
    }
    
    return ss.str();
}

void EcoChangeSet::exportToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    
    file << generateDetailedReport();
    
    // Add machine-readable section
    file << "\n\n# Machine-Readable Format\n";
    file << "# TYPE,KEY,DETAILS\n";
    for (const auto& change : changes_) {
        file << changeTypeToString(change->getType()) << ","
             << change->getKey() << ","
             << change->toString() << "\n";
    }
}

void EcoChangeSet::clear() {
    changes_.clear();
    change_index_.clear();
    invalidateStats();
}

// NetlistComparator implementation
std::unique_ptr<EcoChangeSet> NetlistComparator::compare(
    const NetlistData& r0_data, const NetlistData& r1_data) {
    
    auto changes = std::make_unique<EcoChangeSet>();
    
    compareInstances(r0_data, r1_data, *changes);
    compareConnections(r0_data, r1_data, *changes);
    compareNets(r0_data, r1_data, *changes);
    
    return changes;
}

void NetlistComparator::compareInstances(
    const NetlistData& r0, const NetlistData& r1, EcoChangeSet& changes) {
    
    // Find added instances
    for (const auto& [name, info] : r1.instances) {
        if (r0.instances.find(name) == r0.instances.end()) {
            changes.addInstanceAdded(info);
        }
    }
    
    // Find removed instances
    for (const auto& [name, info] : r0.instances) {
        if (r1.instances.find(name) == r1.instances.end()) {
            changes.addInstanceRemoved(info);
        }
    }
}

void NetlistComparator::compareConnections(
    const NetlistData& r0, const NetlistData& r1, EcoChangeSet& changes) {
    
    // Find added and modified connections
    for (const auto& [key, info] : r1.connections) {
        auto it = r0.connections.find(key);
        if (it == r0.connections.end()) {
            changes.addConnectionAdded(info);
        } else if (it->second.net_name != info.net_name) {
            ConnectionChange::Info mod_info = info;
            mod_info.old_net_name = it->second.net_name;
            changes.addConnectionModified(mod_info);
        }
    }
    
    // Find removed connections
    for (const auto& [key, info] : r0.connections) {
        if (r1.connections.find(key) == r1.connections.end()) {
            changes.addConnectionRemoved(info);
        }
    }
}

void NetlistComparator::compareNets(
    const NetlistData& r0, const NetlistData& r1, EcoChangeSet& changes) {
    
    // Find added nets
    for (const auto& [name, info] : r1.nets) {
        if (r0.nets.find(name) == r0.nets.end()) {
            changes.addNetAdded(info);
        }
    }
    
    // Find removed nets
    for (const auto& [name, info] : r0.nets) {
        if (r1.nets.find(name) == r1.nets.end()) {
            changes.addNetRemoved(info);
        }
    }
    
    // Find modified nets (different connection count)
    for (const auto& [name, r1_info] : r1.nets) {
        auto it = r0.nets.find(name);
        if (it != r0.nets.end()) {
            if (it->second.connections.size() != r1_info.connections.size()) {
                changes.addNetModified(r1_info);
            }
        }
    }
}


}//namespace eco

