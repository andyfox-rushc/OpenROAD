#pragma once
#include <random>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>


namespace odb {
class dbDatabase;
}

namespace sta {
class Sta;
}

namespace utl {
class Logger;
}

namespace ord {
class OpenRoad;
}

using namespace ord;

namespace eco {

  class EcoDesignManager;
  class EcoQLearningTrainer;  
  
class RlEco
{
public:
  RlEco();
  ~RlEco();

  void init(OpenRoad* openroad);

  OpenRoad* getOpenRoad(){
    printf("RlEco.h getting openroad %p", openroad_);
    return openroad_;
  }
  // Spare cell identification
  void identifySpareCells();
  int getNumSpareCells() const;

  bool successfullyInit() { return successfully_init_;}
  // RL training
  void trainAgent(int episodes, 
                  double learning_rate = 0.001,
                  double epsilon = 0.1,
                  double gamma = 0.99);
  
  void saveModel(const char* filepath);
  void loadModel(const char* filepath, size_t feature_size, size_t action_size);
  

  // RL configuration
  void setReplayBufferSize(int size);
  void setBatchSize(int size);
  void setHiddenLayers(const std::vector<int>& layers);
  void setRewardWeights(double timing_weight,
                        double wirelength_weight,
                        double congestion_weight);

  //handy debug
  void reportFeatures();
  void applyTrainedModel();
  
  // Checkpointing
  void enableCheckpoints(const char* directory, int interval);
  void disableCheckpoints();

  // Fast inference
  std::string getCurrentTimestamp() const;
  // Model management
  bool hasTrainedModel() const { return has_trained_model_; }
  void setModelTrained(bool trained) { has_trained_model_ = trained; }
  void reportSpareCells();

private:
  OpenRoad* openroad_;
  odb::dbDatabase* db_;
  sta::Sta* sta_;
  utl::Logger* logger_;

  std::shared_ptr<EcoDesignManager> design_manager_;
  std::unique_ptr<EcoQLearningTrainer> trainer_;

  std::mt19937 generator_;
  
  // Configuration
  //  std::vector<size_t> hidden_layers_ = {128, 64, 32};
  std::vector<size_t> hidden_layers_ = {256, 256, 256};  
  size_t replay_buffer_size_ = 10000;
  size_t batch_size_ = 32;
  float timing_weight_ = 1.0;
  float wirelength_weight_ = 0.5;
  float congestion_weight_ = 0.3;
  
  // Checkpointing
  bool checkpoint_enabled_ = false;
  std::string checkpoint_dir_ = "./checkpoints";
  int checkpoint_interval_ = 100;

  //Training dataset management
  // Training dataset management
  std::vector<std::string> training_scenarios_;
  std::map<std::string, double> metrics_history_;
  bool has_trained_model_ = false;
  
  // Performance tracking
  std::chrono::high_resolution_clock::time_point inference_start_;
  std::chrono::high_resolution_clock::time_point inference_end_;
  

  void trackSpareCells();

  bool successfully_init_ = false;
  
};

}//namespace eco


