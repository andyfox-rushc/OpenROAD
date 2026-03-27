#include "odb/db.h"
#include "rl_eco/EcoTypes.h"
#include "rl_eco/RlEco.h"
#include "rl_eco/EcoDesignManager.h"
#include "rl_eco/EcoQLearningTrainer.h"
#include "rl_eco/EcoRLEnvironment.h"



#include "ord/OpenRoad.hh"

#include "sta/Sta.hh"
#include "utl/Logger.h"

#include <filesystem>
#include <chrono>
#include <cstring>
#include <random>
#include <algorithm>
#include <tcl.h>

using namespace ord;
namespace eco {

RlEco::RlEco() = default;
RlEco::~RlEco() = default;

  
void RlEco::init(OpenRoad* openroad)
{
  successfully_init_ = false;
 if (!openroad) {
        throw std::runtime_error("Null OpenRoad pointer provided to RlEco::init");
    }
     // Store the openroad pointer
    openroad_ = openroad;
    
    // Get the logger from openroad
    logger_ = openroad->getLogger();
    if (!logger_) {
        throw std::runtime_error("Failed to get logger from OpenRoad");
    }
    
    logger_->info(utl::ECO, 1, "Initializing RL-based ECO flow");
    
    // Get the database from OpenROAD
    odb::dbDatabase* db = openroad_->getDb();
    if (!db) {
        logger_->warn(utl::ECO, 100, "No database loaded. Please read a design first.");
        return;
    }
    
    // Verify we have a valid design
    odb::dbChip* chip = db->getChip();
    if (!chip) {
        logger_->warn(utl::ECO, 101, "No chip in database. Please read a design first.");
        return;
    }
    
    odb::dbBlock* block = chip->getBlock();
    if (!block) {
        logger_->warn(utl::ECO, 102, "No block in database. Please read a design first.");
        return;
    }
    
    // Get STA from OpenROAD (optional - warn if not available)
    sta::dbSta* sta = openroad_->getSta();
    if (!sta) {
        logger_->warn(utl::ECO, 103, "STA not available - timing-driven ECO features will be disabled");
    }
    
    // Initialize the design manager
    try {
      grt::GlobalRouter* router = openroad_-> getGlobalRouter();
      rsz::Resizer* resizer = openroad_ -> getResizer();
      design_manager_ = std::make_unique<EcoDesignManager>(db, sta, router, resizer,logger_);
      logger_->info(utl::ECO, 2, "Design manager initialized successfully");
    } catch (const std::exception& e) {
        logger_->error(utl::ECO, 104, "Failed to initialize design manager: {}", e.what());
        return;
    }
    
    
    // Log some basic design statistics
    if (design_manager_ && block) {
        logger_->info(utl::ECO, 3, "Design: {}", block->getName());
        logger_->info(utl::ECO, 4, "Number of instances: {}", block->getInsts().size());
        logger_->info(utl::ECO, 5, "Number of nets: {}", block->getNets().size());
    }

    std::random_device rd;
    generator_ = std::mt19937(rd());
    logger_->info(utl::ECO, 6, "RL-based ECO flow initialized successfully");
    successfully_init_ = true;
}

  

  
void RlEco::identifySpareCells()
{
  
  if (!design_manager_) {
    logger_->error(utl::RLE, 3, "ECO RL not initialized");
  }
  design_manager_->identifySpareCells();
  int num_spare = design_manager_->getSpareCells().size();
  logger_->info(utl::RLE, 4, "Identified {} spare cells", num_spare);
}

  
void RlEco::trainAgent(int episodes, 
                       double learning_rate,
                       double epsilon,
                       double gamma)
{
  if (!design_manager_ ) {
    logger_->error(utl::RLE, 5, "No design manager");
  }
  
  // Create Q-learning configuration
  QLearningConfig config;
  config.hidden_layers = hidden_layers_;
  config.learning_rate = learning_rate;
  config.epsilon_start = epsilon;
  config.epsilon_end = epsilon * 0.01;  // End at 1% of start value
  config.gamma = gamma;
  config.batch_size = batch_size_;
  config.replay_buffer_size = replay_buffer_size_;
  config.max_episodes = episodes;
  config.default_action_size = 500;
  // Set reward weights
  config.timing_weight = timing_weight_;
  config.wirelength_weight = wirelength_weight_;
  config.congestion_weight = congestion_weight_;
  
  // Create trainer
  trainer_ = std::make_unique<EcoQLearningTrainer>(design_manager_, config,logger_);
  
  logger_->info(utl::RLE, 6, "Starting RL training:");
  logger_->info(utl::RLE, 7, "  Episodes: {}", episodes);
  logger_->info(utl::RLE, 8, "  Learning rate: {}", learning_rate);
  logger_->info(utl::RLE, 9, "  Initial epsilon: {}", epsilon);
  logger_->info(utl::RLE, 10, "  Hidden layers: {} {} ", 
                hidden_layers_[0], hidden_layers_[1]);
  
  // Enable checkpointing if configured
  if (checkpoint_enabled_) {
    // Create checkpoint directory if it doesn't exist
    std::filesystem::create_directories(checkpoint_dir_);
  }
  
  // Custom training loop with checkpointing
  for (int episode = 0; episode < episodes; ++episode) {
    trainer_->trainEpisode();
    
    // Progress reporting
    if (episode % 100 == 0) {
      trainer_->printTrainingProgress(episode);
    }
    
    // Checkpointing
    if (checkpoint_enabled_ && episode > 0 && episode % checkpoint_interval_ == 0) {
      std::string checkpoint_path = checkpoint_dir_ + "/checkpoint_episode_" + 
                                    std::to_string(episode) + ".ckpt";
      trainer_->saveAgent(checkpoint_path);
      logger_->info(utl::RLE, 11, "Saved checkpoint at episode {}", episode);
    }
    
    // Allow TCL events
    if (episode % 10 == 0) {
      Tcl_DoOneEvent(TCL_DONT_WAIT);
    }
  }
   // Print final training report
  logger_->info(utl::RLE, 12, "\n{}", trainer_->generateTrainingReport());
}


void RlEco::saveModel(const char* filepath)
{
  if (!trainer_) {
    logger_->error(utl::RLE, 16, "No trained model to save");
  }
  trainer_->saveAgent(filepath);
  logger_->info(utl::RLE, 17, "Saved model to {}", filepath);
}


  void RlEco::loadModel(const char* filepath, size_t feature_size, size_t action_size)
  {
    if (!design_manager_) {
      logger_->error(utl::RLE, 18, "ECO RL not initialized");
    }
    if (trainer_ ){
      logger_->error(utl::RLE, 19,
		     "ECO RL in inference mode, please start from scratch. No trainer");
    }
    QLearningConfig config;
    //no exploration
    config.epsilon_start = 0.0;
    config.epsilon_end = 0.0;
    config.hidden_layers = hidden_layers_;
    trainer_ = std::make_unique<EcoQLearningTrainer>(design_manager_,
						     config,
						     logger_,
						     
						     filepath,
						     feature_size,
						     action_size);
  }


void RlEco::applyTrainedModel()
{
  if (trainer_){
    trainer_ -> applyInference();
  }
}


void RlEco::reportFeatures(){
      double tns = design_manager_ -> evaluateTotalNegativeSlack();
      double wns = design_manager_ -> evaluateWorstNegativeSlack();
      printf("TNS %.10f\n",tns);
      printf("WNS %.10f\n",wns);
      if (trainer_){
	printf("Max Action Size %d", trainer_ -> max_action_size());
	printf("State Size %d", trainer_ ->  state_size());
      }
      else{
	printf("Cannot report state features & action size unless trainer available. Run training then report\n");
      }
}

// Configuration setters
void RlEco::setReplayBufferSize(int size)
{
  replay_buffer_size_ = size;
}

void RlEco::setBatchSize(int size)
{
  batch_size_ = size;
}

void RlEco::setHiddenLayers(const std::vector<int>& layers)
{
  printf("In set hidden layers\n");
  hidden_layers_.clear();
  for (int size : layers) {
    hidden_layers_.push_back(static_cast<size_t>(size));
  }
}

void RlEco::setRewardWeights(double timing_weight,
                             double wirelength_weight,
                             double congestion_weight)
{
  timing_weight_ = timing_weight;
  wirelength_weight_ = wirelength_weight;
  congestion_weight_ = congestion_weight;
}

// Checkpointing
void RlEco::enableCheckpoints(const char* directory, int interval)
{
  checkpoint_enabled_ = true;
  checkpoint_dir_ = directory;
  checkpoint_interval_ = interval;
}

void RlEco::disableCheckpoints()
{
  checkpoint_enabled_ = false;
}



int RlEco::getNumSpareCells() const
{
  return (design_manager_ -> getSpareCells().size());
}



// Helper to get current timestamp
std::string RlEco::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}


void RlEco::reportSpareCells()
{
    if (!design_manager_) {
        logger_->error(utl::ECO, 1100, "Design manager not initialized");
        return;
    }
    
    // Make sure spare cells have been identified
    design_manager_->identifySpareCells();
    
    // Call the design manager's reporting method
    design_manager_->reportSpareCells();
}

}
