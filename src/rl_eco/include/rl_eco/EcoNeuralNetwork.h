// EcoNeuralNetwork.h
// Neural network implementation for ECO Q-learning

#ifndef ECO_NEURAL_NETWORK_H
#define ECO_NEURAL_NETWORK_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>
#include <string>

namespace eco {

// Activation functions
enum class ActivationType {
    RELU,
    TANH,
    SIGMOID,
    LINEAR
};

// Single layer in the network
class Layer {
public:
    Layer(size_t input_size, size_t output_size, 
          ActivationType activation = ActivationType::RELU);

  // In Layer class (add to header):
  void setWeights(const std::vector<std::vector<double>>& weights) { weights_ = weights; }
  void setBiases(const std::vector<double>& biases) { biases_ = biases; }
  
    std::vector<double> forward(const std::vector<double>& input);
    std::vector<double> backward(const std::vector<double>& grad_output, 
                                double learning_rate);
    
    // Getters
    size_t getInputSize() const { return input_size_; }
    size_t getOutputSize() const { return output_size_; }
    const std::vector<std::vector<double>>& getWeights() const { return weights_; }
    const std::vector<double>& getBiases() const { return biases_; }
    
    // Initialize weights
    void initializeWeights(double scale = 0.1);
    
private:
    size_t input_size_;
    size_t output_size_;
    ActivationType activation_;
    
    std::vector<std::vector<double>> weights_;
    std::vector<double> biases_;
    std::vector<double> last_input_;
    std::vector<double> last_output_;
    std::vector<double> last_activated_;
    
    double activate(double x);
    double activateDerivative(double x);
    std::vector<double> activate(const std::vector<double>& x);
    std::vector<double> activateDerivative(const std::vector<double>& x);
};

// Simple feedforward neural network
class NeuralNetwork {
public:
    NeuralNetwork(const std::vector<size_t>& layer_sizes,
                  const std::vector<ActivationType>& activations = { } );

  // Copy constructor - deep copy the layers
    NeuralNetwork(const NeuralNetwork& other);
    
    // Assignment operator - deep copy the layers
    NeuralNetwork& operator=(const NeuralNetwork& other);
    
    // Move constructor and assignment (default is fine)
    NeuralNetwork(NeuralNetwork&&) = default;
    NeuralNetwork& operator=(NeuralNetwork&&) = default;
    
    // Forward pass
    std::vector<double> forward(const std::vector<double>& input);
    
    // Backward pass (returns input gradients)
    std::vector<double> backward(const std::vector<double>& grad_output, 
                                double learning_rate);
    
    // Training utilities
    void train(const std::vector<double>& input,
              const std::vector<double>& target,
              double learning_rate);
    
    double computeLoss(const std::vector<double>& prediction,
                      const std::vector<double>& target);
    
    // Model persistence
    void save(const std::string& filename) const;
    void load(const std::string& filename);
    
    // Getters
    size_t getInputSize() const { return layers_.front()->getInputSize(); }
    size_t getOutputSize() const { return layers_.back()->getOutputSize(); }
    size_t getNumLayers() const { return layers_.size(); }
    
    // Weight initialization
    void initializeWeights(double scale = 0.1);
  void copyWeightsFrom(const NeuralNetwork& other);
  
private:
    std::vector<std::unique_ptr<Layer>> layers_;
    std::default_random_engine generator_;
};

// Experience replay buffer for training
struct Experience {
    std::vector<double> state;
    size_t action;
    double reward;
    std::vector<double> next_state;
    bool done;
};

class ReplayBuffer {
public:
    ReplayBuffer(size_t capacity);
    
    void add(const Experience& exp);
    std::vector<Experience> sample(size_t batch_size);
    size_t size() const { return buffer_.size(); }
    bool canSample(size_t batch_size) const { return size() >= batch_size; }
    void clear() { buffer_.clear(); current_ = 0; }
    
private:
    size_t capacity_;
    size_t current_;
    std::vector<Experience> buffer_;
    std::default_random_engine generator_;
};

}  // namespace eco

#endif  // ECO_NEURAL_NETWORK_H
