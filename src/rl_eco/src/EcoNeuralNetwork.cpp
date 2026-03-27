// EcoNeuralNetwork.cpp
// Implementation of neural network for ECO Q-learning

#include "rl_eco/EcoNeuralNetwork.h"
#include <fstream>
#include <iostream>
#include <numeric>
#include <cassert>

namespace eco {

// Layer implementation
Layer::Layer(size_t input_size, size_t output_size, ActivationType activation)
    : input_size_(input_size), 
      output_size_(output_size), 
      activation_(activation) {
    
    weights_.resize(output_size, std::vector<double>(input_size));
    biases_.resize(output_size, 0.0);
    initializeWeights();
}

void Layer::initializeWeights(double scale) {
  std::random_device rd;
  std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution(0.0, scale);
    
    // Xavier initialization scale
    double xavier_scale = std::sqrt(2.0 / (input_size_ + output_size_));
    
    for (auto& row : weights_) {
        for (auto& w : row) {
            w = distribution(generator) * xavier_scale;
        }
    }
    
    for (auto& b : biases_) {
        b = distribution(generator) * 0.01;
    }
}

  

std::vector<double> Layer::forward(const std::vector<double>& input) {
  if (input.size() != input_size_){
    printf("Cannot forward propagate, input size mis-match\n");
  }
    assert(input.size() == input_size_);
    last_input_ = input;
    
    std::vector<double> output(output_size_, 0.0);
    printf("*Output size %d\n", output_size_);
    
    // Compute linear transformation
    for (size_t i = 0; i < output_size_; ++i) {
        for (size_t j = 0; j < input_size_; ++j) {
            output[i] += weights_[i][j] * input[j];
        }
        output[i] += biases_[i];
    }
    
    last_output_ = output;
    last_activated_ = activate(output);
    return last_activated_;
}

std::vector<double> Layer::backward(const std::vector<double>& grad_output, 
                                   double learning_rate) {
    assert(grad_output.size() == output_size_);
    
    // Compute gradients w.r.t. pre-activation
    std::vector<double> grad_pre_activation(output_size_);
    for (size_t i = 0; i < output_size_; ++i) {
        grad_pre_activation[i] = grad_output[i] * activateDerivative(last_output_[i]);
    }
    
    // Compute gradients w.r.t. input
    std::vector<double> grad_input(input_size_, 0.0);
    for (size_t j = 0; j < input_size_; ++j) {
        for (size_t i = 0; i < output_size_; ++i) {
            grad_input[j] += weights_[i][j] * grad_pre_activation[i];
        }
    }
    
    // Update weights and biases
    for (size_t i = 0; i < output_size_; ++i) {
        for (size_t j = 0; j < input_size_; ++j) {
            weights_[i][j] -= learning_rate * grad_pre_activation[i] * last_input_[j];
        }
        biases_[i] -= learning_rate * grad_pre_activation[i];
    }
    
    return grad_input;
}

double Layer::activate(double x) {
    switch (activation_) {
        case ActivationType::RELU:
            return std::max(0.0, x);
        case ActivationType::TANH:
            return std::tanh(x);
        case ActivationType::SIGMOID:
	  if (x > 10) return 1.0;  // Prevent overflow
	  if (x < -10) return 0.0;  // Prevent underflow
	  return 1.0 / (1.0 + std::exp(-x));
        case ActivationType::LINEAR:
        default:
            return x;
    }
}

double Layer::activateDerivative(double x) {
    switch (activation_) {
        case ActivationType::RELU:
            return x > 0 ? 1.0 : 0.0;
        case ActivationType::TANH: {
            double t = std::tanh(x);
            return 1.0 - t * t;
        }
        case ActivationType::SIGMOID: {
            double s = 1.0 / (1.0 + std::exp(-x));
            return s * (1.0 - s);
        }
        case ActivationType::LINEAR:
        default:
            return 1.0;
    }
}

std::vector<double> Layer::activate(const std::vector<double>& x) {
    std::vector<double> result(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        result[i] = activate(x[i]);
    }
    return result;
}

std::vector<double> Layer::activateDerivative(const std::vector<double>& x) {
    std::vector<double> result(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        result[i] = activateDerivative(x[i]);
    }
    return result;
}

// NeuralNetwork implementation
NeuralNetwork::NeuralNetwork(const std::vector<size_t>& layer_sizes,
                           const std::vector<ActivationType>& activations) {
    assert(layer_sizes.size() >= 2);

    int ix=0;
    for (auto s: layer_sizes){
      printf("Layer %d Size %d\n", ix, s);
      ix++;
    }
    std::vector<ActivationType> acts = activations;
    if (acts.empty()) {
        // Default: ReLU for hidden layers, linear for output
        acts.resize(layer_sizes.size() - 1, ActivationType::RELU);
        acts.back() = ActivationType::LINEAR;
    }
    
    for (size_t i = 0; i < layer_sizes.size() - 1; ++i) {
        layers_.emplace_back(std::make_unique<Layer>(
            layer_sizes[i], layer_sizes[i + 1], acts[i]
        ));
    }
}

std::vector<double> NeuralNetwork::forward(const std::vector<double>& input) {
    std::vector<double> output = input;
    for (const auto& layer : layers_) {
        output = layer->forward(output);
    }
    return output;
}

std::vector<double> NeuralNetwork::backward(const std::vector<double>& grad_output, 
                                           double learning_rate) {
    std::vector<double> grad = grad_output;
    for (auto it = layers_.rbegin(); it != layers_.rend(); ++it) {
        grad = (*it)->backward(grad, learning_rate);
    }
    return grad;
}

void NeuralNetwork::train(const std::vector<double>& input,
                         const std::vector<double>& target,
                         double learning_rate) {
    // Forward pass
    std::vector<double> output = forward(input);
    
    // Compute loss gradient (MSE)
    std::vector<double> grad_output(output.size());
    for (size_t i = 0; i < output.size(); ++i) {
        grad_output[i] = 2.0 * (output[i] - target[i]) / output.size();
    }
    
    // Backward pass
    backward(grad_output, learning_rate);
}

double NeuralNetwork::computeLoss(const std::vector<double>& prediction,
                                 const std::vector<double>& target) {
    assert(prediction.size() == target.size());
    double loss = 0.0;
    for (size_t i = 0; i < prediction.size(); ++i) {
        double diff = prediction[i] - target[i];
        loss += diff * diff;
    }
    return loss / prediction.size();
}

void NeuralNetwork::initializeWeights(double scale) {
  
    for (auto& layer : layers_) {
        layer->initializeWeights(scale);
    }
}

  void NeuralNetwork::copyWeightsFrom(const NeuralNetwork& other) {
    assert(layers_.size() == other.layers_.size());
    for (size_t i = 0; i < layers_.size(); ++i) {
        layers_[i]->setWeights(other.layers_[i]->getWeights());
        layers_[i]->setBiases(other.layers_[i]->getBiases());
    }
}

void NeuralNetwork::save(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for saving: " + filename);
    }
    
    // Save network architecture
    size_t num_layers = layers_.size();
    file.write(reinterpret_cast<const char*>(&num_layers), sizeof(num_layers));
    
    // Save each layer
    for (const auto& layer : layers_) {
        size_t input_size = layer->getInputSize();
        size_t output_size = layer->getOutputSize();
        file.write(reinterpret_cast<const char*>(&input_size), sizeof(input_size));
        file.write(reinterpret_cast<const char*>(&output_size), sizeof(output_size));
        
        // Save weights
        const auto& weights = layer->getWeights();
        for (const auto& row : weights) {
            file.write(reinterpret_cast<const char*>(row.data()), 
                      row.size() * sizeof(double));
        }
        
        // Save biases
        const auto& biases = layer->getBiases();
        file.write(reinterpret_cast<const char*>(biases.data()), 
                  biases.size() * sizeof(double));
    }
}

void NeuralNetwork::load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for loading: " + filename);
    }
    
    size_t num_layers;
    file.read(reinterpret_cast<char*>(&num_layers), sizeof(num_layers));
    assert(num_layers == layers_.size());
    
    for (auto& layer : layers_) {
        size_t input_size, output_size;
        file.read(reinterpret_cast<char*>(&input_size), sizeof(input_size));
        file.read(reinterpret_cast<char*>(&output_size), sizeof(output_size));
        printf("input size %d layer input size %d\n",
	       input_size,
	       layer->getInputSize());
        assert(input_size == layer->getInputSize());
	if (output_size != layer->getOutputSize()){
	  printf("Error output size mis-match: output size %d layer output size %d\n",
		 output_size,
		 layer -> getOutputSize());
	}
        assert(output_size == layer->getOutputSize());
        
        // Load weights
        std::vector<std::vector<double>> weights(output_size, std::vector<double>(input_size));
        for (auto& row : weights) {
            file.read(reinterpret_cast<char*>(row.data()), row.size() * sizeof(double));
        }
        layer->setWeights(weights);
        
        // Load biases
        std::vector<double> biases(output_size);
        file.read(reinterpret_cast<char*>(biases.data()), biases.size() * sizeof(double));
        layer->setBiases(biases);
    }
}

  
// ReplayBuffer implementation
ReplayBuffer::ReplayBuffer(size_t capacity) 
  : capacity_(capacity), current_(0), generator_(std::random_device{}()) {
    buffer_.reserve(capacity);
}

void ReplayBuffer::add(const Experience& exp) {
    if (buffer_.size() < capacity_) {
        buffer_.push_back(exp);
    } else {
        buffer_[current_] = exp;
    }
    current_ = (current_ + 1) % capacity_;
}

std::vector<Experience> ReplayBuffer::sample(size_t batch_size) {
    std::uniform_int_distribution<size_t> dist(0, buffer_.size() - 1);
    std::vector<Experience> batch;
    batch.reserve(batch_size);
    
    for (size_t i = 0; i < batch_size; ++i) {
        batch.push_back(buffer_[dist(generator_)]);
    }
    
    return batch;
}


  //copy
  // Copy constructor
NeuralNetwork::NeuralNetwork(const NeuralNetwork& other) {
    // Recreate the network architecture
    for (const auto& layer : other.layers_) {
        layers_.emplace_back(std::make_unique<Layer>(
            layer->getInputSize(), 
            layer->getOutputSize(), 
            ActivationType::RELU  // You might want to store activation type in Layer
        ));
    }
    
    // Copy weights and biases
    copyWeightsFrom(other);
}

// Assignment operator
NeuralNetwork& NeuralNetwork::operator=(const NeuralNetwork& other) {
    if (this != &other) {
        // Clear existing layers
        layers_.clear();
        
        // Recreate the network architecture
        for (const auto& layer : other.layers_) {
            layers_.emplace_back(std::make_unique<Layer>(
                layer->getInputSize(), 
                layer->getOutputSize(), 
                ActivationType::RELU  // You might want to store activation type in Layer
            ));
        }
        
        // Copy weights and biases
        copyWeightsFrom(other);
    }
    return *this;
}
}  // namespace eco
