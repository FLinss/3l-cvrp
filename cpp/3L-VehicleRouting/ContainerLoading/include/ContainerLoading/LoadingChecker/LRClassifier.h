#pragma once


#include "Classifier.h"
#include <torch/script.h>
#include <numeric> // for std::iota


namespace ContainerLoading {
    
class LRClassifier : public Classifier{
public:

    explicit LRClassifier(const ContainerLoadingParams& containerLoadingParams);

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    void saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container,
                                const float output,
                                const int status)const override;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    bool classify(const std::vector<Model::Cuboid>& items,
                   const Collections::IdVector& route,
                   const Model::Container& container) override;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    float classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container) override;

private:
    
    torch::Tensor mean_tensor;
    torch::Tensor std_tensor;
    torch::jit::script::Module model;


    void loadStandardScalingFromJson(const std::string& scaler_path) override;

    void save_tensor_to_csv(const torch::Tensor& tensor, const int status, const float output)const; 

    torch::Tensor applyStandardScaling(const torch::Tensor& input) const;

    torch::Tensor extractFeatures(const std::vector<Model::Cuboid>& items,
                                  const Collections::IdVector& route,
                                  const Model::Container& container) const;

};

}  // namespace ContainerLaoding

