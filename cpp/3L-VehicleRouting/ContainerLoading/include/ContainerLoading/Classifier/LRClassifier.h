#pragma once

#include "BaseClassifier.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace ContainerLoading {
    
class LRClassifier : public BaseClassifier{
public:

    explicit LRClassifier(const ContainerLoadingParams& containerLoadingParams);

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    void saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container,
                                float output,
                                int status) const override;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    bool classify(const std::vector<Model::Cuboid>& items,
                   const Collections::IdVector& route,
                   const Model::Container& container) override;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    float classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container) override;

private:
    void loadStandardScalingFromJson(const fs::path& scaler_path) override;
    void loadModelfromPath(const fs::path& model_path) override;

        // ---- Feature extraction (Eigen) ----
    Eigen::VectorXf extractFeatures(const std::vector<Model::Cuboid>& items,
                                    const Collections::IdVector& route,
                                    const Model::Container& container) const;

    // ---- Scaling (Eigen) ----
    Eigen::VectorXf applyStandardScaling(const Eigen::VectorXf& x) const;

    // ---- Utility ----
    static inline float sigmoid_stable(float z) {
        return (z >= 0.0f) ? 1.0f / (1.0f + std::exp(-z))
                           : std::exp(z) / (1.0f + std::exp(z));
    }

    void save_vector_to_csv(const Eigen::VectorXf& row,
                            int status,
                            float output) const;

private:
    // Parameters
    Eigen::VectorXf mean_;   // [N]
    Eigen::VectorXf std_;    // [N]

    Eigen::VectorXf w_;      // [N]
    float           b_ = 0.0f;
    float mLogitThreshold = 0.0f;

};

}  // namespace ContainerLaoding

