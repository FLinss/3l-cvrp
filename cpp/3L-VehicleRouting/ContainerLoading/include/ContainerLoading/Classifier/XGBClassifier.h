#pragma once

#include "BaseClassifier.h"


namespace ContainerLoading {
    
class XGBClassifier : public BaseClassifier{
public:

    explicit XGBClassifier(const ContainerLoadingParams& containerLoadingParams);

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    void saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container,
                                const float output,
                                const int status) const override;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    bool classify(const std::vector<Model::Cuboid>& items,
                   const Collections::IdVector& route,
                   const Model::Container& container) override;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    float classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container) override;

private:
    void loadStandardScalingFromJson(const std::string& scaler_path) override;
};

}  // namespace ContainerLaoding
