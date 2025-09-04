#pragma once

#include "BaseClassifier.h"
#include <xgboost/c_api.h>   // from XGBoost build/include


namespace ContainerLoading {
    
class XGBClassifier : public BaseClassifier{
public:

    explicit XGBClassifier(const ContainerLoadingParams& containerLoadingParams);

    // Body of destructor 
    ~XGBClassifier(){
        XGBoosterFree(booster);
    }

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
    BoosterHandle booster;

    void loadStandardScalingFromJson(const fs::path& scaler_path) override;
    void loadModelfromPath(const fs::path& model_path) override;


    std::array<float, 48> extractFeatures(const std::vector<Model::Cuboid>& items,
                                            const Collections::IdVector& route,
                                            const Model::Container& container) const;

        
    void save_features_to_csv(const std::array<float, 48>,
                            const int status,
                            const float output) const;
};

}  // namespace ContainerLaoding
