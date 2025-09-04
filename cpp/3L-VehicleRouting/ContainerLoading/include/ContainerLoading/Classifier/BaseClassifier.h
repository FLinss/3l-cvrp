// File: MLModelsContainer.h
#pragma once

#include "CommonBasics/Helper/ModelServices.h"
#include "Model/ContainerLoadingInstance.h"
#include "ProblemParameters.h"
#include "nlohmann/json.hpp"
#include <filesystem>
#include <numeric>   // std::accumulate
#include <iterator>  // std::distance
#include <cmath>     // std::pow, std::sqrt
#include <sstream>   // std::ostringstream
#include <iomanip>   // std::put_time, std::setw, std::setfill
#include <chrono>    // std::chrono::
#include <ctime>     // std::tm, std::localtime
#include <fstream>
#include <algorithm>
#include <stdexcept>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace ContainerLoading {
    
class BaseClassifier {
public:

    BaseClassifier(const ContainerLoadingParams& containerLoadingParams);
    virtual ~BaseClassifier() = default;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    virtual void saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container,
                                float output,
                                int status) const = 0;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    virtual bool classify(const std::vector<Model::Cuboid>& items,
                   const Collections::IdVector& route,
                   const Model::Container& container) = 0;

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    virtual float classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                                const Collections::IdVector& route,
                                const Model::Container& container) = 0;


protected:

    virtual void loadStandardScalingFromJson(const fs::path& scaler_path) = 0;
    virtual void loadModelfromPath(const fs::path& model_path) = 0;

    float mAcceptanceThreshold;
    std::string mSaveTensorPath;
    std::string modelTypeString{};   

    std::string get_timestamp() const;

    static float getMean(std::vector<float>::const_iterator first,
                        std::vector<float>::const_iterator last);
    static float getStd (std::vector<float>::const_iterator first,
                        std::vector<float>::const_iterator last);

};

}  // namespace ContainerLaoding

