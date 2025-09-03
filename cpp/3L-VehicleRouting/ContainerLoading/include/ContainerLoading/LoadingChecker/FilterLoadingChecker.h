#pragma once

#include "BaseLoadingChecker.h"
#include "Classifier/LRClassifier.h"
#include "Classifier/XGBClassifier.h"
#include "Classifier/FFNNClassifier.h"

namespace ContainerLoading
{
using namespace Algorithms;

class FilterLoadingChecker : public BaseLoadingChecker
{
  public:

    explicit FilterLoadingChecker(const ContainerLoadingParams& parameters)
    : BaseLoadingChecker(parameters)
    {
      switch (Parameters.ModelType){
        case ContainerLoadingParams::ModelTypes::FFNN:
          mClassifier = std::make_unique<FFNNClassifier>(Parameters);
          break;
        case ContainerLoadingParams::ModelTypes::LR:
          mClassifier= std::make_unique<LRClassifier>(Parameters);
          break;
        
        case ContainerLoadingParams::ModelTypes::XGBOOST:
          mClassifier = std::make_unique<XGBClassifier>(Parameters);
          break;
      }
    }

    [[nodiscard]] bool CompleteCheckStartSolution(const Container& container,
                const boost::dynamic_bitset<>& set,
                const Collections::IdVector& stopIds,
                const std::vector<Cuboid>& items,
                double maxRuntime) override;

    [[nodiscard]] bool CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items,
                                    const VehicleRouting::Improvement::ImprovementTypes& localsearchtype,
                                    double maxRuntime) override;

    [[nodiscard]] bool ExactCheckNoClassifier(const Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Cuboid>& items,
                                        double maxRuntime) override;

  private:
    std::unique_ptr<BaseClassifier> mClassifier;
};
}