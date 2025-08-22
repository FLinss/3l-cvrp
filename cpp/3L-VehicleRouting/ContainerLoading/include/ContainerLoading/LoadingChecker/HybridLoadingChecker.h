#pragma once

#include "BaseLoadingChecker.h"
#include "Classifier/LRClassifier.h"
#include "Classifier/XGBClassifier.h"
#include "Classifier/FFNNClassifier.h"

namespace ContainerLoading
{
using namespace Algorithms;

class HybridLoadingChecker : public BaseLoadingChecker
{
  public:

    explicit HybridLoadingChecker(const ContainerLoadingParams& parameters, const double maxruntime)
    : BaseLoadingChecker(parameters,maxruntime)
    {
      switch (Parameters.ModelType){
        case ContainerLoadingParams::ModelTypes::FFNN:
          mClassifier = std::make_unique<FFNNClassifier>(Parameters);
          break;
        case ContainerLoadingParams::ModelTypes::LR:
          mClassifier = std::make_unique<LRClassifier>(Parameters);
          break;
        
        case ContainerLoadingParams::ModelTypes::XGBOOST:
          mClassifier = std::make_unique<XGBClassifier>(Parameters);
          break;
      }
    }

    [[nodiscard]] bool CompleteCheckStartSolution(const Container& container,
                const boost::dynamic_bitset<>& set,
                const Collections::IdVector& stopIds,
                const std::vector<Cuboid>& items) override;

    [[nodiscard]] bool CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items,
                                    const VehicleRouting::Improvement::ImprovementTypes& localsearchtype) override;

    [[nodiscard]] bool ExactCheckNoClassifier(const Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Cuboid>& items) override;

  private:
    std::unique_ptr<BaseClassifier> mClassifier;
};
}