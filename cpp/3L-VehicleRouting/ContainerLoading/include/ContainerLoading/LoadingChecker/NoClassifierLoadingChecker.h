#pragma once

#include "BaseLoadingChecker.h"

namespace ContainerLoading
{
using namespace Algorithms;

class NoClassifierLoadingChecker : public BaseLoadingChecker
{
  public:

    using BaseLoadingChecker::BaseLoadingChecker; // inherits ctors


    [[nodiscard]] bool CompleteCheckStartSolution(const Model::Container& container,
                const boost::dynamic_bitset<>& set,
                const Collections::IdVector& stopIds,
                const std::vector<Model::Cuboid>& items,
                double maxRuntime) override;

    [[nodiscard]] bool CompleteCheck(const Model::Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Model::Cuboid>& items,
                                    const VehicleRouting::Improvement::ImprovementTypes& localsearchtype,
                                    double maxRuntime) override;

    [[nodiscard]] bool ExactCheckNoClassifier(const Model::Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Model::Cuboid>& items,
                                        double maxRuntime) override;

};
}