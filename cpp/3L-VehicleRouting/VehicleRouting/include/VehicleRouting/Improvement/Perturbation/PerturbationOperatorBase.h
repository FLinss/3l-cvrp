#pragma once

#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "Model/Instance.h"
#include "Model/Solution.h"
#include "Algorithms/BCRoutingParams.h"
#include "Algorithms/Evaluation.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <unordered_set>
#include <algorithm>
#include <random>
#include <optional>

namespace VehicleRouting {
namespace Improvement {

using PerturbationMove = std::tuple<double, size_t, size_t, size_t, size_t, int, int>;

class PerturbationOperatorBase {
public:
    virtual ~PerturbationOperatorBase() = default;

    void Run(const Instance* const instance,
            const InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution,
            std::mt19937& rng) const;


  private:
    ImprovementTypes mType = ImprovementTypes::Perturbation;

  protected:
    virtual std::optional<PerturbationMove> DetermineMoves(const Instance* instance,
                                                    const std::vector<Route>& routes,
                                                    std::mt19937& rng) const = 0;

    virtual void ChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move) const = 0;
    virtual void RevertChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move) const = 0;

    void UpdateRouteVolumeWeight(std::vector<Route>& routes, const PerturbationMove& move) const;

};


}
}  // namespace
