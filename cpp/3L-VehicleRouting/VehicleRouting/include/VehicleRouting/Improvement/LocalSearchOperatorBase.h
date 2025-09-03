#pragma once

#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "Model/Instance.h"
#include "Model/Solution.h"
#include "Algorithms/BCRoutingParams.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "Helper/Timer.h"

#include <optional>
#include <tuple>
#include <vector>

namespace VehicleRouting {
namespace Improvement {

using VRP_InputParameters = VehicleRouting::Algorithms::InputParameters;

class LocalSearchOperatorBase {
public:
    virtual ~LocalSearchOperatorBase() = default;

    virtual void Run(const Instance* const instance,
                    const VRP_InputParameters* const inputParameters,
                    ContainerLoading::BaseLoadingChecker* loadingChecker,
                    const Helper::Timer* const mTimer,
                    Model::Solution& currentSolution) const = 0;
};

}}  // namespace
