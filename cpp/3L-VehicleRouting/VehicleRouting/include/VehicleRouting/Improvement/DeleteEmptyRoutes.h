#pragma once

#include "Improvement/LocalSearchOperatorBase.h"

namespace VehicleRouting
{

namespace Improvement
{

class DeleteEmptyRoutes : public LocalSearchOperatorBase
{
    void Run(const Instance* const instance,
            const InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution) const override;

};

}
}