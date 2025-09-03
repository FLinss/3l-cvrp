#pragma once

#include "Improvement/LocalSearchOperatorBase.h"

namespace VehicleRouting
{

namespace Improvement
{

class DeleteEmptyRoutes : public LocalSearchOperatorBase
{
    void Run(const Model::Instance* const instance,
            const VRP_InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution) const override;

};

}
}