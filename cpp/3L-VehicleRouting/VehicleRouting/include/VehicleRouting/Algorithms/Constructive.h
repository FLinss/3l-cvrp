#pragma once

#include "Algorithms/BCRoutingParams.h"
#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "Helper/Timer.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <random>
#include <algorithm>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Constructive
{


class Savings
{
  public:
    Savings(const Model::Instance* const instance,
            const InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* timer)

    : mInstance(instance), mInputParameters(inputParameters), mLoadingChecker(loadingChecker), mTimer(timer){};

    std::vector<Model::Route> Run();

  private:
    const Model::Instance* const mInstance;
    const InputParameters* const mInputParameters;
    ContainerLoading::BaseLoadingChecker* mLoadingChecker;
    const Helper::Timer* const mTimer;

    bool ConcatRoutes(Collections::IdVector& frontSequence,
                      const Collections::IdVector& backSequence,
                      const ContainerLoading::Model::Container& container);
    void
        DeleteSavings(std::vector<std::tuple<double, size_t, size_t>>& savingsValues, size_t startNode, size_t endNode);
};

class ModifiedSavings
{
  public:
    ModifiedSavings(const Model::Instance* const instance,
                    const InputParameters* const inputParameters,
                    ContainerLoading::BaseLoadingChecker* loadingChecker,
                    std::mt19937* rng,
                    const Helper::Timer* timer)

    : mInstance(instance), mInputParameters(inputParameters), mLoadingChecker(loadingChecker), mRNG(rng),  mTimer(timer){};

    std::vector<Model::Route> Run();

  private:
    const Model::Instance* const mInstance;
    const InputParameters* const mInputParameters;
    ContainerLoading::BaseLoadingChecker* mLoadingChecker;
    std::mt19937* mRNG;
    const Helper::Timer* const mTimer;

    void RepairProcedure(std::vector<Model::Route>& solution);
    std::vector<std::tuple<double, size_t, size_t>> DetermineInsertionCostsAllRoutes(std::vector<Model::Route>& solution,
                                                                                     size_t nodeId);
    std::vector<std::tuple<double, size_t, size_t>>
        DetermineInsertionCosts(const Model::Route& route, size_t routeId, size_t nodeId);
    Collections::IdVector InsertInRandomRoute(Model::Route& route, size_t nodeToInsert);
    bool InsertionFeasible(Model::Route& route, size_t nodeToInsert, size_t position);
};

}
}
}