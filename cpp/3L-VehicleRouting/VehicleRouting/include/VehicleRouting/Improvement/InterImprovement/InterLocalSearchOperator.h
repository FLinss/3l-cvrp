#pragma once

#include "Improvement/LocalSearchOperatorBase.h"

namespace VehicleRouting
{
namespace Improvement
{

using InterMove = std::tuple<double, size_t, size_t, size_t, size_t, int, int>;

class InterLocalSearchOperator : public LocalSearchOperatorBase
{
  public:
    void Run(const Instance* const instance,
            const InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution) const override;

  private:
    ImprovementTypes mType = ImprovementTypes::Inter;

  protected:
    std::optional<double> GetBestMove(const Instance* const instance,
                                      const InputParameters* const inputParameters,
                                      ContainerLoading::BaseLoadingChecker* loadingChecker,
                                      const Helper::Timer* const mTimer,
                                      std::vector<Route>& routes,
                                      std::vector<InterMove>& moves) const;

    virtual std::vector<InterMove> DetermineMoves(const Instance* instance,
                                                  const std::vector<Route>& routes) const = 0;

    virtual void ChangeRoutes(std::vector<Route>& routes, const InterMove& move) const = 0;
    virtual void RevertChangeRoutes(std::vector<Route>& routes, const InterMove& move) const = 0; 

    void UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterMove& move) const;

};
}
}