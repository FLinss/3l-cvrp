#pragma once

#include "Improvement/LocalSearchOperatorBase.h"

namespace VehicleRouting
{
namespace Improvement
{

using IntraMove = std::tuple<double, size_t, size_t>;

class IntraLocalSearchOperator : public LocalSearchOperatorBase
{
  public:
    void Run(const Model::Instance* const instance,
            const VRP_InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution) const override;

  private:
    ImprovementTypes mType = ImprovementTypes::Intra;

  protected:
    std::optional<double> GetBestMove(const Model::Instance* instance,
                                      const VRP_InputParameters* const inputParameters,
                                      ContainerLoading::BaseLoadingChecker* loadingChecker,
                                      const Helper::Timer* const mTimer,  
                                      Collections::IdVector& route,
                                      std::vector<IntraMove>& moves) const;

    virtual std::vector<IntraMove> DetermineMoves(const Model::Instance* instance,
                                                  const Collections::IdVector& route) const = 0;

    virtual void ChangeRoute(Collections::IdVector& route, const size_t i, const size_t k) const = 0;
    virtual void RevertRoute(Collections::IdVector& route, const size_t k, const size_t i) const = 0;
};
}
}