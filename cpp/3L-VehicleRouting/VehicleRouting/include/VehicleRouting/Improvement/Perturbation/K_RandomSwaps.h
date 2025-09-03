#pragma once

#include "PerturbationOperatorBase.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

class K_RandomSwaps : public PerturbationOperatorBase
{
  private:
     std::optional<PerturbationMove> DetermineMoves(const Model::Instance* instance,
                                                    const std::vector<Model::Route>& routes,
                                                    std::mt19937& rng) const override;

     void ChangeRoutes(std::vector<Model::Route>& routes, const PerturbationMove& move) const override;
     void RevertChangeRoutes(std::vector<Model::Route>& routes, const PerturbationMove& move) const override;
};
}
}