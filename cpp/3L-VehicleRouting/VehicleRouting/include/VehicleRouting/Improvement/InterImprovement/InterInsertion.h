#pragma once

#include "InterLocalSearchOperator.h"

namespace VehicleRouting
{

namespace Improvement
{

class InterInsertion : public InterLocalSearchOperator
{
  private:
    std::vector<InterMove> DetermineMoves(const Model::Instance* instance,
                                          const std::vector<Model::Route>& routes) const override;

    void ChangeRoutes(std::vector<Model::Route>& routes, const InterMove& move) const override;
    void RevertChangeRoutes(std::vector<Model::Route>& routes, const InterMove& move) const override;
};
}
}