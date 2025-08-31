#pragma once

#include "IntraLocalSearchOperator.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

class IntraSwap : public IntraLocalSearchOperator
{
  private:
    std::vector<IntraMove> DetermineMoves(const Instance* instance, const Collections::IdVector& route) const override;

    void ChangeRoute(Collections::IdVector& route, const size_t node_i, const size_t node_k) const override;

    void RevertRoute(Collections::IdVector& route, const size_t node_k, const size_t node_i) const override;
};


}
}
