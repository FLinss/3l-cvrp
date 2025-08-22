#pragma once

#include "IntraLocalSearchOperator.h"

namespace VehicleRouting
{

namespace Improvement
{

class IntraInsertion : public IntraLocalSearchOperator
{
  private:
    std::vector<IntraMove> DetermineMoves(const Instance* instance, const Collections::IdVector& route) const override;

  void ChangeRoute(Collections::IdVector& route, const size_t i, const size_t k) const override;

  void RevertRoute(Collections::IdVector& route, const size_t k, const size_t i)const  override;

};
}
}