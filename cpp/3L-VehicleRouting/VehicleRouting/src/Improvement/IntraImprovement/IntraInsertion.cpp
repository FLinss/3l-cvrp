#include "Improvement/IntraImprovement/IntraInsertion.h"

namespace VehicleRouting
{
namespace Improvement
{


std::vector<IntraMove> IntraInsertion::DetermineMoves(const Model::Instance* const instance,
                                                        const Collections::IdVector& route) const
{

    std::vector<IntraMove> moves{};
    auto savings = 0.0; 

    for (size_t node_i = 0; node_i < route.size() - 1; ++node_i)
    {
        for (size_t position_k = 0; position_k <= route.size(); ++position_k)
        {
            if (position_k == node_i - 1 || position_k == node_i || position_k == node_i + 1) continue; // Inserting before or after itself makes no change

            savings = Algorithms::Evaluator::CalculateIntraInsertionDelta(instance, route, node_i, position_k);

            if (savings < -1e-3)
            {
                //std::cout << "Found new savings: " << savings << std::endl; 
                moves.emplace_back(savings, node_i, position_k);
            }
        }
    }

    return moves;
}

void IntraInsertion::ChangeRoute(Collections::IdVector& route, const size_t i, const size_t k) const
{

    auto position_k = k;
    if (position_k > i) --position_k; 

    // 1. Save the value at node_i
    auto value = route[i];

    // 2. Erase the element
    route.erase(route.begin() + i);

    // 4. Insert the saved value
    route.insert(route.begin() + position_k, value);
}

void IntraInsertion::RevertRoute(Collections::IdVector& route, const size_t old_i, const size_t old_k) const
{
    size_t new_i = (old_k > old_i) ? old_k - 1 : old_k;
    size_t new_k = (old_k > old_i) ? old_i : old_i + 1;

    ChangeRoute(route, new_i, new_k);
}
}
}