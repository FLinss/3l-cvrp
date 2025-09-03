#include "Improvement/Perturbation/PerturbationOperatorBase.h"

namespace VehicleRouting {
namespace Improvement {

void PerturbationOperatorBase::Run(const Instance* const instance,
                                    const VRP_InputParameters* const inputParameters,
                                    ContainerLoading::BaseLoadingChecker* loadingChecker,
                                    const Helper::Timer* const mTimer,
                                    Model::Solution& currentSolution,
                                    std::mt19937& rng) const
{

    int succesful_moves = 0;

    std::vector<Route>& routes = currentSolution.Routes;

    if (routes.size() < 2)
    {
        return;
    }

    //Preinitialize values
    const auto& container = instance->Vehicles.front().Containers.front();

    // Implement unordered_set as tabu list

    while(succesful_moves < inputParameters->IteratedLocalSearch.K_RandomMoves){

        // Implement unordered_set as tabu list
        //Update DetermineMovesTo give only one Move, where routes are different! 
        // Only impirotant taht weight and volume restrictions are applied! 
        // Copy code from GetBestMove here and update found swaps if swaps are feasible!

        auto move = DetermineMoves(instance, routes, rng);

        if(!move){
            break;
        }

        bool controlFlag = true;

        ChangeRoutes(routes, *move);

        for(const auto& route_index : {std::get<1>(*move), std::get<2>(*move)})
        {
            auto& route = routes[route_index];

            if(route.Sequence.empty()){
                continue;
            }

            
            auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);
            auto selectedItems = Algorithms::InterfaceConversions::SelectItems(route.Sequence, instance->Nodes, false);
            
            double maxRuntime = inputParameters->DetermineMaxRuntime(Algorithms::IteratedLocalSearchParams::CallType::Exact, mTimer->getElapsedTime());
            if (!loadingChecker->CompleteCheck(container, set, route.Sequence, selectedItems, mType, maxRuntime))
            {
                controlFlag = false;
                break;
            }

        }
        // Change routes back if not feasible!
        if (!controlFlag)
        {
            RevertChangeRoutes(routes, *move);
            continue;
        }

        UpdateRouteVolumeWeight(routes, *move);
        currentSolution.Costs += std::get<0>(*move);
        ++succesful_moves;
    }  
};

void PerturbationOperatorBase::UpdateRouteVolumeWeight(std::vector<Route>& routes, const PerturbationMove& move) const {

    const auto item_delta = std::get<5>(move);
    const auto volume_delta = std::get<6>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    route_i.TotalWeight -= item_delta;
    route_k.TotalWeight += item_delta;
    route_i.TotalVolume -= volume_delta;
    route_k.TotalVolume += volume_delta;
};


}
}  // namespace
