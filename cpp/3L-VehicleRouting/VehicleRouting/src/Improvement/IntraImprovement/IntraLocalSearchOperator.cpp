#include "Improvement/IntraImprovement/IntraLocalSearchOperator.h"

namespace VehicleRouting
{
namespace Improvement
{

void IntraLocalSearchOperator::Run(const Instance* const instance,
            const VRP_InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution) const 
{                                    

   for(auto& route : currentSolution.Routes){

        if (route.Sequence.size() < 2)
        {
            continue;
        }

        while(true){

            auto moves = DetermineMoves(instance, route.Sequence);
            auto savings = GetBestMove(instance, inputParameters, loadingChecker,mTimer, route.Sequence, moves);

            if(!savings){
                break;
            }else{
                currentSolution.Costs += *savings;
            }
        }
    }
       
};

std::optional<double> IntraLocalSearchOperator::GetBestMove(const Instance* const instance,
                                                            const VRP_InputParameters* const inputParameters,
                                                            ContainerLoading::BaseLoadingChecker* loadingChecker,
                                                            const Helper::Timer* const mTimer,
                                                            Collections::IdVector& route,
                                                            std::vector<IntraMove>& moves) const 
{

    if (moves.size() == 0)
    {
        return std::nullopt;
    }

    std::ranges::sort(moves, [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
    });

    auto set =  loadingChecker->MakeBitset(instance->Nodes.size(), route);

    const auto& container = instance->Vehicles.front().Containers.front();

    for (const auto& move: moves)
    {

        ChangeRoute(route, std::get<1>(move), std::get<2>(move));
        
        auto selectedItems = Algorithms::InterfaceConversions::SelectItems(route, instance->Nodes, false);
        double maxRuntime = inputParameters->DetermineMaxRuntime(Algorithms::IteratedLocalSearchParams::CallType::Exact, mTimer->getElapsedTime());
        if (loadingChecker->CompleteCheck(container, set, route, selectedItems, mType, maxRuntime))
        {
            return std::get<0>(move);
        }
        
        //Change routes back if it was not feasible! 
        //TODO Dont forget to delete 
        RevertRoute(route, std::get<1>(move), std::get<2>(move));
    }

    return std::nullopt;


};

}
}