#include "Improvement/InterImprovement/InterLocalSearchOperator.h"
#include "CommonBasics/Helper/ModelServices.h"

namespace VehicleRouting
{
namespace Improvement
{

void InterLocalSearchOperator::Run(const Instance* const instance,
            const InputParameters* const inputParameters,
            ContainerLoading::BaseLoadingChecker* loadingChecker,
            const Helper::Timer* const mTimer,
            Model::Solution& currentSolution) const {



  std::vector<Route>& routes = currentSolution.Routes;

  if (routes.size() < 2)
  {
      return;
  }

  while(true){

      auto moves = DetermineMoves(instance, routes);
      auto savings = GetBestMove(instance, inputParameters, loadingChecker, mTimer, routes, moves);

      if(!savings){
          break;
      }else{
          currentSolution.Costs += *savings;
      }
  }

  return;

}



std::optional<double> InterLocalSearchOperator::GetBestMove(const Instance* const instance,
                                                            const InputParameters* const inputParameters,
                                                            ContainerLoading::BaseLoadingChecker* loadingChecker,
                                                            const Helper::Timer* const mTimer,
                                                            std::vector<Route>& routes,
                                                            std::vector<InterMove>& moves) const
{
  if (moves.size() == 0)
  {
      return std::nullopt;
  }

  std::ranges::sort(moves, [](const auto& a, const auto& b) {
      return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
  });
  
  //TODO - Create Bitset for all the two routes!
  //auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

  //Initiate variables before loop
  const auto& container = instance->Vehicles.front().Containers.front();

  for (const auto& move: moves)
  {
      bool controlFlag = true;
      

      if (loadingChecker->Parameters.LoadingFlags == LoadingFlag::NoneSet)
      {
          UpdateRouteVolumeWeight(routes, move);
          return std::get<0>(move);
      }

      ChangeRoutes(routes, move);


      for(auto& route_index : {std::get<1>(move), std::get<2>(move)})
      {
        auto& route = routes[route_index];
        if(route.Sequence.empty()){
            continue;
        }        
        
        auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);
        auto selectedItems = Algorithms::InterfaceConversions::SelectItems(route.Sequence, instance->Nodes, false);
        double maxRuntime = inputParameters->DetermineMaxRuntime(IteratedLocalSearchParams::CallType::Exact, mTimer->getElapsedTime());
        if (!loadingChecker->CompleteCheck(container,  set, route.Sequence, selectedItems, mType, maxRuntime)){
            controlFlag = false; 
            break;
        }

      }
      // Change routes back if not feasible!
      if (!controlFlag)
      {
          RevertChangeRoutes(routes, move);
          continue;
      }

      UpdateRouteVolumeWeight(routes, move);
      return std::get<0>(move);
  }

  return std::nullopt;

};


void InterLocalSearchOperator::UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterMove& move) const {

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
}
