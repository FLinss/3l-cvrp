#include "Improvement/DeleteEmptyRoutes.h"

namespace VehicleRouting
{
namespace Improvement
{

void DeleteEmptyRoutes::Run(const Instance* const instance,
                            const InputParameters* const inputParameters,
                            ContainerLoading::BaseLoadingChecker* loadingChecker,
                            const Helper::Timer* const mTimer,
                            Model::Solution& currentSolution) const{

    // Remove empty routes
    auto& routes = currentSolution.Routes;

    routes.erase(std::remove_if(routes.begin(), routes.end(),
                                [](const Route& route) {
                                    return route.Sequence.empty();
                                }),
                 routes.end());

    // Reassign internal route IDs or reindex if necessary
    for (size_t i = 0; i < routes.size(); ++i)
    {
        routes[i].Id = static_cast<int>(i); // assuming Route has an Id field
    }
    // Update solution metadata
    currentSolution.NumberOfRoutes = routes.size();
};

}
}