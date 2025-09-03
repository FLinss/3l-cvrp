#pragma once

#include "ContainerLoading/Model/ContainerLoadingInstance.h"
#include "Model/Node.h"

#include <vector>

namespace VehicleRouting
{
namespace Algorithms
{

class InterfaceConversions
{
  public:
    // Duplicate code in LoadingChecker::SelectItems. Is required for the current quick and dirty removal of vehicle
    // routing logic from the container loading module. A cleaner separation requires major rework in the container
    // loading submodule.
    static std::vector<ContainerLoading::Model::Cuboid>
        SelectItems(const Collections::IdVector& nodeIds, const std::vector<Model::Node>& nodes, bool reversedDirection);

    static std::vector<ContainerLoading::Model::Group> NodesToGroup(const std::vector<Model::Node>& nodes);
};

}
}