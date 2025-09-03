#include "LoadingChecker/NoClassifierLoadingChecker.h"

namespace ContainerLoading
{


bool NoClassifierLoadingChecker::CompleteCheckStartSolution(const Model::Container& container,
                const boost::dynamic_bitset<>& set,
                const Collections::IdVector& stopIds,
                const std::vector<Model::Cuboid>& items,
                double maxRuntime)
{    
    if (RouteIsInFeasSequences(stopIds))
    {
        return true;
    }

    if (RouteIsInInfeasSequences(stopIds))
    {
        return false;
    }
    

    auto cpStatus = ConstraintProgrammingSolver(CLP_PackingType::Complete,
                                                container,
                                                set,
                                                stopIds,
                                                items,
                                                false,
                                                maxRuntime);

    return cpStatus == CLP_LoadingStatus::FeasOpt;
}

bool NoClassifierLoadingChecker::CompleteCheck(const Model::Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Model::Cuboid>& items,
                                    const VehicleRouting::Improvement::ImprovementTypes& localsearchtype,
                                    double maxRuntime)
{
    if (RouteIsInFeasSequences(stopIds))
    {
        return true;
    }

    if (RouteIsInInfeasSequences(stopIds))
    {
        return false;
    }
    

    auto cpStatus = ConstraintProgrammingSolver(CLP_PackingType::Complete,
                                                container,
                                                set,
                                                stopIds,
                                                items,
                                                false,
                                                maxRuntime);

    return cpStatus == CLP_LoadingStatus::FeasOpt;

}

bool NoClassifierLoadingChecker::ExactCheckNoClassifier(const Model::Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Model::Cuboid>& items,
                                        double maxRuntime){
    
    return true;
}

}

