#include "LoadingChecker/FilterLoadingChecker.h"

namespace ContainerLoading
{
using namespace Algorithms;


bool FilterLoadingChecker::CompleteCheckStartSolution(const Container& container,
                const boost::dynamic_bitset<>& set,
                const Collections::IdVector& stopIds,
                const std::vector<Cuboid>& items,
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

   if(Parameters.UseFilterStartSolution){

        if(mClassifier->classify(items,stopIds,container)){

            auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                                    container,
                                                    set,
                                                    stopIds,
                                                    items,
                                                    false,
                                                    maxRuntime);

            return cpStatus == LoadingStatus::FeasOpt;
        }
        return false;

   }else{

        auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                                        container,
                                                        set,
                                                        stopIds,
                                                        items,
                                                        false,
                                                        maxRuntime);

        return cpStatus == LoadingStatus::FeasOpt;

   }
}

bool FilterLoadingChecker::CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items,
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
    
    
    if(mClassifier->classify(items,stopIds,container)){

        auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                                container,
                                                set,
                                                stopIds,
                                                items,
                                                false,
                                                maxRuntime);

        return cpStatus == LoadingStatus::FeasOpt;

    }else{
        return false;
    }
}

bool FilterLoadingChecker::ExactCheckNoClassifier(const Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Cuboid>& items,
                                        double maxRuntime){
    
    return true;
}

}