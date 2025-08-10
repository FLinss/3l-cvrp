#include "LoadingChecker/SpeedUpLoadingChecker.h"

namespace ContainerLoading
{
using namespace Algorithms;


bool SpeedUpLoadingChecker::CompleteCheckStartSolution(const Container& container,
                const boost::dynamic_bitset<>& set,
                const Collections::IdVector& stopIds,
                const std::vector<Cuboid>& items)
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
                                                    false);

            return cpStatus == LoadingStatus::FeasOpt;
        }
        return false;

   }else{

        auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                                        container,
                                                        set,
                                                        stopIds,
                                                        items,
                                                        false);

        return cpStatus == LoadingStatus::FeasOpt;

   }
}

bool SpeedUpLoadingChecker::CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items,
                                    const VehicleRouting::Improvement::ImprovementTypes& localsearchtype
                                    )
{
    if (RouteIsInFeasSequences(stopIds))
    {
        return true;
    }

    if (RouteIsInInfeasSequences(stopIds))
    {
        return false;
    }

    //Only use classifier!
    return mClassifier->classify(items,stopIds,container);
}

bool SpeedUpLoadingChecker::ExactCheckNoClassifier(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items){
    
    if (RouteIsInFeasSequences(stopIds))
    {
        return true;
    }

    if (RouteIsInInfeasSequences(stopIds))
    {
        return false;
    }

    auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                                            container,
                                                            set,
                                                            stopIds,
                                                            items,
                                                            false);

    return cpStatus == LoadingStatus::FeasOpt;

}
}