#include "LoadingChecker/HybridLoadingChecker.h"

namespace ContainerLoading
{

bool HybridLoadingChecker::CompleteCheckStartSolution(const Model::Container& container,
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

   if(Parameters.UseFilterStartSolution){

        if(mClassifier->classify(items,stopIds,container)){

            auto cpStatus = ConstraintProgrammingSolver(CLP_PackingType::Complete,
                                                        container,
                                                        set,
                                                        stopIds,
                                                        items,
                                                        false,
                                                        maxRuntime);

            return cpStatus == CLP_LoadingStatus::FeasOpt;
        }
        return false;

   }else{

        auto cpStatus = ConstraintProgrammingSolver(CLP_PackingType::Complete,
                                                    container,
                                                    set,
                                                    stopIds,
                                                    items,
                                                    false,
                                                    maxRuntime);

        return cpStatus == CLP_LoadingStatus::FeasOpt;

   }
}

bool HybridLoadingChecker::CompleteCheck(const Model::Container& container,
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
    
    if(Parameters.UseClassifierLocalSearch.at(localsearchtype)){
        
        return mClassifier->classify(items,stopIds,container);

    }else{

        if(mClassifier->classify(items,stopIds,container)){

        auto cpStatus = ConstraintProgrammingSolver(CLP_PackingType::Complete,
                                                container,
                                                set,
                                                stopIds,
                                                items,
                                                false,
                                                maxRuntime);

        return cpStatus == CLP_LoadingStatus::FeasOpt;

        }
        return false;

    }
}

bool HybridLoadingChecker::ExactCheckNoClassifier(const Model::Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Model::Cuboid>& items,
                                        double maxRuntime){
    
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

}