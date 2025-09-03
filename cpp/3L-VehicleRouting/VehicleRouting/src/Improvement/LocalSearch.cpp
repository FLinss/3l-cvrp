

#include "Improvement/LocalSearch.h"

namespace VehicleRouting
{
namespace Improvement
{

// Build operator lists once, from whatever vectors your config provides
LocalSearch::LocalSearch(const VRP_InputParameters* const params,
                        const Instance* const inst,
                        const Helper::Timer* const timer,
                        ContainerLoading::BaseLoadingChecker* checker,
                        std::mt19937& rng)
                        : mInstance(inst),
                        mInputParameters(params),
                        mTimer(timer),
                        mLoadingChecker(checker),
                        mRNG(rng)
{   
    for (auto t : params ->IteratedLocalSearch.localSearchTypes)
        if (auto op = this->CreateLocalSearchOperator(t))
            lsOperators.emplace_back(std::move(op));

    for (auto t : params ->IteratedLocalSearch.perturbationTypes)
        if (auto op = this->CreatePerturbationOperator(t))
            pertOperators.emplace_back(std::move(op));
}


// Run all local‑search moves in order
void LocalSearch::RunLocalSearch(Model::Solution& sol) const
{
    for (auto& op : lsOperators){
        op->Run(mInstance, mInputParameters, mLoadingChecker, mTimer, sol);
    }
};

// Run all perturbations in order
void LocalSearch::RunPerturbation(Model::Solution& sol) const
{
    for (auto& op : pertOperators){
        //TODO handles nullptr case! 
        if(op){
            op->Run(mInstance, mInputParameters, mLoadingChecker, mTimer, sol, mRNG);
        }
        break;
    }
};

// Run all perturbations in order
void LocalSearch::RunBigPerturbation(Model::Solution&  sol) const 
{
    for (auto& op : pertOperators)
        //TODO handles nullptr case! 
        if(op){
            op->Run(mInstance, mInputParameters, mLoadingChecker, mTimer, sol, mRNG);
        }
};

std::unique_ptr<LocalSearchOperatorBase> LocalSearch::CreateLocalSearchOperator(const VRP_LocalSearchtypes& t) const
{
    switch (t)
    {
        case VRP_LocalSearchtypes::TwoOpt:          
            return std::make_unique<TwoOpt>();
        case VRP_LocalSearchtypes::IntraSwap:       
            return std::make_unique<IntraSwap>();
        case VRP_LocalSearchtypes::IntraInsertion:       
            return std::make_unique<IntraInsertion>();
        case VRP_LocalSearchtypes::InterSwap:       
            return std::make_unique<InterSwap>();
        case VRP_LocalSearchtypes::InterInsertion:       
            return std::make_unique<InterInsertion>();
        case VRP_LocalSearchtypes::DeleteEmptyRoutes:       
            return std::make_unique<DeleteEmptyRoutes>();
        default:                                
            return nullptr;                       
    }
}

std::unique_ptr<PerturbationOperatorBase> LocalSearch::CreatePerturbationOperator(const VRP_PerturbationTypes& t) const
{
        switch (t)
        {
            case VRP_PerturbationTypes::K_RandomSwaps:  
                return std::make_unique<K_RandomSwaps>();
            case VRP_PerturbationTypes::K_RandomInsertions:
                return std::make_unique<K_RandomInsertions>();
            default:                               
                return nullptr;
        }
};

}} // namespace VehicleRouting::Improvement