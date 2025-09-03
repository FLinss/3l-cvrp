#pragma once

#include "IntraImprovement/TwoOpt.h"
#include "InterImprovement/InterSwap.h"
#include "IntraImprovement/IntraSwap.h"
#include "InterImprovement/InterInsertion.h"
#include "IntraImprovement/IntraInsertion.h"
#include "Perturbation/K_RandomSwaps.h"
#include "Perturbation/K_RandomInsertions.h"
#include "DeleteEmptyRoutes.h"

#include "Helper/Timer.h"

namespace VehicleRouting
{
namespace Improvement
{

class LocalSearch
{
public:
    // Build operator lists once, from whatever vectors your config provides
    LocalSearch(const InputParameters* const params,
                const Instance* const inst,
                const Helper::Timer* const timer,
                ContainerLoading::BaseLoadingChecker* loadingChecker,
                std::mt19937& rng);

    void RunLocalSearch(Model::Solution& sol) const;
    void RunPerturbation(Model::Solution& sol) const;
    void RunBigPerturbation(Model::Solution& sol) const;

private:
    std::vector<std::unique_ptr<LocalSearchOperatorBase>>  lsOperators;
    std::vector<std::unique_ptr<PerturbationOperatorBase>> pertOperators;

    const Instance* const mInstance = nullptr;
    const Helper::Timer* const mTimer = nullptr;
    const InputParameters* const mInputParameters = nullptr;
    ContainerLoading::BaseLoadingChecker* mLoadingChecker = nullptr;

    std::mt19937& mRNG;

    [[nodiscard]] std::unique_ptr<LocalSearchOperatorBase> CreateLocalSearchOperator(const LocalSearchTypes& t) const;
    [[nodiscard]] std::unique_ptr<PerturbationOperatorBase> CreatePerturbationOperator(const PerturbationTypes& t) const;
};

}} // namespace VehicleRouting::Improvement


