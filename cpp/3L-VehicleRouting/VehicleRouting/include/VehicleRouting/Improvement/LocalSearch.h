#pragma once

#include "IntraImprovement/TwoOpt.h"
#include "InterImprovement/InterSwap.h"
#include "IntraImprovement/IntraSwap.h"
#include "InterImprovement/InterInsertion.h"
#include "IntraImprovement/IntraInsertion.h"
#include "Perturbation/K_RandomSwaps.h"
#include "Perturbation/K_RandomInsertions.h"
#include "IntraImprovement/DeleteEmptyRoutes.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{

class LocalSearch
{
public:
    // Build operator lists once, from whatever vectors your config provides
    LocalSearch(const InputParameters& params,
                const Instance* inst);

    // Run all local‑search moves in order
    void RunLocalSearch(Solution& sol,
                        ContainerLoading::BaseLoadingChecker* checker);

    // Run all perturbations in order
    void RunPerturbation(Solution& sol,
                        ContainerLoading::BaseLoadingChecker* checker,
                         std::mt19937& rng);

        // Run all perturbations in order
    void RunBigPerturbation(Solution& sol,
                        ContainerLoading::BaseLoadingChecker* checker,
                         std::mt19937& rng);

private:
    std::vector<std::unique_ptr<LocalSearchOperatorBase>>  lsOperators;
    std::vector<std::unique_ptr<PerturbationOperatorBase>> pertOperators;
    const Instance* mInstance = nullptr;
    const InputParameters mInputParameters;

    std::unique_ptr<LocalSearchOperatorBase> CreateLocalSearchOperator(const LocalSearchTypes& t);
    std::unique_ptr<PerturbationOperatorBase> CreatePerturbationOperator(const PerturbationTypes& t);
};

}} // namespace VehicleRouting::Improvement


