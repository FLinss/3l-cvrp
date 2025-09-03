#pragma once

#include "ContainerLoading/ProblemParameters.h"
#include <unordered_map>

// NOLINTBEGIN(readability-magic-numbers)

namespace VehicleRouting
{
namespace Algorithms
{

enum class LocalSearchTypes
{
    None,
    TwoOpt,
    InterSwap,
    IntraSwap,
    InterInsertion,
    IntraInsertion,
    FullEnumeration,
    DeleteEmptyRoutes
};

enum class LoadingCheckerTypes{
    Filter,
    NoClassifier,
    SpeedUp,
    Hybrid
};

enum class PerturbationTypes
{
    None,
    K_RandomSwaps,
    K_RandomInsertions
};


struct IteratedLocalSearchParams
{
  public:
    enum class StartSolutionType
    {
        None = 0,
        ModifiedSavings,
        Savings,
        SPHeuristic
    };

    enum class CallType
    {
        None,
        Exact,
        ExactLimit,
        ILS,
        Constructive
    };

    bool RunILS = false;
    bool RunLS = true;
    int NoImprLimit = 100;
    int K_RandomMoves = 1;
    int MaxIterationsWithoutImprovement = 10000;
    int RoundsWithNoImprovement = 3;
    LoadingCheckerTypes LoadingCheckerType = LoadingCheckerTypes::NoClassifier;
    StartSolutionType StartSolution = StartSolutionType::ModifiedSavings;
    bool CP_Check = false;
    int Interval_CP_Check = 1;

    std::vector<PerturbationTypes> perturbationTypes = {PerturbationTypes::None};
    std::vector<LocalSearchTypes> localSearchTypes = {LocalSearchTypes::None};

    std::unordered_map<CallType, double> TimeLimits = {
        {CallType::Exact, std::numeric_limits<double>::max()},
        {CallType::ExactLimit, 1.0},
        {CallType::ILS, 120.0},
        {CallType::Constructive, 10.0}
    };
};

class InputParameters
{
  public:

    IteratedLocalSearchParams IteratedLocalSearch;
    ContainerLoading::ContainerLoadingParams ContainerLoading;

    void SetLoadingFlags(){
        ContainerLoading.SetFlags();
    };

    [[nodiscard]] double DetermineMaxRuntime(IteratedLocalSearchParams::CallType callType,
                                             double residualTime = std::numeric_limits<double>::max()) const
    {
        return std::min(IteratedLocalSearch.TimeLimits.at(callType), residualTime);
    }

    [[nodiscard]] bool IsExact(IteratedLocalSearchParams::CallType callType) const
    {
        return callType == IteratedLocalSearchParams::CallType::Exact;
    }
};

}
}

// NOLINTEND(readability-magic-numbers)