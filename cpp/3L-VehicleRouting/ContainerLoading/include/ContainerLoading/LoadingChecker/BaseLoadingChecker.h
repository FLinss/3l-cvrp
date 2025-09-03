#pragma once

#include "CommonBasics/Helper/ModelServices.h"

#include "ProblemParameters.h"
#include "Model/ContainerLoadingInstance.h"
#include "Model/Container.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/functional/hash.hpp>
#include <memory>

namespace ContainerLoading
{
using namespace Algorithms;

class BaseLoadingChecker
{
  public:
    const ContainerLoadingParams Parameters;

    explicit BaseLoadingChecker(const ContainerLoadingParams& parameters) : Parameters(parameters)
    {
        using enum LoadingFlag;

        std::vector<LoadingFlag> usedLoadingFlags = {Complete, NoSupport, LifoNoSequence};

        constexpr size_t reservedSize = 1000;
        for (const auto flag: usedLoadingFlags)
        {
            mFeasSequences[flag & Parameters.LoadingFlags].reserve(reservedSize);
            mInfSequences[flag & Parameters.LoadingFlags].reserve(reservedSize);
            mUnkSequences[flag & Parameters.LoadingFlags].reserve(reservedSize);

            mFeasibleSets[flag & Parameters.LoadingFlags].reserve(reservedSize);
            mInfSets[flag & Parameters.LoadingFlags].reserve(reservedSize);
            mUnknownSets[flag & Parameters.LoadingFlags].reserve(reservedSize);
        }
    }

    virtual bool CompleteCheck(const Model::Container& container,
                                const boost::dynamic_bitset<>& set,
                                const Collections::IdVector& stopIds,
                                const std::vector<Model::Cuboid>& items,
                                const VehicleRouting::Improvement::ImprovementTypes& localsearchtype,
                                double maxRuntime) = 0;

    virtual bool CompleteCheckStartSolution(const Model::Container& container,
                            const boost::dynamic_bitset<>& set,
                            const Collections::IdVector& stopIds,
                            const std::vector<Model::Cuboid>& items,
                            double maxRuntime) = 0;

    virtual bool ExactCheckNoClassifier(const Model::Container& container,
                                        const boost::dynamic_bitset<>& set,
                                        const Collections::IdVector& stopIds,
                                        const std::vector<Model::Cuboid>& items,
                                        double maxRuntime) = 0;

    [[nodiscard]] std::vector<Model::Cuboid>
        SelectItems(const Collections::IdVector& nodeIds, std::vector<Model::Group>& nodes, bool reversedDirection) const;

    [[nodiscard]] LoadingStatus ConstraintProgrammingSolver(PackingType packingType,
                                                            const Model::Container& container,
                                                            const boost::dynamic_bitset<>& set,
                                                            const Collections::IdVector& stopIds,
                                                            const std::vector<Model::Cuboid>& items,
                                                            bool isCallTypeExact,
                                                            double maxRuntime);

    [[nodiscard]] LoadingStatus ConstraintProgrammingSolverGetPacking(PackingType packingType,
                                                                      const Model::Container& container,
                                                                      const Collections::IdVector& stopIds,
                                                                      std::vector<Model::Cuboid>& items,
                                                                      double maxRuntime) const;

    [[nodiscard]] double GetElapsedTime();

    void AddFeasibleSequenceFromOutside(const Collections::IdVector& route);

    [[nodiscard]] bool RouteIsInFeasSequences(const Collections::IdVector& route) const;

    [[nodiscard]] bool RouteIsInInfeasSequences(const Collections::IdVector& route) const;

    [[nodiscard]] boost::dynamic_bitset<> MakeBitset(size_t size, const Collections::IdVector& sequence) const;

  private:
    Collections::SequenceVector mCompleteFeasSeq;
    /// Set of customer combinations that are infeasible.
    /// -> There is no path in combination C that respects all constraints
    /// -> At least 2 vehicles are needed to serve all customers in C
    std::vector<boost::dynamic_bitset<>> mInfeasibleCustomerCombinations;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mFeasibleSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mFeasSequences;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mInfSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mInfSequences;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mUnknownSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mUnkSequences;

    void AddFeasibleRoute(const Collections::IdVector& route);

    [[nodiscard]] bool SequenceIsInfeasibleCP(const Collections::IdVector& sequence, LoadingFlag mask) const;
    [[nodiscard]] bool SequenceIsUnknownCP(const Collections::IdVector& sequence, LoadingFlag mask) const;
    [[nodiscard]] bool SequenceIsFeasible(const Collections::IdVector& sequence, LoadingFlag mask) const;

    [[nodiscard]] bool SetIsInfeasibleCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;
    [[nodiscard]] bool SetIsUnknownCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;
    [[nodiscard]] bool SetIsFeasibleCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;

    [[nodiscard]] LoadingFlag BuildMask(PackingType type) const;

    [[nodiscard]] LoadingStatus GetPrecheckStatusCP(const Collections::IdVector& sequence,
                                                    const boost::dynamic_bitset<>& set,
                                                    LoadingFlag mask,
                                                    bool isCallTypeExact);

    void AddStatus(const Collections::IdVector& sequence,
                   const boost::dynamic_bitset<>& set,
                   LoadingFlag mask,
                   LoadingStatus status);

};
}