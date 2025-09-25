#include "LoadingChecker/BaseLoadingChecker.h"
#include "Algorithms/SingleContainer/OPP_CP_3D.h"

namespace ContainerLoading
{

std::vector<Model::Cuboid> BaseLoadingChecker::SelectItems(const Collections::IdVector& nodeIds,
                                                std::vector<Model::Group>& nodes,
                                                bool reversedDirection) const
{
    std::vector<Model::Cuboid> selectedItems;
    selectedItems.reserve(nodes.size() * 3);
    if (!reversedDirection)
    {
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            auto& items = nodes[nodeIds[i]].Items;

            for (auto& item: items)
            {
                item.GroupId = nodeIds.size() - 1 - i;
                selectedItems.push_back(item);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            auto& items = nodes[nodeIds[i]].Items;

            for (auto& item: items)
            {
                item.GroupId = i;
                selectedItems.push_back(item);
            }
        }
    }

    return selectedItems;
}

CLP_LoadingStatus BaseLoadingChecker::ConstraintProgrammingSolver(CLP_PackingType packingType,
                                                          const Model::Container& container,
                                                          const boost::dynamic_bitset<>& set,
                                                          const Collections::IdVector& stopIds,
                                                          const std::vector<Model::Cuboid>& items,
                                                          bool isCallTypeExact,
                                                          double maxRunTime)
{
    if(maxRunTime < 0.0 + 1e-5)
    {
        return CLP_LoadingStatus::Unknown;
    }

    auto loadingMask = BuildMask(packingType);

    auto precheckStatus = GetPrecheckStatusCP(stopIds, set, loadingMask, isCallTypeExact);
    if (precheckStatus != CLP_LoadingStatus::Invalid)
    {
        return precheckStatus;
    }

    auto numberStops = stopIds.size();
    auto containerLoadingCP = Algorithms::ContainerLoadingCP(Parameters,
                                                 container,
                                                 items,
                                                 numberStops,
                                                 loadingMask,
                                                 Parameters.SupportArea,
                                                 maxRunTime);

    auto status = containerLoadingCP.Solve();

    if (status == CLP_LoadingStatus::Invalid)
    {
        throw std::runtime_error("Loading status invalid in CP model!");
    }

    if (isCallTypeExact && status == CLP_LoadingStatus::Unknown)
    {
        return CLP_LoadingStatus::Invalid;
    }

    AddStatus(stopIds, set, loadingMask, status);

    return status;
}

CLP_LoadingStatus BaseLoadingChecker::ConstraintProgrammingSolverGetPacking(CLP_PackingType packingType,
                                                                        const Model::Container& container,
                                                                        const Collections::IdVector& stopIds,
                                                                        std::vector<Model::Cuboid>& items,
                                                                        double maxRuntime) const
{
    if (maxRuntime < 0.0 + 1e-5)
    {
        return CLP_LoadingStatus::Invalid;
    }

    auto loadingMask = BuildMask(packingType);

    auto numberStops = stopIds.size();

    auto containerLoadingCP = Algorithms::ContainerLoadingCP(Parameters,
                                                 container,
                                                 items,
                                                 numberStops,
                                                 loadingMask,
                                                 Parameters.SupportArea,
                                                 maxRuntime);

    auto status = containerLoadingCP.Solve();

    if (status == CLP_LoadingStatus::Invalid)
    {
        throw std::runtime_error("Loading status invalid in CP model!");
    }

    if (status == CLP_LoadingStatus::FeasOpt)
    {
        containerLoadingCP.ExtractPacking(items);
    }

    return status;
}


void BaseLoadingChecker::AddFeasibleSequenceFromOutside(const Collections::IdVector& route) { AddFeasibleRoute(route); }

bool BaseLoadingChecker::RouteIsInFeasSequences(const Collections::IdVector& route) const
{
    return mFeasSequences.at(Parameters.LoadingFlags).contains(route);
}

bool BaseLoadingChecker::RouteIsInInfeasSequences(const Collections::IdVector& route) const
{
    return mInfSequences.at(Parameters.LoadingFlags).contains(route);
}

boost::dynamic_bitset<> BaseLoadingChecker::MakeBitset(size_t size, const Collections::IdVector& sequence) const
{
    boost::dynamic_bitset<> set(size);
    for (const auto i: sequence)
    {
        set.set(i);
    }

    return set;
};

void BaseLoadingChecker::AddFeasibleRoute(const Collections::IdVector& route)
{
    mFeasSequences[Parameters.LoadingFlags].insert(route);
    mCompleteFeasSeq.push_back(route);
}

bool BaseLoadingChecker::SequenceIsInfeasibleCP(const Collections::IdVector& sequence, const CLP_LoadingFlag mask) const
{
    return mInfSequences.at(mask).contains(sequence);
}

bool BaseLoadingChecker::SequenceIsUnknownCP(const Collections::IdVector& sequence, const CLP_LoadingFlag mask) const
{
    return mUnkSequences.at(mask).contains(sequence);
}

bool BaseLoadingChecker::SequenceIsFeasible(const Collections::IdVector& sequence, const CLP_LoadingFlag mask) const
{
    return mFeasSequences.at(mask).contains(sequence);
}

bool BaseLoadingChecker::SetIsInfeasibleCP(const boost::dynamic_bitset<>& set, const CLP_LoadingFlag mask) const
{
    const auto& sets = mInfSets.at(mask);
    if (!IsSet(mask, CLP_LoadingFlag::Support))
    {
        // If support is disabled, set S is infeasible when S is a superset of an infeasible set.
        auto setComparer = [set](const boost::dynamic_bitset<>& infeasibleSet)
        { return (set & infeasibleSet).count() == infeasibleSet.count(); };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }
    else
    {
        // If support is enabled, only exact matching of sets can be used as adding additional items can lead to
        // feasibility.
        auto setComparer = [set](const boost::dynamic_bitset<>& feasibleCombination)
        { return set == feasibleCombination; };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }

    return false;
}

bool BaseLoadingChecker::SetIsUnknownCP(const boost::dynamic_bitset<>& set, const CLP_LoadingFlag mask) const
{
    const auto& sets = mUnknownSets.at(mask);

    auto setComparer = [set](const boost::dynamic_bitset<>& feasibleCombination) { return set == feasibleCombination; };

    if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
    {
        return true;
    }

    return false;
}

bool BaseLoadingChecker::SetIsFeasibleCP(const boost::dynamic_bitset<>& set, const CLP_LoadingFlag mask) const
{
    const auto& sets = mFeasibleSets.at(mask);
    if (!IsSet(mask, CLP_LoadingFlag::Support))
    {
        // If support is disabled, set S is feasible when S is a subset of a feasible set.
        auto setComparer = [set](const boost::dynamic_bitset<>& feasibleSet)
        { return (set & feasibleSet).count() == set.count(); };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }
    else
    {
        // If support is enabled, only exact matching of sets can be used as removing items can lead to infeasibility.
        auto setComparer = [set](const boost::dynamic_bitset<>& feasibleCombi) { return set == feasibleCombi; };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }

    return false;
}

CLP_LoadingFlag BaseLoadingChecker::BuildMask(CLP_PackingType type) const
{
    switch (type)
    {
        case CLP_PackingType::Complete:
            return CLP_LoadingFlag::Complete &Parameters.LoadingFlags;
        case CLP_PackingType::NoSupport:
            return CLP_LoadingFlag::NoSupport &Parameters.LoadingFlags;
        case CLP_PackingType::LifoNoSequence:
            return CLP_LoadingFlag::LifoNoSequence &Parameters.LoadingFlags;
        default:
            throw std::runtime_error("CLP_PackingType not implemented in mask builder.");
    }

    return CLP_LoadingFlag();
}

CLP_LoadingStatus BaseLoadingChecker::GetPrecheckStatusCP(const Collections::IdVector& sequence,
                                                  const boost::dynamic_bitset<>& set,
                                                  const CLP_LoadingFlag mask,
                                                  const bool isCallTypeExact)
{
    if (IsSet(mask, CLP_LoadingFlag::Sequence))
    {
        if (SequenceIsInfeasibleCP(sequence, mask))
        {
            ////std::cout << "Sequence already stored as infeasible (CP)." << "\n";
            return CLP_LoadingStatus::Infeasible;
        }

        if (SequenceIsFeasible(sequence, mask))
        {
            ////std::cout << "Sequence already stored as feasible (CP)." << "\n";
            return CLP_LoadingStatus::FeasOpt;
        }

        if (!isCallTypeExact && SequenceIsUnknownCP(sequence, mask))
        {
            ////std::cout << "Sequence already stored as unknown (CP)." << "\n";
            return CLP_LoadingStatus::Unknown;
        }
    }
    else
    {
        if (SetIsInfeasibleCP(set, mask))
        {
            ////std::cout << "Set already stored as infeasible (CP)." << "\n";
            return CLP_LoadingStatus::Infeasible;
        }

        if (SetIsFeasibleCP(set, mask))
        {
            ////std::cout << "Set already stored as feasible (CP)." << "\n";
            if (mask ==Parameters.LoadingFlags && !SequenceIsFeasible(sequence, mask))
            {
                AddFeasibleRoute(sequence);
            }

            return CLP_LoadingStatus::FeasOpt;
        }

        if (!isCallTypeExact && SetIsUnknownCP(set, mask))
        {
            ////std::cout << "Set already stored as unknown (CP)." << "\n";
            return CLP_LoadingStatus::Unknown;
        }
    }

    return CLP_LoadingStatus::Invalid;
}

void BaseLoadingChecker::AddStatus(const Collections::IdVector& sequence,
                               const boost::dynamic_bitset<>& set,
                               const CLP_LoadingFlag mask,
                               const CLP_LoadingStatus status)
{
    // Add to feasible sequences although lifo might be disabled; needed for SP heuristic.
    if (status == CLP_LoadingStatus::FeasOpt && mask ==Parameters.LoadingFlags)
    {
        AddFeasibleRoute(sequence);
        if (!IsSet(mask, CLP_LoadingFlag::Lifo))
        {
            mFeasibleSets[mask].push_back(set);
        }

        return;
    }

    // If lifo is enabled, order of stops is relevant -> sequence of ids.
    if (IsSet(mask, CLP_LoadingFlag::Sequence))
    {
        switch (status)
        {
            case CLP_LoadingStatus::FeasOpt:
            {
                mFeasSequences[mask].insert(sequence);
                return;
            }
            case CLP_LoadingStatus::Infeasible:
            {
                mInfSequences[mask].insert(sequence);
                return;
            }
            case CLP_LoadingStatus::Unknown:
            {
                mUnkSequences[mask].insert(sequence);
                return;
            }
            default:
                throw std::runtime_error("CLP_LoadingStatus invalid!");
        }
    }
    // If lifo is disabled, order of stops is not relevant -> set of ids.
    else
    {
        switch (status)
        {
            case CLP_LoadingStatus::FeasOpt:
            {
                mFeasibleSets[mask].push_back(set);
                return;
            }
            case CLP_LoadingStatus::Infeasible:
            {
                mInfSets[mask].push_back(set);
                return;
            }
            case CLP_LoadingStatus::Unknown:
            {
                mUnknownSets[mask].push_back(set);
                return;
            }
            default:
                throw std::runtime_error("CLP_LoadingStatus invalid!");
        }
    }
}

void BaseLoadingChecker::WriteSequencesToFile(const std::string& outputPath,const std::string& saveSequenceString) const
{
    nlohmann::json jsonObj;
    jsonObj["AllFeasibleRoutes"] = mFeasSequences;
    jsonObj["AllInFeasibleRoutes"] = mInfSequences;
    jsonObj["AllUnknownRoutes"] = mUnkSequences;
    jsonObj["AllFeasibleSets"] = mFeasibleSets;
    jsonObj["AllInFeasibleSets"] = mInfSets;
    jsonObj["AllUnknownSets"] = mUnknownSets;

    std::ofstream outputFile(outputPath + "/" + saveSequenceString + ".json");
    if (!outputFile.is_open())
    {
        std::cerr << "Unable to open the route file!\n";
        return;
    }

    // manually write top-level keys with compact value per line
    outputFile << "{\n";
    outputFile << "\"Sequences\":{\n";
    outputFile << "  \"AllFeasible\": " << jsonObj["AllFeasibleRoutes"].dump() << ",\n";
    outputFile << "  \"AllInFeasibles\": " << jsonObj["AllInFeasibleRoutes"].dump() << ",\n";
    outputFile << "  \"AllUnknown\": " << jsonObj["AllUnknownRoutes"].dump() << "\n";
    outputFile << "},\n";
    outputFile << "\"Sets\":{\n";
    outputFile << "  \"AllFeasible\": " << jsonObj["AllFeasibleSets"].dump() << ",\n";
    outputFile << "  \"AllInFeasible\": " << jsonObj["AllInFeasibleSets"].dump() << ",\n";
    outputFile << "  \"AllUnknown\": " << jsonObj["AllUnknownSets"].dump() << "\n";
    outputFile << "}\n";
    outputFile << "}\n";
}


}