#pragma once

#include "Model/Container.h"
#include "LoadingStatus.h"
#include <boost/dynamic_bitset/dynamic_bitset.hpp>

#include <unordered_map>
#include <vector>

namespace ContainerLoading
{
namespace Algorithms
{

using int64 = int64_t;
using PlacementPoints1D = std::vector<bool>;
using PlacementPoints2D = std::vector<std::vector<bool>>;
using PlacementPoints3D = std::vector<std::vector<std::vector<bool>>>;

enum class PlacementPattern
{
    None,
    UnitDiscretization,
    NormalPatterns,
    ReducedRasterPoints,
    RegularNormalPatterns,
    MeetInTheMiddle
};

struct ItemPlacementPoints
{
    PlacementPoints3D Coordinates;
};

struct ItemPlacementPatterns
{
    std::vector<int64> X;
    std::vector<int64> Y;
    std::vector<int64> Z;
};

struct ItemPlacementPatternsBitset
{
    boost::dynamic_bitset<> X;
    boost::dynamic_bitset<> Y;
    boost::dynamic_bitset<> Z;
};

class PlacementPointGenerator
{
  public:
    enum class MeetInTheMiddleMinimizationTarget
    {
        /// Minimize |M_is|
        IndividualPlacementPoints,

        /// Minimize |M_s|
        PlacementPointUnion
    };

    static std::tuple<PlacementPattern, PlacementPattern, PlacementPattern>
        SelectMinimalFeasiblePatternType(LoadingFlag loadingMask);

    /// Keep std::vector versions for reference.
    /*
    static std::vector<int64> DetermineNormalPatternsX(
        const Model::Container& container,
        const std::vector<Model::Cuboid>& items);

    static std::vector<int64> DetermineNormalPatternsY(
        const Model::Container& container,
        const std::vector<Model::Cuboid>& items);

    static std::vector<int64> DetermineNormalPatternsZ(
        const Model::Container& container,
        const std::vector<Model::Cuboid>& items);
    */

    /// Regular normal patterns according to Côté, J. F., & Iori, M. (2018). The meet-in-the-middle principle for
    /// cutting and packing problems. INFORMS Journal on Computing, 30(4), 646-661.
    static boost::dynamic_bitset<>
        DetermineRegularNormalPatternsX(int containerDx, int actualContainerDx, const std::vector<Model::Cuboid*>& items);

    static boost::dynamic_bitset<>
        DetermineRegularNormalPatternsY(int containerDy, int actualContainerDy, const std::vector<Model::Cuboid*>& items);

    static boost::dynamic_bitset<>
        DetermineRegularNormalPatternsZ(int containerDz, int actualContainerDz, const std::vector<Model::Cuboid*>& items);

    static std::tuple<std::vector<int64>, std::vector<int64>, std::vector<int64>>
        DetermineMeetInTheMiddlePatterns(const Model::Container& container, std::vector<Model::Cuboid>& items);

    static std::vector<ItemPlacementPatternsBitset>
        DetermineMinimalMeetInTheMiddlePatterns(const Model::Container& container,
                                                std::vector<Model::Cuboid>& items,
                                                MeetInTheMiddleMinimizationTarget minimizationTarget);
    static std::vector<boost::dynamic_bitset<>>
        DetermineMinimalMeetInTheMiddlePatterns(const Model::Container& container,
                                                std::vector<Model::Cuboid>& items,
                                                MeetInTheMiddleMinimizationTarget minimizationTarget,
                                                Model::Axis axis);

    static std::vector<boost::dynamic_bitset<>>
        DetermineUnitDiscretizationPoints(const Model::Container& container, std::vector<Model::Cuboid>& items, Model::Axis axis);
    static std::vector<boost::dynamic_bitset<>>
        GenerateRegularNormalPatterns(const Model::Container& container, std::vector<Model::Cuboid>& items, Model::Axis axis);

    static std::tuple<boost::dynamic_bitset<>, boost::dynamic_bitset<>, boost::dynamic_bitset<>>
        DetermineItemSpecificMeetInTheMiddlePatterns(const Model::Container& container,
                                                     const std::vector<Model::Cuboid*>& items,
                                                     const Model::Cuboid& itemI);

    static boost::dynamic_bitset<> DetermineRegularNormalPatterns(const Model::Container& container,
                                                                  const std::vector<Model::Cuboid*>& items,
                                                                  const Model::Cuboid& itemI,
                                                                  std::vector<int>& meetInTheMiddlePointsLeftX,
                                                                  std::vector<int>& meetInTheMiddlePointsRightX,
                                                                  MeetInTheMiddleMinimizationTarget minimizationTarget,
                                                                  Model::Axis axis);

    static boost::dynamic_bitset<> DetermineMeetInTheMiddlePatterns(const Model::Container& container,
                                                                    const std::vector<Model::Cuboid*>& items,
                                                                    const Model::Cuboid& itemI,
                                                                    int threshold,
                                                                    Model::Axis axis);

    /// Preprocessing steps according to Côté, J. F., & Iori, M. (2018). The meet-in-the-middle principle for cutting
    /// and packing problems. INFORMS Journal on Computing, 30(4), 646-661.
    static std::vector<boost::dynamic_bitset<>> DetermineReducedMeetInTheMiddlePatterns(std::vector<Model::Cuboid>& items,
                                                                                        const Model::Container& container,
                                                                                        int threshold,
                                                                                        Model::Axis axis,
                                                                                        bool enablePreprocessingStep1,
                                                                                        bool enablePreprocessingStep2);

    static void DetermineSingleItemReducedLeftRightPatterns(int threshold,
                                                            int separationThreshold,
                                                            boost::dynamic_bitset<>& placementPointsLeft,
                                                            const Model::Container& container,
                                                            Model::Axis axis,
                                                            int minSelectedItemDimension,
                                                            const std::vector<Model::Cuboid*>& doublyFilteredItems,
                                                            boost::dynamic_bitset<>& placementPointsRightPrime,
                                                            const std::vector<Model::Cuboid*>& filteredItems);

    static void DetermineEnlargedItemDimensionsLeft(
        std::vector<Model::Cuboid>& items,
        Model::Axis axis,
        std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
        const std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsLeft,
        const Model::Container& container,
        const std::vector<boost::dynamic_bitset<>>& preliminaryItemSpecificMeetInTheMiddleSets);
    static void DetermineEnlargedItemDimensionsRight(
        std::vector<Model::Cuboid>& items,
        Model::Axis axis,
        std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
        const std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsLeft,
        std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsRight,
        const std::vector<boost::dynamic_bitset<>>& preliminaryItemSpecificMeetInTheMiddleSets);

    static void RemoveRedundantPatterns(std::vector<Model::Cuboid>& items,
                                        const std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
                                        std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsLeft);

    static std::vector<boost::dynamic_bitset<>>
        GenerateMeetInTheMiddlePatterns(std::vector<Model::Cuboid>& items,
                                        const Model::Container& container,
                                        const std::vector<boost::dynamic_bitset<>>& regularNormalPatterns,
                                        int threshold,
                                        Model::Axis axis);

    static std::vector<int64> ConvertPlacementBitsetToVector(const boost::dynamic_bitset<>& itemSpecificPatternBitset);
    static ItemPlacementPatterns
        ConvertPlacementBitsetToVector(const boost::dynamic_bitset<>& itemSpecificPatternBitsetX,
                                       const boost::dynamic_bitset<>& itemSpecificPatternBitsetY,
                                       const boost::dynamic_bitset<>& itemSpecificPatternBitsetZ);
    static ItemPlacementPatterns
        ConvertPlacementBitsetToVector(const ItemPlacementPatternsBitset& itemSpecificPatternBitset);

    ////static std::vector<int64> DetermineStartPointsFlatSpan(const std::vector<int64>& patterns); // Keep for
    /// reference.
    static std::vector<int64>
        DetermineEndPoints(const std::vector<int64>& patterns, int dim, int rotDim, bool enableRotation);

    static std::vector<boost::dynamic_bitset<>> GeneratePlacementPatterns(const Model::Container& container,
                                                                          std::vector<Model::Cuboid>& items,
                                                                          PlacementPattern placementPatternType,
                                                                          Model::Axis axis);

    static std::vector<std::vector<int64>> GeneratePlacementPatternsBaseType(const Model::Container& container,
                                                                             std::vector<Model::Cuboid>& items,
                                                                             PlacementPattern placementPatternType,
                                                                             Model::Axis axis);

    static std::unordered_map<Model::Cuboid, ItemPlacementPatterns, Model::HomogeneityHash, Model::HomogeneityHash>
        GeneratePlacementPatterns(const Model::Container& container,
                                  std::vector<Model::Cuboid>& items,
                                  PlacementPattern placementPatternTypeX,
                                  PlacementPattern placementPatternTypeY,
                                  PlacementPattern placementPatternTypeZ);

    static std::tuple<std::unordered_map<Model::Cuboid, ItemPlacementPatterns, Model::HomogeneityHash, Model::HomogeneityHash>,
                      ItemPlacementPatterns>
        GeneratePlacementPatternsWithUnion(const Model::Container& container,
                                           std::vector<Model::Cuboid>& items,
                                           PlacementPattern placementPatternTypeX,
                                           PlacementPattern placementPatternTypeY,
                                           PlacementPattern placementPatternTypeZ);

    static void CountPlacementPoints(const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetX,
                                     const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetY,
                                     const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetZ);
};

}
}