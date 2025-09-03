#pragma once

#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/FilterLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/HybridLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/NoClassifierLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/SpeedUpLoadingChecker.h"
#include "ContainerLoading/Algorithms/LoadingStatus.h"
#include "ContainerLoading/Helper/HelperIO.h"
#include "Improvement/LocalSearch.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "Algorithms/Constructive.h"
#include "CommonBasics/Helper/ModelServices.h"
#include "Helper/HelperIO.h"
#include "Helper/Serialization.h"
#include "Helper/Timer.h"
#include "Model/Instance.h"
#include "Model/Solution.h"

#include <fstream>
#include <iostream>
#include <random>
#include <cstdint>
#include <memory>

namespace VehicleRouting
{
namespace Algorithms
{
class IteratedLocalSearch
{
  public:
  IteratedLocalSearch(Model::Instance* instance,
                      const InputParameters& inputParameters,
                      const std::string& startSolutionFolderPath,
                      const std::string& outputPath,
                      const int seedOffset)
    : mInstance(instance),
      mInputParameters(inputParameters),
      mStartSolutionFolderPath(startSolutionFolderPath),
      mOutputPath(outputPath),
      mSeedOffset(seedOffset),
      mSolutionTracker(mSeedOffset)
    {
        //Initialize RNG 
        mRNG.seed(42 + seedOffset);

        switch (mInputParameters.IteratedLocalSearch.LoadingCheckerType)
        {
            case LoadingCheckerTypes::Filter:          
                mLoadingChecker = std::make_unique<ContainerLoading::FilterLoadingChecker>(mInputParameters.ContainerLoading);
                break;
            case LoadingCheckerTypes::NoClassifier:       
                mLoadingChecker = std::make_unique<ContainerLoading::NoClassifierLoadingChecker>(mInputParameters.ContainerLoading);
                break;
            case LoadingCheckerTypes::SpeedUp:       
                mLoadingChecker = std::make_unique<ContainerLoading::SpeedUpLoadingChecker>(mInputParameters.ContainerLoading);
                break;
            case LoadingCheckerTypes::Hybrid:       
                mLoadingChecker = std::make_unique<ContainerLoading::HybridLoadingChecker>(mInputParameters.ContainerLoading);     
                break;               
        }

        //Initialize Local Search
        mLocalSearch = std::make_unique<Improvement::LocalSearch>(&mInputParameters, mInstance, &mTimer, mLoadingChecker.get(), mRNG);
    }

    void Solve();

  private:
    Model::Instance* mInstance;
    InputParameters mInputParameters;
    std::string mStartSolutionFolderPath;
    std::string mOutputPath;
    int mSeedOffset;

    std::ofstream mLogFile;
    Model::Solution mCurrentSolution;
    Model::Solution mBestSolution;

    Model::SolutionTracker mSolutionTracker;

    std::mt19937 mRNG;
    std::unique_ptr<ContainerLoading::BaseLoadingChecker> mLoadingChecker;
    std::unique_ptr<Improvement::LocalSearch> mLocalSearch;
    Helper::Timer mTimer = Helper::Timer();

    void TestSingleCustomerRoutes();
    void DeterminePackingSolution(Model::OutputSolution& outputSolution);
    void PrintSolution(const Model::OutputSolution& outputSolution);

    void WriteSolutionSolutionValidator(const Model::OutputSolution& outputSolution);

    void StartSolutionProcedure();
    void GenerateStartSolutionSavings();
    void GenerateStartSolutionModifiedSavings();
    void GenerateStartSolutionSPHeuristic();
    bool IsCurrentSolutionCPValid(const Model::Solution& solution);

};
}
}