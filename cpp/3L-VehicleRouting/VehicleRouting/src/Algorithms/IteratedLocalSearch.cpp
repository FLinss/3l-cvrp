#include "Algorithms/IteratedLocalSearch.h"

namespace VehicleRouting
{
namespace Algorithms
{

using CLP_Container =  ContainerLoading::Model::Container;
using CLP_Cuboid =  ContainerLoading::Model::Cuboid;
using CLP_Group =  ContainerLoading::Model::Group;
using CLP_PackingType = ContainerLoading::Algorithms::PackingType;
using CLP_LoadingStatus = ContainerLoading::Algorithms::LoadingStatus;


void IteratedLocalSearch::TestSingleCustomerRoutes()
{
    mLogFile << "ProblemVariant: " << (int)mInputParameters.ContainerLoading.Variant << "\n";

    CLP_Container container =  mInstance->Vehicles.front().Containers.front();

    double maxRuntime = mInputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ILS);
    for (const auto& customer: mInstance->GetCustomers())
    {
        Collections::IdVector route = {customer.InternId};

        auto items = InterfaceConversions::SelectItems(route, mInstance->Nodes, false);

        auto exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(CLP_PackingType::Complete,
                                                         container,
                                                         mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route),
                                                         route,
                                                         items,
                                                         mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact),
                                                         maxRuntime);

        if (exactStatus != CLP_LoadingStatus::FeasOpt)
        {
            mLogFile << "Single customer route with " << customer.InternId << "is infeasible.\n";

            auto relStatus = mLoadingChecker->ConstraintProgrammingSolver(
                CLP_PackingType::NoSupport,
                container,
                mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route),
                route,
                items,
                mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact),
                maxRuntime);

            if (relStatus != CLP_LoadingStatus::FeasOpt)
            {
                throw std::runtime_error("Single customer route is infeasible!");
            }
        }
    }
}

void IteratedLocalSearch::StartSolutionProcedure()
{
    mLogFile << "## Start start solution procedure ##\n";

    using enum IteratedLocalSearchParams::StartSolutionType;

    switch (mInputParameters.IteratedLocalSearch.StartSolution)
    {
        case None:
            return;
        case ModifiedSavings:
            GenerateStartSolutionModifiedSavings();
            break;
        case Savings:
            GenerateStartSolutionSavings();
            
            if (mCurrentSolution.Routes.size() > mInstance->Vehicles.size()){
                mLogFile << "Not enough vehicles for start solution, copying existing vehicle(s)...\n";

                size_t deficit = mCurrentSolution.Routes.size() - mInstance->Vehicles.size();
                const Vehicle& templateVehicle = mInstance->Vehicles.back();  // or Vehicles[0], your choice

                for (size_t i = 0; i < deficit; ++i)
                {
                    Vehicle newVehicle = templateVehicle;
                    newVehicle.InternId = static_cast<int>(mInstance->Vehicles.size());  // Ensure unique ID if needed
                    mInstance->Vehicles.push_back(std::move(newVehicle));
                }

                mLogFile << "Copied " << deficit << " vehicle(s). Total vehicles now: "
                        << mInstance->Vehicles.size() << "\n";
            }
            break;
        default:
            throw std::runtime_error("Start solution type not implemented.");
    }

    mCurrentSolution.NumberOfRoutes = mCurrentSolution.Routes.size();
    mCurrentSolution.DetermineCosts(mInstance);
    //TODO Change in Repar Modified Saavings, that Totalvoluem and Totalweight is updated! 
    mCurrentSolution.DeterminWeightsVolumes(mInstance);
    mBestSolution = mCurrentSolution;

    //Initital Local Search
    if(mInputParameters.IteratedLocalSearch.RunLS){

       mLocalSearch->RunLocalSearch(mCurrentSolution);

        //Wont be applied, when current = best solution
        if(mInputParameters.IteratedLocalSearch.CP_Check){
            if(!(IsCurrentSolutionCPValid(mCurrentSolution))){
                mCurrentSolution = mBestSolution;
            }
        }
        if(mCurrentSolution.Costs < mBestSolution.Costs){
            mBestSolution = mCurrentSolution;
        }
    }

    // TODO change, here just for intializing
    mSolutionTracker.UpdateBothSolutions(mTimer.getElapsedTime(), mBestSolution.Costs);

    OutputSolution outputSolution(mCurrentSolution, mInstance);

    // Save values of start solution
    mLogFile << "Start solution with " << outputSolution.NumberOfRoutes << " Vehicles and total costs "
             << outputSolution.Costs << " in " << mTimer.getElapsedTime() << " s.\n";

    std::string solutionString = "StartSolution-" + mInstance->Name;
    Serializer::WriteToJson(outputSolution, mOutputPath, solutionString);

    mLogFile << "### END PREPROCESSING ###\n";
}

void IteratedLocalSearch::GenerateStartSolutionModifiedSavings()
{
    mCurrentSolution.Routes =
        Constructive::ModifiedSavings(mInstance,
                                        &mInputParameters,
                                        mLoadingChecker.get(),
                                        &mRNG,
                                        &mTimer).Run();

    //TODO what should i do with these values? 
    
    int id = 0;
    //Label routes new
    for (auto& route : mCurrentSolution.Routes)
    {
        route.Id = id;
        ++id;
    }
}

void IteratedLocalSearch::GenerateStartSolutionSavings()
{
    mCurrentSolution.Routes =
        Constructive::Savings(mInstance,
                             &mInputParameters,
                             mLoadingChecker.get(),
                             &mTimer).Run();

    //TODO what should i do with these values? 
    
    int id = 0;
    //Label routes new
    for (auto& route : mCurrentSolution.Routes)
    {
        route.Id = id;
        ++id;
    }
}


bool IteratedLocalSearch::IsCurrentSolutionCPValid(const Model::Solution& solution) {

    const auto& container = mInstance->Vehicles.front().Containers.front();
    const auto& nodeSize = mInstance->Nodes.size();

    for(const auto& route : solution.Routes) {

        if(route.Sequence.size() > 0){

            auto items = InterfaceConversions::SelectItems(route.Sequence, mInstance->Nodes, false);
            double maxRuntime = mInputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ILS, mTimer.getResidualTime());
            if(!(mLoadingChecker->ExactCheckNoClassifier(container,
                                                    mLoadingChecker->MakeBitset(nodeSize, route.Sequence),
                                                    route.Sequence,
                                                    items,
                                                    maxRuntime))){
                //std::cout << "Route was rejected by CPSolver" << std::endl;
                ++mSolutionTracker.rejections;
                return false;
            }
        }
    }
    return true;
}


void IteratedLocalSearch::Solve()
{
    mTimer.startOverallTime(); 

    mLogFile << "### START HEURISTIC APPROACH APPROACH ###\n";

    //Write input parameters to json file
    std::string parameterString = "Parameters-" + mInstance->Name;
    Serializer::WriteToJson(mInputParameters, mOutputPath, parameterString);

    //Test if one of the one customer routes is infeasible
    TestSingleCustomerRoutes();

    StartSolutionProcedure();
    mTimer.calculateStartSolutionTime();
    int iterations_without_improvement{0};

    Model::Solution lastValidSolution = mCurrentSolution;

    double maxRuntime = mInputParameters.IteratedLocalSearch.TimeLimits[IteratedLocalSearchParams::CallType::ILS];
    if(mInputParameters.IteratedLocalSearch.RunILS){
        while(mTimer.getElapsedTime() < maxRuntime && iterations_without_improvement < mInputParameters.IteratedLocalSearch.MaxIterationsWithoutImprovement){

            std::cout << "Run: " << mSolutionTracker.iterations << " - CurrentCosts: " << mCurrentSolution.Costs << " - BestCosts:" << mBestSolution.Costs << std::endl;

            mLocalSearch->RunPerturbation(mCurrentSolution);
            mLocalSearch->RunLocalSearch(mCurrentSolution);

            ++mSolutionTracker.iterations;
            if(mInputParameters.IteratedLocalSearch.CP_Check){
                if((mSolutionTracker.iterations % mInputParameters.IteratedLocalSearch.Interval_CP_Check) == 0){
                    if(!(IsCurrentSolutionCPValid(mCurrentSolution))){
                        mCurrentSolution = lastValidSolution;
                        --mSolutionTracker.NoImpr;
                    }
                }else{
                    continue;
                }
            }
    
            if(mCurrentSolution.Costs < mBestSolution.Costs - 1e-2){
                mSolutionTracker.UpdateBothSolutions(mTimer.getElapsedTime(), mCurrentSolution.Costs);
                mBestSolution = mCurrentSolution;
                mSolutionTracker.NoImpr = 0;
                iterations_without_improvement = 0;
                lastValidSolution = mCurrentSolution;
                continue;
            }

            mSolutionTracker.UpdateCurrSolution(mTimer.getElapsedTime(), mCurrentSolution.Costs);
            if(mSolutionTracker.NoImpr >= mInputParameters.IteratedLocalSearch.NoImprLimit){
                mCurrentSolution = mBestSolution;
                mSolutionTracker.NoImpr = 0;
            }

            ++mSolutionTracker.NoImpr;
            ++iterations_without_improvement;
            lastValidSolution = mCurrentSolution;

        }
    }

    mTimer.calculateMetaHeuristicTime();
    

    auto statistics = SolverStatistics(mTimer,
                                       mSolutionTracker);

    std::string solutionStatisticsString = "SolutionStatistics-" + mInstance->Name;
    Serializer::WriteToJson(statistics, mOutputPath, solutionStatisticsString);

    //Save Sequences
    if(mInputParameters.IteratedLocalSearch.SaveSequences){
        std::string saveSequenceString = "Routes2_" + mInstance->Name;
        mLoadingChecker->WriteSequencesToFile(mOutputPath, saveSequenceString);
    }

    OutputSolution final_outputSolution(mBestSolution, mInstance);
    DeterminePackingSolution(final_outputSolution);

    auto solFile = SolutionFile(mInputParameters, statistics, final_outputSolution);

    std::string solutionString = "Solution-" + mInstance->Name;
    Serializer::WriteToJson(solFile, mOutputPath, solutionString);

    WriteSolutionSolutionValidator(final_outputSolution);
}

void IteratedLocalSearch::DeterminePackingSolution(OutputSolution& outputSolution)
{
    //Check if costs are equal to calcualted costs
    auto costs_before = outputSolution.Costs;
    outputSolution.DetermineCosts(mInstance);
    auto epsilon = 1e-9;

    if(fabs(costs_before - outputSolution.Costs) >= epsilon)
    {
        throw std::runtime_error("Solution exits with costs of " + std::to_string(costs_before) +
                                 " but has real costs of " + std::to_string(outputSolution.Costs));
    }

    //outputSolution.NumberOfRoutes = outputSolution.Tours.size();
    for (size_t tourId = 0; tourId < outputSolution.Tours.size(); tourId++)
    {
        auto& tour = outputSolution.Tours[tourId];
        auto& route = tour.Route;
        auto& container = tour.Vehicle.Containers.front();
        Collections::IdVector stopIds;
        std::vector<CLP_Cuboid> selectedItems;
        auto totalWeight = 0.0;
        auto totalVolume = 0.0;

        for (size_t i = 0; i < route.size(); ++i)
        {
            auto& items = route[i].Items;
            totalWeight += route[i].TotalWeight;
            totalVolume += route[i].TotalVolume;
            stopIds.push_back(route[i].InternId);

            for (auto& item: items)
            {
                item.GroupId = route.size() - 1 - i;
                selectedItems.emplace_back(item);
            }
        }

        if (totalWeight > container.WeightLimit)
        {
            throw std::runtime_error("Route " + std::to_string(tourId) + tour.Print() + " with total weight "
                                     + std::to_string(totalWeight) + " exceeds weight limit "
                                     + std::to_string(container.WeightLimit));
        }

        if (totalVolume > container.Volume)
        {
            throw std::runtime_error("Route " + std::to_string(tourId) + tour.Print() + " with total volume "
                                     + std::to_string(totalVolume) + " exceeds volume limit "
                                     + std::to_string(container.Volume));
        }

        mLogFile << "Route " << std::to_string(tourId) + tour.Print()
                 << ": weight util " + std::to_string(totalWeight / container.WeightLimit)
                 << " | volume util " + std::to_string(totalVolume / container.Volume) << " | ";

        if (!mInputParameters.ContainerLoading.EnableThreeDimensionalLoading)
        {
            mLogFile << "\n";
            continue;
        }
        double maxRuntime = mInputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ILS);
        auto exactStatus = mLoadingChecker->ConstraintProgrammingSolverGetPacking(CLP_PackingType::Complete,
                                                                                    container,
                                                                                    stopIds,
                                                                                    selectedItems,
                                                                                    maxRuntime);

        std::string feasStatusCP = exactStatus == CLP_LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusCP << " with CP model"
                 << "\n";

        // TODO: packing as return value of loading checker

        if (exactStatus == CLP_LoadingStatus::Infeasible)
        {
            throw std::runtime_error("Loading infeasible according to CP model.");
        }

        size_t cItems = 0;
        for (auto& stop: route)
        {
            for (auto& item: stop.Items)
            {
                item = selectedItems[cItems];
                cItems++;
            }
        }
    }
}

void IteratedLocalSearch::PrintSolution(const OutputSolution& outputSolution)
{
    for (size_t tourId = 0; tourId < outputSolution.Tours.size(); tourId++)
    {
        const Tour& tour = outputSolution.Tours[tourId];
        const std::vector<Node>& route = tour.Route;

        mLogFile << "0 -> ";
        for (const auto& node: route)
        {
            mLogFile << node.InternId << " -> ";
        }
        mLogFile << "0"
                 << "\n";
    }
}

void IteratedLocalSearch::WriteSolutionSolutionValidator(const OutputSolution& outputSolution)
{
    ResultWriter::SolutionValidator::WriteInput(mOutputPath,
                                                mInstance->Name,
                                                InterfaceConversions::NodesToGroup(mInstance->Nodes),
                                                mInstance->Vehicles[0].Containers[0],
                                                mInstance->Vehicles.size());

    std::vector<std::vector<CLP_Group>> routes;
    routes.reserve(outputSolution.Tours.size());
    for (const auto& tour: outputSolution.Tours)
    {
        routes.emplace_back(InterfaceConversions::NodesToGroup(tour.Route));
    }

    ResultWriter::SolutionValidator::WriteOutput(mOutputPath, mInstance->Name, routes, outputSolution.Costs);
}

}
}