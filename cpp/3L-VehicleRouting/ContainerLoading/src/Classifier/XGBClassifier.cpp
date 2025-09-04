#include "Classifier/XGBClassifier.h"

namespace ContainerLoading{

void XGBClassifier::loadStandardScalingFromJson(const fs::path& scaler_path){

    return;

}

void XGBClassifier::loadModelfromPath(const fs::path& model_path)
{
    std::ifstream file(model_path);
    if (!file) {
        throw std::runtime_error("Could not open model JSON file.");
    }

    return;

}

XGBClassifier::XGBClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams)
{

    fs::path dir (containerLoadingParams.BaseModelPath);
    fs::path model_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_model.pt");
    fs::path  model_path = dir / model_file;

    loadModelfromPath(model_path);
}




void XGBClassifier::saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                                const Collections::IdVector& route,
                                                const Model::Container& container,
                                                float output,
                                                int status) const {
    
    return;                                                
}

float XGBClassifier::classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                            const Collections::IdVector& route,
                            const Model::Container& container){

    return 0.5f;
}       

bool XGBClassifier::classify(const std::vector<Model::Cuboid>& items,
                           const Collections::IdVector& route,
                           const Model::Container& container){

    BoosterHandle booster{nullptr};
    return false;
}

}