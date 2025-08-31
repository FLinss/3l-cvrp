#include "Classifier/XGBClassifier.h"

namespace ContainerLoading{

void XGBClassifier::loadStandardScalingFromJson(const std::string& scaler_path){

    return;

}

XGBClassifier::XGBClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams)
{
    loadStandardScalingFromJson(containerLoadingParams.ModelValuesJson);
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


    return false;
}

}