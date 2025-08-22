#include "Classifier/LRClassifier.h"


namespace ContainerLoading{

void LRClassifier::loadStandardScalingFromJson(const std::string& scaler_path){

    return;

}

LRClassifier::LRClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams)
{
    loadStandardScalingFromJson(containerLoadingParams.ModelValuesJson);

}



void LRClassifier::saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                                const Collections::IdVector& route,
                                                const Model::Container& container,
                                                const float output,
                                                const int status) const{

   return;
}

float LRClassifier::classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                            const Collections::IdVector& route,
                            const Model::Container& container){

    return 0.5f;
}

bool LRClassifier::classify(const std::vector<Model::Cuboid>& items,
                           const Collections::IdVector& route,
                           const Model::Container& container) {

    return false;
}

}