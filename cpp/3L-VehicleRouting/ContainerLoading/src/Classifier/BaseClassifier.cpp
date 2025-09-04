// File: BaseClassifier.cpp

#include "Classifier/BaseClassifier.h"

namespace ContainerLoading {

BaseClassifier::BaseClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    mAcceptanceThreshold(containerLoadingParams.AcceptanceThreshold),
    mSaveTensorPath(containerLoadingParams.TensorDataFilePath)
{
    switch (containerLoadingParams.ModelType){
        case ContainerLoadingParams::ModelTypes::FFNN:
            modelTypeString = "FFNN";
            break;
        case ContainerLoadingParams::ModelTypes::LR:
            modelTypeString = "LR";
            break;
        case ContainerLoadingParams::ModelTypes::XGBOOST:
            modelTypeString = "XGB";
            break;
      }
}

float BaseClassifier::getMean(std::vector<float>::const_iterator first,
                          std::vector<float>::const_iterator last)
{
    auto count = std::distance(first, last);
    if (count == 0) return 0.0f;

    float sum = std::accumulate(first, last, 0.0f);
    return sum / count;
}


float BaseClassifier::getStd(std::vector<float>::const_iterator first,
                         std::vector<float>::const_iterator last)
{
    auto count = std::distance(first, last);
    if (count <= 1) return 0.0f;

    float mean = getMean(first, last);

    float variance = 0.0f;
    for (auto it = first; it != last; ++it)
        variance += std::pow(*it - mean, 2);

    return std::sqrt(variance / count);  // or / (count - 1) for sample stddev
}

std::string BaseClassifier::get_timestamp() const{
    using namespace std::chrono;

    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::tm* parts = std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(parts, "%Y-%m-%d_%H-%M-%S");
    oss << "-" << std::setw(3) << std::setfill('0') << now_ms.count();  // add milliseconds

    return oss.str();
}



}  // namespace ContainerLoading
