#include "Classifier/LRClassifier.h"
namespace ContainerLoading{

void LRClassifier::loadStandardScalingFromJson(const fs::path& scaler_path) {

    std::ifstream file(scaler_path);
    if (!file) throw std::runtime_error("Could not open scaling JSON file.");

    nlohmann::json j; file >> j;

    // Accept float or double in JSON; store as float
    std::vector<float> mean_vec, std_vec;
    if (j.at("mean").is_array() && !j.at("mean").empty() && j.at("mean")[0].is_number_float()) {
        mean_vec = j.at("mean").get<std::vector<float>>();
    } else {
        std::vector<double> tmp = j.at("mean").get<std::vector<double>>();
        mean_vec.assign(tmp.begin(), tmp.end());
    }
    if (j.at("std").is_array() && !j.at("std").empty() && j.at("std")[0].is_number_float()) {
        std_vec = j.at("std").get<std::vector<float>>();
    } else {
        std::vector<double> tmp = j.at("std").get<std::vector<double>>();
        std_vec.assign(tmp.begin(), tmp.end());
    }

    const int N = static_cast<int>(mean_vec.size());
    mean_.resize(N);
    std_.resize(N);
    for (int i = 0; i < N; ++i) {
        mean_[i] = mean_vec[i];
        std_[i]  = std::max(std_vec[i], 1e-6f); // avoid div by zero
    }
}

void LRClassifier::loadModelfromPath(const fs::path& model_path)
{
    std::ifstream file(model_path);
    if (!file) {
        throw std::runtime_error("Could not open model JSON file.");
    }

    nlohmann::json j; file >> j;

    std::vector<float> wv;
    if (j.at("w").is_array() && !j.at("w").empty() && j.at("w")[0].is_number_float()) {
        wv = j.at("w").get<std::vector<float>>();
    } else {
        std::vector<double> tmp = j.at("w").get<std::vector<double>>();
        wv.assign(tmp.begin(), tmp.end());
    }
    b_ = j.at("b").get<float>();

    const int N = static_cast<int>(wv.size());
    w_.resize(N);
    for (int i = 0; i < N; ++i) w_[i] = wv[i];


}

LRClassifier::LRClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams)
{
    fs::path dir (containerLoadingParams.BaseModelPath);
    fs::path model_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_model.pt");
    fs::path scaler_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_scaler.json");
    fs::path  model_path = dir / model_file;
    fs::path  scaler_path = dir / scaler_file;

    loadModelfromPath(model_path);

    // Basic sanity with scaling size
    if (mean_.size() != 0 && mean_.size() != w_.size()) {
        throw std::runtime_error("LR weight size does not match scaler size.");
    }

    loadStandardScalingFromJson(scaler_path);
}

float LRClassifier::classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                                         const Collections::IdVector& route,
                                         const Model::Container& container)
{
    Eigen::VectorXf x  = extractFeatures(items, route, container); // [N]
    Eigen::VectorXf xs = applyStandardScaling(x);                  // [N]
    const float z = w_.dot(xs) + b_;
    return sigmoid_stable(z);
}

bool LRClassifier::classify(const std::vector<Model::Cuboid>& items,
                            const Collections::IdVector& route,
                            const Model::Container& container)
{
    return classifyReturnOutput(items, route, container) > mAcceptanceThreshold;
}

void LRClassifier::saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                         const Collections::IdVector& route,
                                         const Model::Container& container,
                                         float output,
                                         int status) const
{
    Eigen::VectorXf x  = extractFeatures(items, route, container);
    Eigen::VectorXf xs = applyStandardScaling(x);
    save_vector_to_csv(xs, status, output);
}

// ----------------- Scaling -----------------

Eigen::VectorXf LRClassifier::applyStandardScaling(const Eigen::VectorXf& x) const {
    if (mean_.size() == 0 || std_.size() == 0) return x; // no-op if not loaded
    Eigen::VectorXf y = x;
    // elementwise (x - mean) / std
    y.array() -= mean_.array();
    y.array() /= std_.array();
    return y;
}


Eigen::VectorXf LRClassifier::extractFeatures(const std::vector<Model::Cuboid>& items,
                                              const Collections::IdVector& route,
                                              const Model::Container& container) const
{
    const int N = 46;
    Eigen::VectorXf r = Eigen::VectorXf::Zero(N); // [N]
    // Mirror your FFNN feature logic, but write into r[i] instead of tensor[0][i].

    const float containerWeightLimit = container.WeightLimit;
    const float containerVolume      = container.Volume;
    const float noItems              = static_cast<float>(items.size());
    const float noCustomers          = static_cast<float>(route.size());
    const float containerDx          = container.Dx;
    const float containerDy          = container.Dy;
    const float containerDz          = container.Dz;
    const float containerArea        = container.Area;

    std::vector<int> pyramideValues(route.size());
    std::iota(pyramideValues.begin(), pyramideValues.end(), 1);

    std::vector<float> width_height_ratios(items.size(), 0.0f);
    std::vector<float> length_height_ratios(items.size(), 0.0f);
    std::vector<float> width_length_ratios(items.size(), 0.0f);
    std::vector<float> length_L_ratios(items.size(), 0.0f);
    std::vector<float> width_W_ratios(items.size(), 0.0f);
    std::vector<float> height_H_ratios(items.size(), 0.0f);
    std::vector<float> volume_WLH_ratios(items.size(), 0.0f);
    std::vector<float> height_area_ratios(items.size(), 0.0f);
    std::vector<float> area_AREA_ratios(items.size(), 0.0f);

    float tot_volume = 0.0f;
    float tot_weight = 0.0f;
    float fragile_count = 0.0f;
    float tot_length = 0.0f;
    float tot_width  = 0.0f;
    float tot_height = 0.0f;
    float volumeDistribution = 0.0f;
    float weightDistribution = 0.0f;

    int it = 0;
    for (const auto& item : items) {
        tot_volume += item.Volume;
        tot_weight += item.Weight;
        tot_width  += item.Dy;
        tot_length += item.Dx;
        tot_height += item.Dz;
        if (item.Fragility == Model::Fragility::Fragile) ++fragile_count;

        // NOTE: assumes item.GroupId is a valid index for route/pyramideValues
        weightDistribution += item.Weight * pyramideValues[item.GroupId];
        volumeDistribution += item.Volume * pyramideValues[item.GroupId];

        width_height_ratios[it]  = static_cast<float>(item.Dy) / item.Dz;
        length_height_ratios[it] = static_cast<float>(item.Dx) / item.Dz;
        width_length_ratios[it]  = static_cast<float>(item.Dy) / item.Dx;
        length_L_ratios[it]      = item.Dx / containerDx;
        width_W_ratios[it]       = item.Dy / containerDy;
        height_H_ratios[it]      = item.Dz / containerDz;
        volume_WLH_ratios[it]    = item.Volume / containerVolume;
        height_area_ratios[it]   = item.Dz / item.Area;
        area_AREA_ratios[it]     = item.Area / containerArea;
        ++it;
    }

    // 0..9
    r[0] = noItems;
    r[1] = noCustomers;
    r[2] = tot_volume / containerVolume;
    r[3] = tot_weight / containerWeightLimit;
    r[4] = weightDistribution / containerWeightLimit;
    r[5] = volumeDistribution / containerVolume;
    r[6] = fragile_count / noItems;
    r[7] = tot_length / containerDx;
    r[8] = tot_width  / containerDy;
    r[9] = tot_height / containerDz;

    // 10..45 — stats via your base helpers (assumed protected/public)
    auto set_stats = [&](int base,
                         const std::vector<float>& v) {
        if (v.empty()) {
            r[base+0] = r[base+1] = r[base+2] = r[base+3] = 0.0f;
            return;
        }
        r[base+0] = *std::min_element(v.begin(), v.end());
        r[base+1] = *std::max_element(v.begin(), v.end());
        r[base+2] = BaseClassifier::getMean(v.begin(), v.end());
        r[base+3] = BaseClassifier::getStd(v.begin(), v.end());
    };

    set_stats(10, width_height_ratios);
    set_stats(14, length_height_ratios);
    set_stats(18, width_length_ratios);
    set_stats(22, width_W_ratios);
    set_stats(26, length_L_ratios);
    set_stats(30, height_H_ratios);
    set_stats(34, volume_WLH_ratios);
    set_stats(38, height_area_ratios);
    set_stats(42, area_AREA_ratios);

    return r;
}

// ----------------- CSV save -----------------

void LRClassifier::save_vector_to_csv(const Eigen::VectorXf& row,
                                      int status,
                                      float output) const
{
    std::string filename = mSaveTensorPath + "/tensor_" + get_timestamp() + ".csv";
    std::ofstream file(filename);
    if (!file) return;

    file << status << "," << output << ",";
    for (int j = 0; j < row.size(); ++j) {
        file << row[j];
        if (j + 1 < row.size()) file << ",";
    }
    file << "\n";
}


}