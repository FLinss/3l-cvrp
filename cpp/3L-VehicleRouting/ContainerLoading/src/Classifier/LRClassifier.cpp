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

    // Optional: load a probability threshold saved from training (fallback 0.5)
    float prob_thr = j.contains("threshold") ? j["threshold"].get<float>() : 0.5f;
    mAcceptanceThreshold = prob_thr;
}

void LRClassifier::loadModelfromPath(const fs::path& model_path)
{
    std::ifstream file(model_path);
    if (!file) {
        throw std::runtime_error("Could not open model JSON file.");
    }

    nlohmann::json j; file >> j;

    std::vector<float> wv;
    if (j.at("coef").is_array() && !j.at("coef").empty() && j.at("coef")[0].is_number_float()) {
        wv = j.at("coef").get<std::vector<float>>();
    } else {
        std::vector<double> tmp = j.at("coef").get<std::vector<double>>();
        wv.assign(tmp.begin(), tmp.end());
    }
    b_ = j.at("intercept").get<float>();

    const int N = static_cast<int>(wv.size());
    w_.resize(N);
    for (int i = 0; i < N; ++i) w_[i] = wv[i];


}

LRClassifier::LRClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams)
{
    fs::path dir (containerLoadingParams.BaseModelPath);
    fs::path model_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_model.json");
    fs::path scaler_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_scaler.json");
    fs::path  model_path = dir / model_file;
    fs::path  scaler_path = dir / scaler_file;

    loadModelfromPath(model_path);
    loadStandardScalingFromJson(scaler_path);

    // Basic sanity with scaling size
    if (mean_.size() != 0 && mean_.size() != w_.size()) {
        throw std::runtime_error("LR weight size does not match scaler size.");
    }
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
    const int N = 48;
    Eigen::VectorXf r = Eigen::VectorXf::Zero(N);

    const float containerWeightLimit = container.WeightLimit;
    const float containerVolume      = container.Volume;
    const float containerDx          = container.Dx;
    const float containerDy          = container.Dy;
    const float containerDz          = container.Dz;
    const float containerArea        = container.Area;

    const int64_t noItems     = static_cast<int64_t>(items.size());
    const int64_t noCustomers = static_cast<int64_t>(route.size());

    // per-item ratio arrays for the stats blocks
    std::vector<float> width_height_ratios(noItems), length_height_ratios(noItems),
                       width_length_ratios(noItems), length_L_ratios(noItems),
                       width_W_ratios(noItems),  height_H_ratios(noItems),
                       volume_WLH_ratios(noItems), height_area_ratios(noItems),
                       area_AREA_ratios(noItems);

    float tot_volume=0.f, tot_weight=0.f, tot_length=0.f, tot_width=0.f, tot_height=0.f;
    float fragile_count_total=0.f;

    // aggregate by customer (GroupId)
    std::unordered_map<int,int>   items_per_cust, fragile_per_cust;
    std::unordered_map<int,float> volume_per_cust, weight_per_cust;

    for (size_t it=0; it<items.size(); ++it) {
        const auto& item = items[it];
        tot_volume += item.Volume;
        tot_weight += item.Weight;
        tot_length += item.Dx;
        tot_width  += item.Dy;
        tot_height += item.Dz;

        width_height_ratios[it] = static_cast<float>(item.Dy) / item.Dz;
        length_height_ratios[it]= static_cast<float>(item.Dx) / item.Dz;
        width_length_ratios[it] = static_cast<float>(item.Dy) / item.Dx;
        length_L_ratios[it]     = item.Dx / containerDx;
        width_W_ratios[it]      = item.Dy / containerDy;
        height_H_ratios[it]     = item.Dz / containerDz;
        volume_WLH_ratios[it]   = item.Volume / containerVolume;
        height_area_ratios[it]  = item.Dz / item.Area;
        area_AREA_ratios[it]    = item.Area / containerArea;

        const int gid = item.GroupId;
        items_per_cust[gid]  += 1;
        volume_per_cust[gid] += item.Volume;
        weight_per_cust[gid] += item.Weight;
        if (item.Fragility == Model::Fragility::Fragile) {
            fragile_per_cust[gid] += 1;
            fragile_count_total   += 1.f;
        }
    }

    // Positional pyramid weighting by route order: len(route), len(route)-1, ..., 1
    float fragile_sequence   = 0.f;
    float volumeDistribution = 0.f;
    float weightDistribution = 0.f;

    for (size_t pos = 0; pos < route.size(); ++pos) {
        const int gid = route[pos];
        const float pos_weight = static_cast<float>(route.size() - pos); // e.g. 5,4,3,2,1

        const int n = items_per_cust[gid];
        const int f = fragile_per_cust[gid];
        const float fragile_ratio = (n > 0) ? (static_cast<float>(f) / n) : 0.f;
        fragile_sequence   += pos_weight * fragile_ratio;
        weightDistribution += weight_per_cust[gid] * pos_weight;
        volumeDistribution += volume_per_cust[gid] * pos_weight;
    }

    // ===== write features in EXACT FFNN order =====
    r[0] = static_cast<float>(noItems);      // NoItems
    r[1] = static_cast<float>(noCustomers);  // NoCustomers
    r[2] = (containerVolume > 0.f)      ? (tot_volume / containerVolume)           : 0.f; // Rel Volume
    r[3] = (containerWeightLimit > 0.f) ? (tot_weight / containerWeightLimit)      : 0.f; // Rel Weight
    r[4] = (containerWeightLimit > 0.f) ? (weightDistribution / containerWeightLimit) : 0.f; // Weight Distribution
    r[5] = (containerVolume > 0.f)      ? (volumeDistribution / containerVolume)   : 0.f;   // Volume Distribution
    r[6] = (tot_volume > 0.f) ? (volumeDistribution / tot_volume) : 0.f;                      // Volume Balance
    r[7] = (noItems > 0) ? (fragile_count_total / static_cast<float>(noItems)) : 0.f;        // Fragile Ratio
    r[8] = fragile_sequence;                                                                // Fragile Sequence

    // Rel Total Length/Width/Height (note indices 9..11)
    r[9]  = containerDx > 0.f ? (tot_length / containerDx) : 0.f;
    r[10] = containerDy > 0.f ? (tot_width  / containerDy) : 0.f;
    r[11] = containerDz > 0.f ? (tot_height / containerDz) : 0.f;

    // helper for stats blocks
    auto set_stats = [&](int base, const std::vector<float>& v) {
        if (v.empty()) {
            r[base+0] = r[base+1] = r[base+2] = r[base+3] = 0.f;
            return;
        }
        r[base+0] = *std::min_element(v.begin(), v.end());
        r[base+1] = *std::max_element(v.begin(), v.end());
        r[base+2] = BaseClassifier::getMean(v.begin(), v.end());
        r[base+3] = BaseClassifier::getStd (v.begin(), v.end());
    };

    // 9 blocks × 4 = 36 features → indices 12..47
    set_stats(12, width_height_ratios);
    set_stats(16, length_height_ratios);
    set_stats(20, width_length_ratios);
    set_stats(24, width_W_ratios);
    set_stats(28, length_L_ratios);
    set_stats(32, height_H_ratios);
    set_stats(36, volume_WLH_ratios);
    set_stats(40, height_area_ratios);
    set_stats(44, area_AREA_ratios);

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