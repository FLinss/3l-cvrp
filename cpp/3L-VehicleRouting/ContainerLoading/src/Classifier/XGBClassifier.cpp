#include "Classifier/XGBClassifier.h"
#include <iostream>

namespace ContainerLoading{

std::string XGBClassifier::to_plain_path(const fs::path& p) {
    std::string s = fs::absolute(p).string();
    for (char& c : s) if (c == '\\') c = '/'; // normalize
    return s; // e.g. C:/Users/.../model.json
}

//Dummy function just for overide 
void XGBClassifier::loadStandardScalingFromJson(const fs::path& scaler_path){}

void XGBClassifier::loadModelfromPath(const fs::path& model_path)
{
    if (!fs::exists(model_path)){
        throw std::runtime_error("Model path does not exist!.");
    }

    // create booster_ handle first
    int err = XGBoosterCreate(nullptr, 0, &booster_);
    if (err != 0 || booster_ == nullptr) {
        std::cerr << "[XGBoost] Create failed: " << XGBGetLastError() << std::endl;
        throw std::runtime_error("XGBoosterCreate failed");
    }

    // Optional but recommended
    XGBoosterSetParam(booster_, "predictor", "cpu_predictor");
    XGBoosterSetParam(booster_, "nthread", "4");
    XGBoosterSetParam(booster_, "seed", "0");

    const std::string plain = to_plain_path(model_path);
    // Optional: quick sanity check using std::ifstream
    {
        std::ifstream test(plain, std::ios::binary);
        if (!test) throw std::runtime_error("Std ifstream cannot open: " + plain);
    }

    err = XGBoosterLoadModel(booster_, plain.c_str());
    if (err != 0) {
        std::cerr << "[XGBoost] LoadModel failed: " << XGBGetLastError() << std::endl;
        throw std::runtime_error("XGBoosterLoadModel failed");
    }
}

XGBClassifier::XGBClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams),  booster_(nullptr)
{

    fs::path dir (containerLoadingParams.BaseModelPath);
    fs::path model_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_model.json");
    fs::path  model_path = dir / model_file;

    loadModelfromPath(model_path);
}


std::array<float, 48> XGBClassifier::extractFeatures(const std::vector<Model::Cuboid>& items,
                                                    const Collections::IdVector& route,
                                                    const Model::Container& container) const
{
    std::array<float, 48> r{}; // zero-initialized

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
    std::unordered_map<int,int>   items_per_cust;
    std::unordered_map<int,int>   fragile_per_cust;
    std::unordered_map<int,float> volume_per_cust, weight_per_cust;

    for (size_t it=0; it<items.size(); ++it) {
        const auto& item = items[it];
        tot_volume += item.Volume;
        tot_weight += item.Weight;
        tot_length += item.Dx;
        tot_width  += item.Dy;
        tot_height += item.Dz;

        // guard denominators
        const float Dx = item.Dx;
        const float Dy = item.Dy;
        const float Dz = item.Dz;
        const float Vol = item.Volume;
        const float Ar  = item.Area;

        width_height_ratios[it] = (Dz != 0.f) ? (Dy / Dz) : 0.f;
        length_height_ratios[it]= (Dz != 0.f) ? (Dx / Dz) : 0.f;
        width_length_ratios[it] = (Dx != 0.f) ? (Dy / Dx) : 0.f;
        length_L_ratios[it]     = (containerDx != 0.f) ? (Dx / containerDx) : 0.f;
        width_W_ratios[it]      = (containerDy != 0.f) ? (Dy / containerDy) : 0.f;
        height_H_ratios[it]     = (containerDz != 0.f) ? (Dz / containerDz) : 0.f;
        volume_WLH_ratios[it]   = (containerVolume != 0.f) ? (Vol / containerVolume) : 0.f;
        height_area_ratios[it]  = (Ar != 0.f) ? (Dz / Ar) : 0.f;
        area_AREA_ratios[it]    = (containerArea != 0.f) ? (Ar / containerArea) : 0.f;

        const int gid = item.GroupId;
        items_per_cust[gid]  += 1;
        volume_per_cust[gid] += Vol;
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

// Save a 1D or 2D tensor as CSV with timestamp
void XGBClassifier::save_features_to_csv(const std::array<float, 48> feats,
                                              const int status,
                                              const float output) const{

    std::string filename = mSaveTensorPath + "/tensor_" + get_timestamp() + ".csv";

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n";
        return;
    }
    // First column: status plus outpur
    file << status << "," << output << ",";

    for (const auto& feat : feats) {
        file << feat << ",";
    }
    file << "\n";

    file.close();
}



void XGBClassifier::saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                                const Collections::IdVector& route,
                                                const Model::Container& container,
                                                float output,
                                                int status) const {
    
    // 1) build the flat row of 48 features
    std::array<float, 48> feats = extractFeatures(items, route, container);   
    
    save_features_to_csv(feats, status, output);
}

float XGBClassifier::classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                                            const Collections::IdVector& route,
                                            const Model::Container& container)
{
    // 1) build the flat row of 48 features
    std::array<float, 48> feats = extractFeatures(items, route, container);

    // 2) create DMatrix: nrow=1, ncol=48  (row-major)
    DMatrixHandle dmat = nullptr;
    // missing = -1 (or use NAN if your training used that)

    int err = XGDMatrixCreateFromMat(feats.data(), /*nrow*/1, /*ncol*/48, /*missing*/-1.0f, &dmat);
    if (err != 0 || dmat == nullptr) {
        // handle error appropriately in your codebase
        return std::numeric_limits<float>::quiet_NaN();
    }

    // 3) predict
    bst_ulong out_len = 0;
    const float* out = nullptr;
    err = XGBoosterPredict(booster_, dmat, /*option_mask*/0, /*ntree_limit*/0, 0 /*training*/, &out_len, &out);

    XGDMatrixFree(dmat);

    if (err != 0 || out == nullptr || out_len == 0) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    // typical binary/prob model -> first score

    return out[0];
}     

bool XGBClassifier::classify(const std::vector<Model::Cuboid>& items,
                           const Collections::IdVector& route,
                           const Model::Container& container){

    return classifyReturnOutput(items,route,container) > mAcceptanceThreshold;
}

}