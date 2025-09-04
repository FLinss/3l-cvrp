#include "Classifier/FFNNClassifier.h"

namespace ContainerLoading{

void FFNNClassifier::loadStandardScalingFromJson(const fs::path& scaler_path){

    std::ifstream file(scaler_path);
    if (!file) {
        throw std::runtime_error("Could not open scaling JSON file.");
    }

    nlohmann::json j;
    file >> j;

    std::vector<double> mean_vec = j["mean"];
    std::vector<double> std_vec = j["std"];

    mean_tensor = torch::tensor(mean_vec, torch::kFloat32).unsqueeze(0); // shape: [1, N]
    std_tensor = torch::tensor(std_vec, torch::kFloat32).unsqueeze(0);   // shape: s[1, N]

     // Optional: load a probability threshold saved from training (fallback 0.5)
    float prob_thr = j.contains("threshold") ? j["threshold"].get<float>() : 0.5f;
    mAcceptanceThreshold = prob_thr;

    // Precompute logit(threshold) so we can compare logits directly
    mLogitThreshold = std::log(prob_thr / (1.0f - prob_thr)); // logit
}

void FFNNClassifier::loadModelfromPath(const fs::path& model_path) {
    if (!fs::exists(model_path)) {
        throw std::runtime_error("Model file does not exist: " + model_path.string());
    }

    model = torch::jit::load(model_path.string());

    model.eval();
}

FFNNClassifier::FFNNClassifier(const ContainerLoadingParams& containerLoadingParams) : 
    BaseClassifier(containerLoadingParams)
{
    fs::path dir (containerLoadingParams.BaseModelPath);
    fs::path model_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_model.pt");
    fs::path scaler_file (modelTypeString + "_" + containerLoadingParams.ModelDataSet + "_scaler.json");
    fs::path  model_path = dir / model_file;
    fs::path  scaler_path = dir / scaler_file;
    
    loadModelfromPath(model_path);
    loadStandardScalingFromJson(scaler_path);
}

torch::Tensor FFNNClassifier::applyStandardScaling(const torch::Tensor& input) const{

    return (input - mean_tensor) / std_tensor;
}

torch::Tensor FFNNClassifier::extractFeatures(const std::vector<Model::Cuboid>& items,
                                          const Collections::IdVector& route,
                                          const Model::Container& container) const {

    torch::Tensor result = torch::zeros({1,48});

    const float containerWeightLimit = container.WeightLimit;
    const float containerVolume = container.Volume;
    const float containerDx = container.Dx, containerDy = container.Dy, containerDz = container.Dz;
    const float containerArea = container.Area;

    const int64_t noItems = static_cast<int64_t>(items.size());
    const int64_t noCustomers = static_cast<int64_t>(route.size());


    std::vector<float> width_height_ratios(noItems), length_height_ratios(noItems),
                       width_length_ratios(noItems), length_L_ratios(noItems),
                       width_W_ratios(noItems),  height_H_ratios(noItems),
                       volume_WLH_ratios(noItems), height_area_ratios(noItems),
                       area_AREA_ratios(noItems);

    float tot_volume=0.f, tot_weight=0.f, tot_length=0.f, tot_width=0.f, tot_height=0.f;
    float fragile_count_total=0.f;

    // Precompute per-customer aggregates (counts, fragiles, vol, weight)
    // and per-item ratios for mins/maxs/etc.
    // Build a small index from GroupId -> vector of indices (or counts) for speed.
    std::unordered_map<int,int> items_per_cust, fragile_per_cust;
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

        auto gid = item.GroupId;
        items_per_cust[gid] += 1;
        volume_per_cust[gid] += item.Volume;
        weight_per_cust[gid] += item.Weight;
        if (item.Fragility == Model::Fragility::Fragile) {
            fragile_per_cust[gid] += 1;
            fragile_count_total += 1.f;
        }
    }
    // Positional pyramid weighting by route order: len(route), len(route)-1, ..., 1
    float fragile_sequence = 0.f;
    float volumeDistribution = 0.f;
    float weightDistribution = 0.f;

    for (size_t pos = 0; pos < route.size(); ++pos) {
        int gid = route[pos];
        float pos_weight = static_cast<float>(route.size() - pos); // e.g., 5,4,3,2,1

        int n = items_per_cust[gid];
        int f = fragile_per_cust[gid];
        float fragile_ratio = (n > 0) ? (static_cast<float>(f) / n) : 0.f;
        fragile_sequence += pos_weight * fragile_ratio;

        // If training used positional weights for these too, do the same:
        weightDistribution += weight_per_cust[gid] * pos_weight;
        volumeDistribution += volume_per_cust[gid] * pos_weight;
    }

    result[0][0] = noItems;
    //No Customers
    result[0][1] = noCustomers;

    //'Rel Volume' and 'Rel Weight'
    result[0][2] = tot_volume / containerVolume;
    result[0][3] = tot_weight / containerWeightLimit;

    //'Weight Distribution', 'Volume Distribution'
    result[0][4] = weightDistribution / containerWeightLimit;
    result[0][5] = volumeDistribution / containerVolume;

    //'Volume Balance',
    result[0][6] = volumeDistribution / tot_volume;
    //'Fragile Ratio'
    result[0][7] = (noItems > 0) ? (fragile_count_total / noItems) : 0.f;        // Fragile Ratio

    //'Fragile Sequence'
    result[0][8] = fragile_sequence;

    //Rel Total Length Items', 'Rel Total Width Items', 'Rel Total Height Items', 
    result[0][9] = tot_length / containerDx;
    result[0][10] = tot_width / containerDy;
    result[0][11] = tot_height / containerDz;

    // 'width_height_min', 'width_height_max', 'width_height_mean', 'width_height_std',
    result[0][12] = *std::min_element(width_height_ratios.begin(), width_height_ratios.end());
    result[0][13] = *std::max_element(width_height_ratios.begin(), width_height_ratios.end());
    result[0][14] = getMean(width_height_ratios.begin(), width_height_ratios.end());
    result[0][15] = getStd(width_height_ratios.begin(), width_height_ratios.end());
    
    //'length_height_min', 'length_height_max', 'length_height_mean', 'length_height_std',
    result[0][16] = *std::min_element(length_height_ratios.begin(), length_height_ratios.end());
    result[0][17] = *std::max_element(length_height_ratios.begin(), length_height_ratios.end());
    result[0][18] = getMean(length_height_ratios.begin(), length_height_ratios.end());
    result[0][19] = getStd(length_height_ratios.begin(), length_height_ratios.end());

    // 'width_length_min', 'width_length_max', 'width_length_mean', 'width_length_std',
    result[0][20] = *std::min_element(width_length_ratios.begin(), width_length_ratios.end());
    result[0][21] = *std::max_element(width_length_ratios.begin(), width_length_ratios.end());
    result[0][22] = getMean(width_length_ratios.begin(), width_length_ratios.end());
    result[0][23] = getStd(width_length_ratios.begin(), width_length_ratios.end());

    // 'width_W_min', 'width_W_max', 'width_W_mean', 'width_W_std', 
    result[0][24] = *std::min_element(width_W_ratios.begin(), width_W_ratios.end());
    result[0][25] = *std::max_element(width_W_ratios.begin(), width_W_ratios.end());
    result[0][26] = getMean(width_W_ratios.begin(), width_W_ratios.end());
    result[0][27] = getStd(width_W_ratios.begin(), width_W_ratios.end());

    //'length_L_min', 'length_L_max', 'length_L_mean', 'length_L_std',
    result[0][28] = *std::min_element(length_L_ratios.begin(), length_L_ratios.end());
    result[0][29] = *std::max_element(length_L_ratios.begin(), length_L_ratios.end());
    result[0][30] = getMean(length_L_ratios.begin(), length_L_ratios.end());
    result[0][31] = getStd(length_L_ratios.begin(), length_L_ratios.end());

    //'height_H_min', 'height_H_max', 'height_H_mean', 'height_H_std'
    result[0][32] = *std::min_element(height_H_ratios.begin(), height_H_ratios.end());
    result[0][33] = *std::max_element(height_H_ratios.begin(), height_H_ratios.end());
    result[0][34] = getMean(height_H_ratios.begin(), height_H_ratios.end());
    result[0][35] = getStd(height_H_ratios.begin(), height_H_ratios.end());

    //'volume_WLH_min', 'volume_WLH_max', 'volume_WLH_mean', 'volume_WLH_std'
    result[0][36] = *std::min_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][37] = *std::max_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][38] = getMean(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][39] = getStd(volume_WLH_ratios.begin(), volume_WLH_ratios.end());


    result[0][40] = *std::min_element(height_area_ratios.begin(), height_area_ratios.end());
    result[0][41] = *std::max_element(height_area_ratios.begin(), height_area_ratios.end());
    result[0][42] = getMean(height_area_ratios.begin(), height_area_ratios.end());
    result[0][43] = getStd(height_area_ratios.begin(), height_area_ratios.end());


    result[0][44] = *std::min_element(area_AREA_ratios.begin(), area_AREA_ratios.end());
    result[0][45] = *std::max_element(area_AREA_ratios.begin(), area_AREA_ratios.end());
    result[0][46] = getMean(area_AREA_ratios.begin(), area_AREA_ratios.end());
    result[0][47] = getStd(area_AREA_ratios.begin(), area_AREA_ratios.end());

    // Resize or pad to match model input if needed
    return result;
}


// Save a 1D or 2D tensor as CSV with timestamp
void FFNNClassifier::save_tensor_to_csv(const torch::Tensor& tensor, const int status, const float output) const{
    torch::Tensor cpu_tensor = tensor.detach().cpu();
    std::string filename = mSaveTensorPath + "/tensor_" + get_timestamp() + ".csv";

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n";
        return;
    }

    torch::Tensor tensor_2d = cpu_tensor;
    if (tensor_2d.dim() == 1) {
        tensor_2d = tensor_2d.unsqueeze(0);  // make it 2D: [1, N]
    }

    auto accessor = tensor_2d.accessor<float, 2>();
    for (int i = 0; i < tensor_2d.size(0); ++i) {
        // First column: status
        file << status << "," << output << ",";

        for (int j = 0; j < tensor_2d.size(1); ++j) {
            file << accessor[i][j];
            if (j < tensor_2d.size(1) - 1)
                file << ",";
        }
        file << "\n";
    }

    file.close();
}


void FFNNClassifier::saveClassifierResults(const std::vector<Model::Cuboid>& items,
                                                const Collections::IdVector& route,
                                                const Model::Container& container,
                                                float output,
                                                int status) const {

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor input_scaled = applyStandardScaling(input);

    save_tensor_to_csv(input_scaled, status, output);
}

float FFNNClassifier::classifyReturnOutput(const std::vector<Model::Cuboid>& items,
                            const Collections::IdVector& route,
                            const Model::Container& container){

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor input_scaled = applyStandardScaling(input);
    auto logits = model.forward({input_scaled}).toTensor().squeeze();
                
    // If you want a *probability* to return/log, apply Sigmoid here:
    float prob = 1.0f / (1.0f + std::exp(-logits.item<float>()));
    return prob; // or return logits.item<float>() if you prefer raw logit
}

bool FFNNClassifier::classify(const std::vector<Model::Cuboid>& items,
                           const Collections::IdVector& route,
                           const Model::Container& container){

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor input_scaled = applyStandardScaling(input);
    auto logits = model.forward({input_scaled}).toTensor().squeeze();

    // Compare *logit* to *logit threshold* (no Sigmoid needed)
    return logits.item<float>() >= mLogitThreshold;
}

}