// json_bitset_map_serializers.hpp
#pragma once
#include <boost/dynamic_bitset.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <type_traits> // for std::underlying_type_t
#include <unordered_map>
#include <vector>

namespace ContainerLoading
{
namespace Algorithms
{
enum class LoadingFlag;
}
}

// Put serializers into the global ::nlohmann namespace so they are always found.
namespace nlohmann
{

template <class Block, class Alloc> inline void to_json(json& j, const boost::dynamic_bitset<Block, Alloc>& bs)
{
    j = json::array();
    auto& a = j.get_ref<json::array_t&>();
    a.reserve(bs.count()); // we know how many 1-bits there are

    // Efficient scan: find_first / find_next (lowest index first)
    for (auto i = bs.find_first(); i != boost::dynamic_bitset<Block, Alloc>::npos; i = bs.find_next(i))
    {
        a.push_back(static_cast<std::uint64_t>(i));
    }

    // (If your Boost is ancient and lacks find_first/find_next,
    //  fall back to: for (std::size_t i=0;i<bs.size();++i) if (bs.test(i)) a.push_back(i);)
}

// 2) unordered_map<LoadingFlag, vector<dynamic_bitset<...>>, Hash, Eq, Alloc>
//    -> JSON object with *numeric* keys (as strings, because JSON keys are strings)
template <class Block, class BitAlloc, class Hash, class Eq, class MapAlloc>
inline void to_json(json& j,
                    const std::unordered_map<ContainerLoading::Algorithms::LoadingFlag,
                                             std::vector<boost::dynamic_bitset<Block, BitAlloc>>,
                                             Hash,
                                             Eq,
                                             MapAlloc>& map)
{
    using LF = ContainerLoading::Algorithms::LoadingFlag;
    using U = std::underlying_type_t<LF>;

    j = json::object();
    for (const auto& [flag, vec]: map)
    {
        const U code = static_cast<U>(flag); // numeric value (bitmask)
        j[std::to_string(code)] = vec; // key is "3", "12", ...
        // 'vec' uses the bitset to_json above for each element
    }
}

// ---------- 3) SequenceSet -> array of arrays ----------
inline void to_json(json& j, const Collections::SequenceSet& set)
{
    j = json::array();
    auto& arr = j.get_ref<json::array_t&>();
    arr.reserve(set.size());
    for (const auto& seq: set)
    {
        // assumes seq (IdVector) is itself json-serializable (e.g., vector<int>)
        arr.push_back(seq);
    }
}

// ---------- 4) map<LoadingFlag, SequenceSet> ----------
template <class Hash, class Eq, class MapAlloc>
inline void to_json(
    json& j,
    const std::unordered_map<ContainerLoading::Algorithms::LoadingFlag, Collections::SequenceSet, Hash, Eq, MapAlloc>&
        map)
{
    using LF = ContainerLoading::Algorithms::LoadingFlag;
    using U = std::underlying_type_t<LF>;
    j = json::object();
    for (const auto& [flag, seqSet]: map)
    {
        j[std::to_string(static_cast<U>(flag))] = seqSet; // numeric flag keys
    }
}

} // namespace nlohmann
