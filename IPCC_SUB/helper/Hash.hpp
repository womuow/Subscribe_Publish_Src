#ifndef _HASH_H_
#define _HASH_H_

#include <stdexcept>

namespace HBC_HELPER
{

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

} // end namespace HBC_HELPER

#endif