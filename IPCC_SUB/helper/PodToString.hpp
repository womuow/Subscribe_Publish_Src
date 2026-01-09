#ifndef _POD_2_STRING_H_
#define _POD_2_STRING_H_

#include <string>
#include <cstring>
#include <vector>

namespace HBC_HELPER
{

template<typename T>
std::string serializeToBinary(const T& input) {
    std::string result;
    result.resize(sizeof(T));
    std::memcpy(result.data(), &input, sizeof(T));
    return result;
}

template<typename T>
T deserializeFromBinary(const std::string& input_str) {
    T result;
    if (input_str.size() == sizeof(T)) {
        std::memcpy(&result, input_str.data(), sizeof(T));
    }
    return result;
}

template<typename T>
T deserializeFromBinary(const void* input_ptr) {
    T result;
    std::memcpy(&result, input_ptr, sizeof(T));
    return result;
}

template<typename T>
T deserializeFromBinary(const std::vector<void*> input) {
    T result;
    std::memcpy(&result, input, sizeof(T));
    return result;
}

template<typename I,typename T>
T deserializeFromBinary(const I& input) {
    T result;
    std::memcpy(&result, &input[0], sizeof(T));
    return result;
}

} // end namespace HBC_HELPER

#endif