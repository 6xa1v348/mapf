#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <random>
#include <regex>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>


constexpr int MAX_WEIGHT = INT_MAX / 2;
using Time = std::chrono::steady_clock;

template <typename T>
inline bool inArray(const T& a, const std::vector<T>& arr) {
    auto itr = std::find(arr.begin(), arr.end(), a);
    return itr != arr.end();
}

template <typename T>
inline bool inArray(const T* a, const std::vector<T*>& arr) {
    auto itr = std::find(arr.begin(), arr.end(), a);
    return itr != arr.end();
}

inline int getRandomInt(int min, int max, std::mt19937& MT) {
    std::uniform_int_distribution<int> r(min, max);
    return r(MT);
}

inline float getRandomFloat(float min, float max, std::mt19937& MT) {
    std::uniform_real_distribution<float> r(min, max);
    return r(MT);
}

inline auto getElapsedTime(const Time::time_point& t) {
    auto t2 = Time::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t).count();
}
