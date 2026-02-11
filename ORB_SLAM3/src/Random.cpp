#include <random>

#include "Random.h"


namespace Random
{

int RandomInt(int lo, int hi)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::uniform_int_distribution<int> dist(lo, hi);
    return dist(gen);
}

};
