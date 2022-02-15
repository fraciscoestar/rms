#include <iostream>
#include <iterator>
#include "../include/rms/noiseGenerator.hpp"

int main(int argc, char* argv[])
{
    std::vector<double> data {30, 30, 30, 30, 31, 31, 32, 32};

    const double mean = 0.0;
    const double stddev = 2.5;

    NoiseGenerator noiseGenerator(mean, stddev);

    for (auto& x : data)
    {
        x = noiseGenerator.AddAWGN(x);
    }

    std::copy(begin(data), end(data), std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n";

    return 0;
}