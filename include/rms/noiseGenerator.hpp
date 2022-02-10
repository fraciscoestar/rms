#include <iostream>
#include <random>

class NoiseGenerator
{
    public:
        struct NoiseParams
        {
            double mean;
            double stddev;

            NoiseParams() {}

            NoiseParams(double mean, double stddev)
            {
                this->mean = mean;
                this->stddev = stddev;
            }
        };    

        NoiseGenerator(double mean, double stddev)
        {
            noiseParams.mean = mean;
            noiseParams.stddev = stddev;
            dist = std::normal_distribution<double>(mean, stddev);
        }

        NoiseGenerator()
        {
            noiseParams = NoiseParams(0., 0.);
            dist = std::normal_distribution<double>(noiseParams.mean, noiseParams.stddev);
        }

        NoiseGenerator(const NoiseGenerator& noiseGenerator)
        {
            noiseParams = noiseGenerator.noiseParams;
            dist = std::normal_distribution<double>(noiseParams.mean, noiseParams.stddev);
        }

        NoiseGenerator(const NoiseGenerator&& noiseGenerator)
        {
            noiseParams = noiseGenerator.noiseParams;
            dist = std::normal_distribution<double>(noiseParams.mean, noiseParams.stddev);
        }

        double AddAWGN(const double& value)
        {
            return value + dist(generator);
        }

        NoiseParams GetNoiseParams()
        {
            return noiseParams;
        }

    private:
        std::default_random_engine generator;
        std::normal_distribution<double> dist;
        NoiseParams noiseParams;
};  