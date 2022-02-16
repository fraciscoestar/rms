#ifndef NOISEGENERATOR_H
#define NOISEGENERATOR_H

#include <iostream>
#include <random>
#include <chrono>

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
            
            generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
            dist = std::normal_distribution<double>(mean, stddev);
        }

        NoiseGenerator()
        {
            noiseParams = NoiseParams(0., 0.);
            generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
            dist = std::normal_distribution<double>(noiseParams.mean, noiseParams.stddev);
        }

        NoiseGenerator(const NoiseGenerator& noiseGenerator)
        {
            noiseParams = noiseGenerator.noiseParams;
            generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
            dist = std::normal_distribution<double>(noiseParams.mean, noiseParams.stddev);
        }

        NoiseGenerator(const NoiseGenerator&& noiseGenerator)
        {
            noiseParams = noiseGenerator.noiseParams;
            generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
            dist = std::normal_distribution<double>(noiseParams.mean, noiseParams.stddev);
        }

        void Initialize(double& mean, double& stddev)
        {
            noiseParams = NoiseParams(mean, stddev);
            generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
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

#endif