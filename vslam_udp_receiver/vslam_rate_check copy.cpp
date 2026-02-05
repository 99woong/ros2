#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: analyzer <log.csv>\n";
        return -1;
    }

    std::ifstream ifs(argv[1]);
    if (!ifs.is_open()) {
        std::cerr << "file open failed\n";
        return -1;
    }

    std::string line;
    std::getline(ifs, line); // header

    std::vector<double> dt;
    int64_t prev_ts = -1;

    while (std::getline(ifs, line))
    {
        int64_t ts;
        sscanf(line.c_str(), "%ld", &ts);

        if (prev_ts >= 0) {
            dt.push_back((ts - prev_ts) * 1e-9);
        }
        prev_ts = ts;
    }

    ifs.close();

    double sum = 0;
    for (double v : dt) sum += v;
    double mean = sum / dt.size();

    double var = 0;
    for (double v : dt) var += (v - mean) * (v - mean);
    var /= dt.size();

    double stddev = std::sqrt(var);

    auto [min_it, max_it] = std::minmax_element(dt.begin(), dt.end());

    std::cout << "Samples: " << dt.size() + 1 << "\n";
    std::cout << "Mean dt: " << mean << " s\n";
    std::cout << "Std dt : " << stddev << " s\n";
    std::cout << "Min dt : " << *min_it << " s\n";
    std::cout << "Max dt : " << *max_it << " s\n";
    std::cout << "Mean Rate: " << 1.0 / mean << " Hz\n";

    return 0;
}

