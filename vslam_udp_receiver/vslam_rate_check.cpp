#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

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
    std::getline(ifs, line); // skip header

    std::vector<double> rates_hz;
    int64_t prev_ts = -1;

    while (std::getline(ifs, line))
    {
        int64_t ts;
        if (sscanf(line.c_str(), "%ld", &ts) != 1)
            continue;

        if (prev_ts >= 0)
        {
            double dt = (ts - prev_ts) * 1e-9; // sec
            if (dt > 0.0)
                rates_hz.push_back(1.0 / dt);
        }
        prev_ts = ts;
    }
    ifs.close();

    if (rates_hz.size() < 5) {
        std::cerr << "not enough samples\n";
        return -1;
    }

    // 정렬
    std::sort(rates_hz.begin(), rates_hz.end());

    // 이상치 제거 (min 1개, max 1개)
    rates_hz.erase(rates_hz.begin());
    rates_hz.pop_back();

    const size_t N = rates_hz.size();

    // 평균
    double sum = 0.0;
    for (double r : rates_hz) sum += r;
    double mean = sum / N;

    // 표준편차
    double var = 0.0;
    for (double r : rates_hz)
        var += (r - mean) * (r - mean);
    var /= N;

    double stddev = std::sqrt(var);

    // 최소 / 최대
    auto [min_it, max_it] = std::minmax_element(
        rates_hz.begin(), rates_hz.end());

    std::cout << "Samples (used) : " << N << "\n";
    std::cout << "Mean Rate     : " << mean << " Hz\n";
    std::cout << "Std Dev       : " << stddev << " Hz\n";
    std::cout << "Min Rate      : " << *min_it << " Hz\n";
    std::cout << "Max Rate      : " << *max_it << " Hz\n";

    return 0;
}
