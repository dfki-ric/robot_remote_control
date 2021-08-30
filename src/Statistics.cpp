#include "Statistics.hpp"
#include <algorithm>

namespace robot_remote_control {

Statistics::Stats::Stats(const double &runningAvgSamples) {
    statdata.bytesTotal = 0;
    statdata.bytesSinceLast = 0;
    statdata.bpsLast = 0;
    statdata.bpsAvg = 0;
    gettimeofday(&statdata.lastCalc, 0);
    statdata.runningAvgSamples = runningAvgSamples;
    runningAvgFactor = ((runningAvgSamples-1)/runningAvgSamples);
}


void Statistics::Stats::addBytesSent(const double& bytes) {
    statdata.bytesTotal += bytes;
    statdata.bytesSinceLast += bytes;
}

void Statistics::Stats::calculate(timeval* currenttime) {
    timersub(currenttime, &statdata.lastCalc, &diff);
    double seconds = (diff.tv_sec * 1000000 + static_cast<double>(diff.tv_usec))/1000000.0;
    statdata.bpsLast = (statdata.bytesSinceLast/seconds);
    statdata.bpsAvg = (statdata.bpsAvg * runningAvgFactor) + (statdata.bpsLast/statdata.runningAvgSamples);
    statdata.lastCalc = *currenttime;
    statdata.bytesSize = statdata.bytesSinceLast;
    statdata.bytesSinceLast = 0;
}

void Statistics::Stats::print(const std::string& name) {
    printf("%.2f kBytes/s avg: %s (size kB: %.2f)\n", statdata.bpsAvg/1000.0, name.c_str(), statdata.bytesSize/1000.0);
}

void Statistics::calculate() {
    gettimeofday(&currenttime, 0);
    global.calculate(&currenttime);
    std::for_each(stat_per_type.begin(), stat_per_type.end(), [&](auto & stat){
        stat.calculate(&currenttime);
    });
}

void Statistics::print(const bool &verbose) {
    global.print("global");
    if (verbose) {
        for (int i = 1; i < names.size(); ++i) {
            stat_per_type[i].print(names[i]);
        }
    }
}



}  // namespace robot_remote_control
