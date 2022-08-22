//
// Created by Kai on 01.05.2022.
//

#include "common.h"


double avg(const std::vector<double> &vec) {
    double sum = 0;
    for(double i : vec) {
        sum += i;
    }
    return sum / ((double)vec.size());
}
