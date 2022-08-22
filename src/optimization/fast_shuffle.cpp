//
// Created by Kai on 28.04.2022.
//

#include "fast_shuffle.h"
#include "fast_shuffle_pcg.h"

// map random value to [0,range) with slight bias
static inline uint32_t gaalign::pcg32_random_bounded_divisionless_with_slight_bias(uint32_t range) {
    uint64_t random32bit, multiresult;
    random32bit =  pcg32_random();
    multiresult = random32bit * range;
    return multiresult >> 32; // [0, range)
}

// good old Fisher-Yates shuffle, shuffling an array of integers, without division
// From: https://github.com/lemire/Code-used-on-Daniel-Lemire-s-blog/blob/master/2016/06/29/shuffle.c
void  gaalign::shuffle_pcg_divisionless_with_slight_bias(uint32_t *storage, uint32_t size) {
    uint32_t i;
    for (i=size; i>1; i--) {
        uint32_t nextpos = gaalign::pcg32_random_bounded_divisionless_with_slight_bias(i);
        uint32_t tmp = storage[i-1];// likely in cache
        uint32_t val = storage[nextpos]; // could be costly
        storage[i - 1] = val;
        storage[nextpos] = tmp; // you might have to read this store later
    }
}

