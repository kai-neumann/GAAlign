//
// Created by Kai on 28.04.2022.
//

#ifndef GAALIGN_FAST_SHUFFLE_H
#define GAALIGN_FAST_SHUFFLE_H

#include "vector"
#include <cstdint>
#include <cmath>

namespace gaalign {
    /*
     * This File contains a high performant shuffling implementation from https://github.com/lemire/Code-used-on-Daniel-Lemire-s-blog/blob/master/2016/06/29/shuffle.c
     * This functions were written by Daniel Lemire's and published on his blog https://lemire.me/blog/2016/06/30/fast-random-shuffling/
     */

    static inline uint32_t pcg32_random_bounded_divisionless_with_slight_bias(uint32_t range);

    void shuffle_pcg_divisionless_with_slight_bias(uint32_t *storage, uint32_t size);
}

#endif //GAALIGN_FAST_SHUFFLE_H
