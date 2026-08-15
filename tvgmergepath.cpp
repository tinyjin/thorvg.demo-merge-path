/*
 * Copyright (c) 2026 the ThorVG project. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "demo_tiles.h"
#include "demo_stress.h"
#include "demo_comb.h"

int main(int argc, char **argv)
{
    auto stress = 0;
    auto comb = 0;
    auto trace = false;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-t")) trace = true;
        else if (!strcmp(argv[i], "-s")) stress = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 12;
        else if (!strcmp(argv[i], "-c")) comb = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 128;
    }

    if (comb > 0) {
        printf("comb: %d teeth crossed by one bar, up to %d crossings land on a single segment\n", comb, comb * 2);
        return tvgdemo::main(new CombDemo(uint32_t(comb), trace), argc, argv, true, 1200, 700, 0);
    }

    if (stress > 0) {
        printf("stress: a union of %d curve blobs, accumulated over %d merges\n", stress, stress - 1);
        return tvgdemo::main(new StressDemo(uint32_t(stress), trace), argc, argv, true, 900, 900, 0);
    }

    printf("tiles: mm 1~5  /  accumulated | shared edge | reused operand | same shape +/-\n");

    return tvgdemo::main(new TilesDemo(trace), argc, argv, true, 1600, 640, 0);
}
