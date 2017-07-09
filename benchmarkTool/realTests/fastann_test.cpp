#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vector>

#include <stdint.h>

#include "fastann.hpp"
#include "rand_point_gen.hpp"

static inline uint64_t rdtsc()
{
    #ifdef __i386__
    uint32_t a, d;
#elif defined __x86_64__
    uint64_t a, d;
#endif

    asm volatile ("rdtsc" : "=a" (a), "=d" (d));

    return ((uint64_t)a | (((uint64_t)d)<<32));
}

template<class Float>
int
test_kdtree(unsigned N, unsigned D, double min_accuracy)
{
    Float* pnts = fastann::gen_unit_random<Float>(N, D, 42);
    Float* qus = fastann::gen_unit_random<Float>(N, D, 43);
    
    std::vector<Float> mins_exact(N);
    std::vector<unsigned> argmins_exact(N);
    std::vector<Float> mins_kdt(N);
    std::vector<unsigned> argmins_kdt(N);

    fastann::nn_obj<Float>* nnobj_exact = fastann::nn_obj_build_exact(pnts, N, D);
    fastann::nn_obj<Float>* nnobj_kdt = fastann::nn_obj_build_kdtree(pnts, N, D, 8, 768);

    nnobj_exact->search_nn(qus, N, &argmins_exact[0], &mins_exact[0]);
    nnobj_kdt->search_nn(qus, N, &argmins_kdt[0], &mins_kdt[0]);
    
    unsigned num_same = 0;
    for (unsigned n = 0; n < N; ++n) {
        if (argmins_exact[n] == argmins_kdt[n]) num_same++;
    }
    
    double accuracy = (double)num_same/N;
    printf("Accuracy: %.1f%%\n", accuracy*100.0);

    if (accuracy > min_accuracy) return 1;
    else return 0;

    delete[] pnts;
    delete[] qus;

    delete nnobj_exact;
    delete nnobj_kdt;
}

template<>
int
test_kdtree<unsigned char>(unsigned N, unsigned D, double min_accuracy)
{
    double* pntst = fastann::gen_unit_random<double>(N, D, 42);
    double* qust = fastann::gen_unit_random<double>(N, D, 43);

    unsigned char* pnts = new unsigned char[N*D];
    unsigned char* qus = new unsigned char[N*D];

    for (unsigned n=0; n < N; ++n) {
        for (unsigned d=0; d < D; ++d) {
            pnts[n*D + d] = (unsigned char)(256.0 * pntst[n*D + d]);
            qus[n*D + d] = (unsigned char)(256.0 * qust[n*D + d]);
        }
    }

    std::vector<unsigned> mins_exact(N);
    std::vector<unsigned> argmins_exact(N);
    std::vector<unsigned> mins_kdt(N);
    std::vector<unsigned> argmins_kdt(N);

    fastann::nn_obj<unsigned char>* nnobj_exact = fastann::nn_obj_build_exact(pnts, N, D);
    fastann::nn_obj<unsigned char>* nnobj_kdt = fastann::nn_obj_build_kdtree(pnts, N, D, 8, 768);

    nnobj_exact->search_nn(qus, N, &argmins_exact[0], &mins_exact[0]);
    nnobj_kdt->search_nn(qus, N, &argmins_kdt[0], &mins_kdt[0]);
    
    unsigned num_same = 0;
    for (unsigned n = 0; n < N; ++n) {
        if (argmins_exact[n] == argmins_kdt[n]) num_same++;
    }

    double accuracy = (double)num_same/N;
    printf("Accuracy: %.1f%%\n", accuracy*100.0);

    if (accuracy > min_accuracy) return 1;
    else return 0;
    
    delete[] pnts;
    delete[] qus;

    delete nnobj_exact;
    delete nnobj_kdt;
}

int
main()
{
    unsigned N = 10000;
    unsigned D = 128;
    
    unsigned num_failed = 0;
    unsigned num_passed = 0;
    double min_accuracy = 0.36; // Seems to be right.
    // This should be repeatable everywhere because of randomkit.c
    // It sounds low, but is to be expected with unit random vectors
    // With SIFT we do _much_ better.

    if (test_kdtree<unsigned char>(N, D, min_accuracy)) { num_passed++; }
    else { num_failed++; }

    if (test_kdtree<float>(N, D, min_accuracy)) { num_passed++; }
    else { num_failed++; }

    if (test_kdtree<double>(N, D, min_accuracy)) { num_passed++; }
    else { num_failed++; }

    printf("NUM_PASSED %d  NUM_FAILED %d\n", num_passed, num_failed);
    
    if (num_failed) return -1;
    else return 0;
}
