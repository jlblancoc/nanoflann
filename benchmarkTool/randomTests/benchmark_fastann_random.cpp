/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2016 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/
 
#include <fastann.hpp>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace fastann;

template <typename T>
T* generateRandomPointCloud(const size_t N, const T max_range = 10)
{
    T* point = new T[N*3];
    for (size_t n=0;n<N;n++)
    {
        for (size_t d=0; d < 3; ++d) 
        {
            point[n*3 + d] = max_range * (rand() % 1000) / T(1000);
        }
    }
    return point;
}

template <typename num_t>
void kdtree_demo(const size_t N, double &buildTimer, double &queryTimer)
{
    num_t* cloudS = generateRandomPointCloud<num_t>(N);
    num_t* cloudT = generateRandomPointCloud<num_t>(N);

    clock_t begin = clock();
    // construct a kd-tree index:
    nn_obj<num_t>* nnobj_exact = nn_obj_build_exact(cloudS, N, 3);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    buildTimer += elapsed_secs;

    {
        vector<num_t> mins_exact(N);
        vector<unsigned> argmins_exact(N);
        clock_t begin = clock();
        // do a knn search
        nnobj_exact->search_nn(cloudT, N, &argmins_exact[0], &mins_exact[0]);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        queryTimer += elapsed_secs/N;
    }
}

int main(int argc, char *argv[])
{
    size_t plotCount = 10;
    size_t maxSize = 10000;
    
    if(argc == 3)
    {
        srand(atoi(argv[2]));
        maxSize = atoi(argv[1]);
    }
    else
    {
        cerr << "**Running Instructions:**\n ./benchmark_fastann_random numPoints seed\nExample:\n ./benchmark_fastann_random 10000 1" << endl;
        return 0;
    }

    // buildTime : time required to build the kd-tree index
    // queryTime : time required to find nearest neighbor for a single point in the kd-tree
    vector<double> buildTime, queryTime;

    for (size_t i=1;i<=plotCount;i++)
    {
        size_t currSize=((i*1.0)/plotCount)*maxSize;
        std::cout<<currSize<<" ";
        double buildTimer = 0, queryTimer = 0;
        kdtree_demo<float>(currSize, buildTimer, queryTimer);
        buildTime.push_back(buildTimer);
        queryTime.push_back(queryTimer);
    }
    cout<<"\n";
    for(size_t i=0;i<buildTime.size();i++)
        std::cout<<buildTime[i]<<" ";
    std::cout<<"\n";

    for(size_t i=0;i<queryTime.size();i++)
        std::cout<<queryTime[i]<<" ";
    std::cout<<"\n";
    return 0;
}
