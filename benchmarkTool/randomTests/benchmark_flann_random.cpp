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

#include <flann/flann.hpp>
#include <ctime>
#include <cstdlib>
#include <iostream>
 
using namespace std;
using namespace flann;

template <typename T>
Matrix<T> generateRandomPointCloud(const size_t N, const T max_range = 10)
{
    size_t dim=3;
    Matrix<T> point(new T[N*dim], N, 3);
    for (size_t i=0;i<N;i++)
    {
        for(size_t j=0;j<dim;j++)
        {
            point[i][j] = max_range * (rand() % 1000) / T(1000);
        }
    }
    return point;
}

template <typename num_t>
void kdtree_demo(const size_t N, double &buildTimer, double &queryTimer)
{
    Matrix<num_t> cloudS = generateRandomPointCloud<num_t>(N);
    Matrix<num_t> cloudT = generateRandomPointCloud<num_t>(N);

    size_t nn=1;

    clock_t begin = clock();
    // construct a kd-tree index:
    Index<L2<num_t> > index(cloudS, flann::KDTreeIndexParams(1));
    index.buildIndex();
    clock_t end = clock();
    buildTimer += double(end - begin) / CLOCKS_PER_SEC;
    
    {
        Matrix<int> indices(new int[cloudT.rows*nn], cloudT.rows, nn);
        Matrix<num_t> dists(new num_t[cloudT.rows*nn], cloudT.rows, nn);
        clock_t begin = clock();
        // do a knn search
        index.knnSearch(cloudT, indices, dists, nn, flann::SearchParams(-1));
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
        cerr << "**Running Instructions:**\n ./benchmark_flann_random numPoints seed\nExample:\n ./benchmark_flann_random 10000 1" << endl;
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
