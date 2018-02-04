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

#include <cstdlib>
#include <ctime>
#include <flann/flann.hpp>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using namespace flann;

// Scan all points from file
template <typename T> Matrix<T> scanPointCloud(unsigned int &N, string file) {
  ifstream read(file.c_str());

  string temp;
  getline(read, temp);

  vector<vector<T>> cloud;
  vector<T> tmp;
  int dim = 3;

  T x, y, z, d;
  N = 0;
  while (read >> x >> y >> z >> d) {
    tmp.resize(dim);
    tmp[0] = x;
    tmp[1] = y;
    tmp[2] = z;
    cloud.push_back(tmp);
    N++;
  }
  Matrix<T> point(new T[N * dim], N, 3);
  for (unsigned int i = 0; i < N; i++) {
    for (unsigned int j = 0; j < dim; j++) {
      point[i][j] = cloud[i][j];
    }
  }
  return point;
}

template <typename num_t> void kdtree_demo(string &path1, string &path2) {
  Matrix<num_t> PcloudS, PcloudT;
  unsigned int N;
  // Scan points from file
  PcloudS = scanPointCloud<num_t>(N, path1);
  PcloudT = scanPointCloud<num_t>(N, path2);

  // buildTime : time required to build the kd-tree index
  // queryTime : time required to find nearest neighbor for a single point in
  // the kd-tree
  vector<double> buildTime, queryTime;

  unsigned int plotCount = 10, nn = 1, dim = 3;

  for (unsigned int i = 1; i <= plotCount; i++) {
    // size of dataset currently being used
    unsigned int currSize = ((i * 1.0) / plotCount) * N;
    std::cout << currSize << " ";
    Matrix<num_t> cloudS(new num_t[currSize * dim], currSize, dim);
    Matrix<num_t> cloudT(new num_t[currSize * dim], currSize, dim);

    for (unsigned int j = 0; j < currSize; j++) {
      for (unsigned int k = 0; k < dim; k++) {
        cloudS[j][k] = PcloudS[j][k];
        cloudT[j][k] = PcloudT[j][k];
      }
    }

    clock_t begin = clock();
    // construct a kd-tree index:
    Index<L2<num_t>> index(cloudS, flann::KDTreeIndexParams(1));
    index.buildIndex();
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    buildTime.push_back(elapsed_secs);

    {
      Matrix<int> indices(new int[cloudT.rows * nn], cloudT.rows, nn);
      Matrix<num_t> dists(new num_t[cloudT.rows * nn], cloudT.rows, nn);
      clock_t begin = clock();
      // do a knn search
      index.knnSearch(cloudT, indices, dists, nn, flann::SearchParams(-1));
      clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      queryTime.push_back(elapsed_secs / currSize);
    }
  }
  std::cout << "\n";

  for (unsigned int i = 0; i < buildTime.size(); i++)
    std::cout << buildTime[i] << " ";
  std::cout << "\n";

  for (unsigned int i = 0; i < queryTime.size(); i++)
    std::cout << queryTime[i] << " ";
  std::cout << "\n";
}

int main(int argc, char **argv) {
  if (argc != 3) {
    cerr << "**Running Instructions:**\n./benchmark_flann_real dataFile1 "
            "dataFile"
         << endl;
    return 0;
  }
  string dataFile1(argv[1]);
  string dataFile2(argv[2]);
  kdtree_demo<double>(dataFile1, dataFile2);
  return 0;
}
