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
#include <fastann.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace fastann;

// Scan all points from file
template <class T> T *scanPointCloud(unsigned int &N, string file) {
  ifstream read(file.c_str());

  string temp;
  getline(read, temp);

  vector<vector<T>> cloud;
  vector<T> tmp;

  T x, y, z, d;
  N = 0;
  while (read >> x >> y >> z >> d) {
    tmp.resize(3);
    tmp[0] = x;
    tmp[1] = y;
    tmp[2] = z;
    cloud.push_back(tmp);
    N++;
  }
  T *point = new T[N * 3];
  for (unsigned int n = 0; n < N; ++n) {
    for (unsigned int d = 0; d < 3; ++d) {
      point[n * 3 + d] = cloud[n][d];
    }
  }
  return point;
}

template <typename num_t> void kdtree_demo(string &path1, string &path2) {
  num_t *PcloudS, *PcloudT;
  unsigned int N;
  // Scan points from file
  PcloudS = scanPointCloud<num_t>(N, path1);
  PcloudT = scanPointCloud<num_t>(N, path2);

  // buildTime : time required to build the kd-tree index
  // queryTime : time required to find nearest neighbor for a single point in
  // the kd-tree
  vector<double> buildTime, queryTime;

  unsigned int plotCount = 10, dim = 3;

  for (unsigned int i = 1; i <= plotCount; i++) {
    // size of dataset currently being used
    unsigned int currSize = ((i * 1.0) / plotCount) * N;
    std::cout << currSize << " ";
    num_t *cloudS = new num_t[currSize * dim];
    num_t *cloudT = new num_t[currSize * dim];
    ;
    for (unsigned int n = 0; n < currSize; ++n) {
      for (unsigned int d = 0; d < dim; ++d) {
        cloudS[n * dim + d] = PcloudS[n * dim + d];
        cloudT[n * dim + d] = PcloudT[n * dim + d];
      }
    }

    clock_t begin = clock();
    // construct a kd-tree index:
    nn_obj<num_t> *nnobj_exact = nn_obj_build_exact(cloudS, currSize, 3);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    buildTime.push_back(elapsed_secs);

    {
      vector<num_t> mins_exact(currSize);
      vector<unsigned> argmins_exact(currSize);
      clock_t begin = clock();
      // do a knn search
      nnobj_exact->search_nn(cloudT, currSize, &argmins_exact[0],
                             &mins_exact[0]);
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
    cerr << "**Running Instructions:**\n./benchmark_fastann_real dataFile1 "
            "dataFile"
         << endl;
    return 0;
  }
  string dataFile1(argv[1]);
  string dataFile2(argv[2]);
  kdtree_demo<double>(dataFile1, dataFile2);
  return 0;
}
