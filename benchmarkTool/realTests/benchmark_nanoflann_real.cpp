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
#include <fstream>
#include <iostream>
#include <nanoflann.hpp>
#include <string>

using namespace std;
using namespace nanoflann;

// This is an exampleof a custom data set class
template <typename T> struct PointCloud {
  struct Point {
    T x, y, z;
  };

  std::vector<Point> pts;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx].x;
    else if (dim == 1)
      return pts[idx].y;
    else
      return pts[idx].z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX> bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

// Scan all points from file
template <typename T>
PointCloud<T> scanPointCloud(unsigned int &N, string file) {
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
  PointCloud<T> point;
  point.pts.resize(N);
  for (unsigned int i = 0; i < N; i++) {
    point.pts[i].x = cloud[i][0];
    point.pts[i].y = cloud[i][1];
    point.pts[i].z = cloud[i][2];
  }
  return point;
}

template <typename num_t> void kdtree_demo(string &path1, string &path2) {
  PointCloud<num_t> PcloudS, PcloudT;
  unsigned int N;
  // Scan points from file
  PcloudS = scanPointCloud<num_t>(N, path1);
  PcloudT = scanPointCloud<num_t>(N, path2);

  // buildTime : time required to build the kd-tree index
  // queryTime : time required to find nearest neighbor for a single point in
  // the kd-tree
  vector<double> buildTime, queryTime;

  unsigned int plotCount = 10;

  for (unsigned int i = 1; i <= plotCount; i++) {
    // size of dataset currently being used
    unsigned int currSize = ((i * 1.0) / plotCount) * N;
    std::cout << currSize << " ";
    PointCloud<num_t> cloudS, cloudT;
    cloudS.pts.resize(currSize);
    cloudT.pts.resize(currSize);

    for (unsigned int j = 0; j < currSize; j++) {
      cloudS.pts[j] = PcloudS.pts[j];
      cloudT.pts[j] = PcloudT.pts[j];
    }

    clock_t begin = clock();
    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */
        >
        my_kd_tree_t;
    my_kd_tree_t index(3 /*dim*/, cloudS,
                       KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    buildTime.push_back(elapsed_secs);

    {
      double elapsed_secs = 0;
      for (unsigned int j = 0; j < currSize; j++) {
        num_t query_pt[3];
        query_pt[0] = cloudT.pts[j].x;
        query_pt[1] = cloudT.pts[j].y;
        query_pt[2] = cloudT.pts[j].z;
        size_t ret_index;
        num_t out_dist_sqr;
        const size_t num_results = 1;
        KNNResultSet<num_t> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);
        clock_t begin = clock();
        // do a knn search
        index.findNeighbors(resultSet, &query_pt[0],
                            nanoflann::SearchParams(10));
        clock_t end = clock();
        elapsed_secs += double(end - begin);
      }
      elapsed_secs /= CLOCKS_PER_SEC;
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
    cerr << "**Running Instructions:**\n./benchmark_nanoflann_real dataFile1 "
            "dataFile"
         << endl;
    return 0;
  }
  string dataFile1(argv[1]);
  string dataFile2(argv[2]);
  kdtree_demo<double>(dataFile1, dataFile2);
  return 0;
}
