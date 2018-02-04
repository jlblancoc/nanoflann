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
#include <kdtree++/kdtree.hpp>
#include <set>
#include <string>
#include <vector>

using namespace std;
// used to ensure all triplets that are accessed via the operator<< are
// initialised.
std::set<const void *> registered;

struct triplet {
  typedef double value_type;

  triplet(value_type a, value_type b, value_type c) {
    d[0] = a;
    d[1] = b;
    d[2] = c;
    bool reg_ok = (registered.find(this) == registered.end());
    assert(reg_ok);
    registered.insert(this).second;
  }

  triplet(const triplet &x) {
    d[0] = x.d[0];
    d[1] = x.d[1];
    d[2] = x.d[2];
    bool reg_ok = (registered.find(this) == registered.end());
    assert(reg_ok);
    registered.insert(this).second;
  }

  ~triplet() {
    bool unreg_ok = (registered.find(this) != registered.end());
    assert(unreg_ok);
    registered.erase(this);
  }

  double distance_to(triplet const &x) const {
    double dist = 0;
    for (int i = 0; i != 3; ++i)
      dist += (d[i] - x.d[i]) * (d[i] - x.d[i]);
    return std::sqrt(dist);
  }

  inline value_type operator[](size_t const N) const { return d[N]; }

  value_type d[3];
};

inline bool operator==(triplet const &A, triplet const &B) {
  return A.d[0] == B.d[0] && A.d[1] == B.d[1] && A.d[2] == B.d[2];
}

std::ostream &operator<<(std::ostream &out, triplet const &T) {
  assert(registered.find(&T) != registered.end());
  return out << '(' << T.d[0] << ',' << T.d[1] << ',' << T.d[2] << ')';
}

inline double tac(triplet t, size_t k) { return t[k]; }

typedef KDTree::KDTree<3, triplet,
                       std::pointer_to_binary_function<triplet, size_t, double>>
    tree_type;

template <typename T>
vector<triplet> generateRandomPointCloud(const size_t N,
                                         const T max_range = 10) {
  vector<triplet> point;
  for (size_t i = 0; i < N; i++) {
    triplet tmp(max_range * (rand() % 1000) / T(1000),
                max_range * (rand() % 1000) / T(1000),
                max_range * (rand() % 1000) / T(1000));
    point.push_back(tmp);
  }
  return point;
}

template <typename num_t>
void kdtree_demo(const size_t N, double &buildTimer, double &queryTimer) {
  vector<triplet> cloudS = generateRandomPointCloud<num_t>(N);
  vector<triplet> cloudT = generateRandomPointCloud<num_t>(N);

  tree_type exact_dist(std::ptr_fun(tac));
  clock_t begin = clock();
  // construct a kd-tree index:
  for (size_t j = 0; j < N; j++)
    exact_dist.insert(cloudS[j]);
  exact_dist.optimise();
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  buildTimer += elapsed_secs;

  {
    double elapsed_secs = 0;
    for (size_t j = 0; j < N; j++) {
      triplet query_pt = cloudT[j];
      std::pair<tree_type::const_iterator, double> found;
      clock_t begin = clock();
      // do a knn search
      found = exact_dist.find_nearest(query_pt);
      clock_t end = clock();
      elapsed_secs += double(end - begin);
    }
    elapsed_secs /= CLOCKS_PER_SEC;
    queryTimer += elapsed_secs / N;
  }
}

int main(int argc, char *argv[]) {
  size_t plotCount = 10;
  size_t maxSize = 10000;

  if (argc == 3) {
    srand(atoi(argv[2]));
    maxSize = atoi(argv[1]);
  } else {
    cerr << "**Running Instructions:**\n ./benchmark_libkdtree_random "
            "numPoints seed\nExample:\n ./benchmark_libkdtree_random 10000 1"
         << endl;
    return 0;
  }

  // buildTime : time required to build the kd-tree index
  // queryTime : time required to find nearest neighbor for a single point in
  // the kd-tree
  vector<double> buildTime, queryTime;

  for (size_t i = 1; i <= plotCount; i++) {
    size_t currSize = ((i * 1.0) / plotCount) * maxSize;
    std::cout << currSize << " ";
    double buildTimer = 0, queryTimer = 0;
    kdtree_demo<float>(currSize, buildTimer, queryTimer);
    buildTime.push_back(buildTimer);
    queryTime.push_back(queryTimer);
  }
  cout << "\n";
  for (size_t i = 0; i < buildTime.size(); i++)
    std::cout << buildTime[i] << " ";
  std::cout << "\n";

  for (size_t i = 0; i < queryTime.size(); i++)
    std::cout << queryTime[i] << " ";
  std::cout << "\n";
  return 0;
}
