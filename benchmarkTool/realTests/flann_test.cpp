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
#include <fstream>

using namespace std;
using namespace flann;

// Scan all points from file
template <typename T>
void scanPointCloud(Matrix<T> &point, unsigned int &N, string file)
{
        ifstream read(file.c_str());
        
        string temp;
        getline(read,temp);

        vector<vector<T> > cloud;
        vector<T> tmp;
        
        T x,y,z,d;
        N=0;
        while(read>>x>>y>>z>>d){
            tmp.clear();
            tmp.push_back(x); tmp.push_back(y); tmp.push_back(z);
            cloud.push_back(tmp);
            N++;
        }
        random_shuffle(cloud.begin(), cloud.end());
    	Matrix<T> Point(new T[N*3], N, 3);
    	for (size_t i=0;i<N;i++)
    	{
    		Point[i][0] = cloud[i][0];
    		Point[i][1] = cloud[i][1];
    		Point[i][2] = cloud[i][2];
        }
        point=Point;
}


template <typename num_t>
void kdtree_demo(string &path)
{
        int nn=1;
        Matrix<num_t> PcloudS, PcloudT;
        unsigned int N;
        // Scan points from file:
        scanPointCloud<num_t>(PcloudS, N, path+"scan1.dat");
        scanPointCloud<num_t>(PcloudT, N, path+"scan2.dat");

        Matrix<num_t> query(new num_t[1*3], 1, 3);
    
        // buildTime : time required to build the kd-tree index
        // queryTime : time required to find nearest neighbor for a single point in the kd-tree
        vector<double> buildTime, queryTime;

        int plotCount=10;
        for(int i=1;i<=plotCount;i++)
        {
            // size of dataset currently being used
            int currSize=((i*1.0)/plotCount)*N;
            Matrix<num_t> cloudS(new num_t[currSize*3], currSize, 3);
            Matrix<num_t> cloudT(new num_t[currSize*3], currSize, 3);

            for(int j=0;j<currSize;j++)
            {
                for(int k=0;k<3;k++)
                {
                    cloudS[j][k]=PcloudS[j][k];
                    cloudT[j][k]=PcloudT[j][k];
                }
            }

            Index<L2<num_t> > index(cloudS, flann::KDTreeIndexParams(1));
            clock_t begin = clock();
            index.buildIndex();
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            buildTime.push_back(elapsed_secs);
            {

                clock_t begin = clock();

                for(int j=0;j<currSize;j++)
                {
                    query[0][0]=cloudT[j][0];
                    query[0][1]=cloudT[j][1];
                    query[0][2]=cloudT[j][2];
                    Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
                    Matrix<num_t> dists(new num_t[query.rows*nn], query.rows, nn);

                    index.knnSearch(query, indices, dists, nn, flann::SearchParams(-1));
                }

                clock_t end = clock();
                double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                queryTime.push_back(elapsed_secs/currSize);
            }
        }

        for(int i=0;i<buildTime.size();i++)
            std::cout<<buildTime[i]<<" ";
        std::cout<<"\n";

        for(int i=0;i<queryTime.size();i++)
            std::cout<<queryTime[i]<<" ";
        std::cout<<"\n";
}

int main()
{
	srand(time(NULL));
        //randomly choose some dataset from dat_avz/001 to dat_avz/010 [fixed right now -- update later]
        string dataset_path="/home/pranjalr34/gsoc/nanoflann/benchmarkTool/realTests/dat_avz/001/";
	kdtree_demo<double>(dataset_path);
	return 0;
}
