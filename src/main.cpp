/*
   Copyright (c) 2010 Toru Tamaki

   Permission is hereby granted, free of charge, to any person
   obtaining a copy of this software and associated documentation
   files (the "Software"), to deal in the Software without
   restriction, including without limitation the rights to use,
   copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following
   conditions:

   The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
   OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>

#include <helper_string.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>

#include "3dregistration.h"

using namespace std;

void loadFile(const char* fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	if (pcl::io::loadPCDFile(fileName, *cloud) == -1) {
		std::cerr << "Failed to read PCD file: " << fileName << std::endl;
		return;
	}

	// Remove NaN values.
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
}

/// Downsample point cloud using voxel grid.
void pointsReduction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size) {
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setInputCloud(cloud);
	voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_filter.filter(*cloud);
}

/// Scale the point cloud.
void alignScaleOnce1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
	Eigen::Vector4f center1, center2;
	Eigen::Matrix3f covariance1, covariance2;
	pcl::computeMeanAndCovarianceMatrix (*cloud1, covariance1, center1);
	pcl::computeMeanAndCovarianceMatrix (*cloud2, covariance2, center2);

	// symmetric scale estimation by Horn 1968
	float s = sqrt( covariance1.trace() / covariance2.trace());

	Eigen::Affine3f scale;
	scale.matrix() <<
		s, 0, 0, 0,
		0, s, 0, 0,
		0, 0, s, 0,
		0, 0, 0, 1;

	pcl::transformPointCloud (*cloud2, *cloud2, scale);
}

void alignScaleOnce(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
	Eigen::Vector4f center1, center2;
	Eigen::Matrix3f covariance1, covariance2;
	pcl::computeMeanAndCovarianceMatrix (*cloud1, covariance1, center1);
	pcl::computeMeanAndCovarianceMatrix (*cloud2, covariance2, center2);

	std::cout << "scale1 = " << covariance1.trace() << std::endl;
	std::cout << "scale2 = " << covariance2.trace() << std::endl;

	Eigen::Affine3f Scale;
	float s;
	s = 1.0 / sqrt( covariance1.trace() );
	s *= sqrt(0.003); // ad-hoc for numerical issue
	Scale.matrix() <<
		s, 0, 0, 0,
	0, s, 0, 0,
	0, 0, s, 0,
	0, 0, 0, 1;
	pcl::transformPointCloud ( *cloud1, *cloud1, Scale );

	s = 1.0 / sqrt( covariance2.trace() );
	s *= sqrt(0.003); // ad-hoc for numerical issue
	Scale.matrix() <<
		s, 0, 0, 0,
	0, s, 0, 0,
	0, 0, s, 0,
	0, 0, 0, 1;

	pcl::transformPointCloud ( *cloud2, *cloud2, Scale );
	pcl::computeMeanAndCovarianceMatrix (*cloud1, covariance1, center1);
	pcl::computeMeanAndCovarianceMatrix (*cloud2, covariance2, center2);

	std::cout << "scale1 = " << covariance1.trace() << std::endl;
	std::cout << "scale2 = " << covariance2.trace() << std::endl;
}

void init_RT(float *h_R, float *h_t) {
	// Set initial rotation to identity.
	h_R[0] = 1.0f;
	h_R[1] = 0.0f;
	h_R[2] = 0.0f;
	h_R[3] = 0.0f;
	h_R[4] = 1.0f;
	h_R[5] = 0.0f;
	h_R[6] = 0.0f;
	h_R[7] = 0.0f;
	h_R[8] = 1.0f;

	// Set initial translation to zeros.
	h_t[0] = 0.0f;
	h_t[1] = 0.0f;
	h_t[2] = 0.0f;
}

void printRT(const float* R, const float* t){
	printf("R\n");
	for(int r=0; r<9; r++){
		printf("%f ", R[r]);
		if((r+1)%3==0) printf("\n");
	}
	printf("t\n");
	for(int r=0; r<3; r++)
		printf("%f ", t[r]);
	printf("\n");
}

void saveRTtoFile(const float* R, const float* t, const char* filename){
	FILE *fp;
	if((fp = fopen(filename, "w")) != NULL){
		for(int r=0; r<9; r++){
			fprintf(fp, "%f ", R[r]);
			if((r+1)%3==0) fprintf(fp,"\n");
		}
		for(int r=0; r<3; r++)
			fprintf(fp,"%f ", t[r]);
		fprintf(fp,"\n");
	}
	fclose(fp);
}

void loadRTfromFile(float* R, float* t, const char* filename) {
	FILE *fp;
	if ((fp = fopen(filename, "r")) != NULL) {
		if (12 != fscanf(fp,"%f%f%f%f%f%f%f%f%f%f%f%f",
				&R[0], &R[1], &R[2],
				&R[3], &R[4], &R[5],
				&R[6], &R[7], &R[8],
				&t[0], &t[1], &t[2]
		)) {
			fprintf(stderr, "Fail to read RT from file [%s]\n", filename);
			exit(1);
		}
	}
}

int main(int argc, char** argv) {
	char *pointFileX, *pointFileY;
	int wrongArg = 0;

	// Read filenames of point clouds X and Y.
	// File format must be pcd.
	if (getCmdLineArgumentString(argc, (const char **) argv, "pointFileX", &pointFileX) &&
		getCmdLineArgumentString(argc, (const char **) argv, "pointFileY", &pointFileY)){
		cout << "option: pointFileX= " << pointFileX << endl;
		cout << "option: pointFileY=" << pointFileY << endl;
	} else {
		cerr << "Wrong arguments. see src." << endl;
		cerr << "min ||X - (R*Y+t) || " << endl;
		exit(1);
	}

	int Xsize, Ysize;

	// initialize parameters
	registrationParameters param;

	// Initialize EM-ICP parameters.
	if (! (param.sigma_p2     = getCmdLineArgumentFloat(argc, (const char **) argv, "sigma_p2") ) )     param.sigma_p2 = 0.01f;
	if (! (param.sigma_inf    = getCmdLineArgumentFloat(argc, (const char **) argv, "sigma_inf") ) )    param.sigma_inf = 0.00001f;
	if (! (param.sigma_factor = getCmdLineArgumentFloat(argc, (const char **) argv, "sigma_factor") ) ) param.sigma_factor = 0.9f;
	if (! (param.d_02         = getCmdLineArgumentFloat(argc, (const char **) argv, "d_02") ) )	        param.d_02 = 0.01f;

	cout << "EM-ICP paramters" << endl
		<< "sigma_p2 " << param.sigma_p2 << endl
		<< "sigma_inf " << param.sigma_inf << endl
		<< "sigma_factor " << param.sigma_factor << endl
		<< "d_02 " << param.d_02 << endl;

	// Set the remaining parameters.
	param.notimer  = checkCmdLineFlag(argc, (const char **) argv, "notimer");
	param.argc     = argc;
	param.argv     = argv;

	// read points, and initialize
	float *h_X, *h_Y;

	// h_X stores points as the order of
	// [X_x1 X_x2 .... X_x(Xsize-1) X_y1 X_y2 .... X_y(Xsize-1)  X_z1 X_z2 .... X_z(Xsize-1) ],
	// where (X_xi X_yi X_zi) is the i-th point in X.
	// h_Y does the same for Y.
	param.cloud_source.reset(new pcl::PointCloud<pcl::PointXYZ>());
	param.cloud_target.reset(new pcl::PointCloud<pcl::PointXYZ>());

	// Load the pointclouds from specified files.
	loadFile(pointFileX, param.cloud_target);
	loadFile(pointFileY, param.cloud_source);

	if (checkCmdLineFlag(argc, (const char **) argv, "alignScaleOnce"))
		alignScaleOnce(param.cloud_target, param.cloud_source);

	// Downsample scene point cloud.
	float pointsReductionRate = 0.005;
	if ( (pointsReductionRate = getCmdLineArgumentFloat(argc, (const char **) argv, "pointsReductionRate") ) ) {
		pointsReduction(param.cloud_target, pointsReductionRate);
		pointsReduction(param.cloud_source, pointsReductionRate);
		cout << "number of points X,Y are reduced to "
			<< pointsReductionRate << "% of original." << endl;
	} else {
		if ( (pointsReductionRate = getCmdLineArgumentFloat(argc, (const char **) argv, "pointsReductionRateX") ) ) {
			pointsReduction(param.cloud_target, pointsReductionRate);
			cout << "number of points X are reduced to "
				<< pointsReductionRate << "% of original." << endl;
		}
		if ( (pointsReductionRate = getCmdLineArgumentFloat(argc, (const char **) argv, "pointsReductionRateY") ) ) {
			pointsReduction(param.cloud_source, pointsReductionRate);
			cout << "number of points Y are reduced to "
				<< pointsReductionRate << "% of original." << endl;
		}
	}

	cout << "Xsize: " << param.cloud_target->size() << endl
		<< "Ysize: " << param.cloud_source->size() << endl;

	float* h_R = new float [9]; // rotation matrix
	float* h_t = new float [3]; // translation vector
	init_RT(h_R, h_t); // set R to Identity matrix, t to zero vector

	// Set the initial guess for the transformation matrix.
	char *loadRTfromFilename;
	if (getCmdLineArgumentString(argc, (const char **) argv, "loadRTfromFile", &loadRTfromFilename)) {
		loadRTfromFile(h_R, h_t, loadRTfromFilename);
	} else {
		init_RT(h_R, h_t); // set R to Identity matrix, t to zero vector
	}
	// Print the initial guess.
	printRT(h_R, h_t);

	clock_t start, end;
	start = clock();

	// Call the function that executes the algorithm.
	emicp(param.cloud_target, param.cloud_source, h_R, h_t, param);

	end = clock();
	printf("elapsed time: %f\n", (double)(end - start) / CLOCKS_PER_SEC);

	// Print the results and write it to file.
	printRT(h_R, h_t);
	char *saveRTtoFilename;
	if (getCmdLineArgumentString(argc, (const char **) argv, "saveRTtoFile", &saveRTtoFilename))
		saveRTtoFile(h_R, h_t, saveRTtoFilename);

	// Clean up.
	delete [] h_X;
	delete [] h_Y;
	delete [] h_R;
	delete [] h_t;

	// Cleanly exit program.
	return 0;
}

