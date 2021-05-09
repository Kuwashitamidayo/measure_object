
#ifndef MEASURE_OBJECTS_H
#define MEASURE_OBJECTS_H

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

void help();
double minDistance(Point A, Point B, Point E);
double lineLength(Point p1, Point p2);
void findBaseLine(Mat src, Point& A, Point& B, double max_a=1.0);
void findTriangle(Mat src, Point A, Point B, Point2f (&triangle)[3], double& line_len);

vector<cv::Point> findHighestWhitePixels(int img_height, vector<int> vector_of_heights, float min_val_height=0.7);
Mat createMapOfMaxHeights(Mat src, vector<int>& vector_of_heights);
vector<cv::Point> remapPointsToOriginalImage(vector<cv::Point> points, Mat rotation_matrix);
void paintPoints(Mat& src, vector<cv::Point> points, int point_size, cv::Scalar color);

void printVector(vector<int> vector);

#endif