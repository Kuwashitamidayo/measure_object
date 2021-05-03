
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
vector<int> findHighestWhitePixel(Mat img, vector<Point>& localMax);
double lineLength(Point p1, Point p2);
Mat createMapOfMaxHeights(Mat src, vector<int>& vector_of_heights);
void findLine(Mat src, Point2f (&triangle)[3], double& line_len, double max_a=1.0);

#endif