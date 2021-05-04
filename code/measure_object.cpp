#include "measure_object.hpp"

void help()
{
    printf("\n"
            "This program calculates width and height of the pins.\n"
            "Pins must be placed on a base (straight horizontal line), and must be directed upwards.\n"
			"You can obtain the best effects if the picture has only 2 colors\n"
			"(if not, then use image preprocessing, like threshold, erode/dilate etc.).\n\n"
            "Call\n"
            "./measure_object [optional path for picture]\n\n");
}

/**
 * Function to return the minimum distance 
 * between a line |AB| and a point E.
 * https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
 *
 * @param A (In) first point (x, y) of the |AB| line.
 * @param B (In) second point (x, y) of the |AB| line.
 * @param E (In) point (x, y), from which distance to |AB| is measured.
 * @return distance from E to |AB| in pixels stored as double.
 */
double minDistance(Point A, Point B, Point E) 
{ 
  
    // vector AB 
    Point AB = Point(B.x - A.x, B.y - A.y);
  
    // vector BP 
    Point BE = Point(E.x - B.x, E.y - B.y);
  
    // vector AP 
    Point AE = Point(E.x - A.x, E.y - A.y);
  
    // Variables to store dot product 
    double AB_BE, AB_AE; 
  
    // Calculating the dot product 
    AB_BE = (AB.x * BE.x + AB.y * BE.y); 
    AB_AE = (AB.x * AE.x + AB.y * AE.y); 
  
    // Case 1 
    if (AB_BE > 0) { 
        // Finding the magnitude 
        double y = E.y - B.y; 
        double x = E.x - B.x; 
        return sqrt(x * x + y * y); 
    } 
  
    // Case 2 
    if (AB_AE < 0) { 
        double y = E.y - A.y; 
        double x = E.x - A.x; 
        return sqrt(x * x + y * y); 
    } 
  
    // Case 3 
    // Finding the perpendicular distance 
    double x1 = AB.x; 
    double y1 = AB.y; 
    double x2 = AE.x; 
    double y2 = AE.y; 
    double mod = sqrt(x1 * x1 + y1 * y1); 
    return abs(x1 * y2 - y1 * x2) / mod; 
    
} 

// in progress
vector<int> findHighestWhitePixel(Mat img, vector<Point>& localMax) 
{
    localMax.clear();
    int step = 20;
    vector<int> highestValVec(img.cols);
    for(int j=0; j<img.cols; j++) {         // iterate from left to right
        for(int i=0; i<img.rows; i++) {     // iterate from up to down
            Point3i pixelVal = Point3i(img.at<cv::Vec3b>(i,j)[0], img.at<cv::Vec3b>(i,j)[1], img.at<cv::Vec3b>(i,j)[2]);
            if (pixelVal == Point3i(255, 255, 255)) {
                highestValVec[j] = i;
                if (j >= step && j <= img.cols - step) {
                    vector<int>::iterator val = max_element(highestValVec.begin() + j-step, highestValVec.begin() + j+step);
                    vector<int>::iterator val_before = max_element(highestValVec.begin() + j-step, highestValVec.begin() + j-(int)(step/2));
                    vector<int>::iterator val_after = max_element(highestValVec.begin() + j+(int)(step/2), highestValVec.begin() + j+step);
                    // printf("Value at %i column: %i\n", j, *val);
                    if (*val == highestValVec[j] && *val_before < highestValVec[j] &&
                    *val_after < highestValVec[j] && *val == highestValVec[j] && 
                    highestValVec[j] > highestValVec[j-1]) {
                        localMax.push_back(Point2i(j, highestValVec[j]));
                        //printf("Local Maximum at (%i, %i)\n", j, highestValVec[j]);
                    }

                }
                break;
            }
        }
    }
    
    return highestValVec;
}

/**
 * Returns a length of the |AB| line.
 *
 * @param A (In) first point (x, y) of the |AB| line.
 * @param B (In) second point (x, y) of the |AB| line.
 * @return length of the |AB| line defined as double.
 */
double lineLength(Point A, Point B) 
{
    return sqrt( pow(B.x - A.x, 2) + pow(B.y - A.y, 2) );
}

/**
 * Creates a map of pixels with max height only, measured from down to up.
 *
 * @param src (In) source image
 * @param vector_of_heights (InOut) returned vector of max heights in each column of the src image
 * @return image on black background with white pixels in positions pointed by vector_of_heights
 */
Mat createMapOfMaxHeights(Mat src, vector<int>& vector_of_heights)
{
    cv::threshold(src, src, 120, 255, THRESH_BINARY);
    // we assume that src is already grayscale
    Mat dst = Mat::zeros(src.rows, src.cols, src.type());
    int max_height;
    for (int i=0; i < src.cols; i++) {
        max_height = src.rows-1;
        for (int j=0; j < src.rows; j++) {
            if ((int)src.at<uchar>(j,i) > 250)
            {
                max_height = src.rows - j;
                break;
            }
        }
        dst.at<uchar>(max_height, i) = 255;
        vector_of_heights.push_back(max_height);
    }
    return dst;
}

/**
 * Find the most suitable base line using HoughLinesP.
 *
 * Returns a triangle with found line as a base.
 *
 * @param src (In) image after edge detection (e.g. Canny or laplacian).
 * @param A (InOut) first point (x, y) of the |AB| line.
 * @param B (InOut) second point (x, y) of the |AB| line.
 * @param max_a (In) max slope for line searching from function y=ax+b. Default is +/-1. Used to ignore other lines
 */
void findBaseLine(Mat src, Point& A, Point& B, double max_a)
{
    vector<Vec4i> lines_houghP; // will hold the results of the detection
    vector<Vec4i> lines_houghP_chosen;
    Mat cdstP;

    int minLineLength = 20;
    int maxLineGap = 10;

    // Probabilistic Line Transform
    HoughLinesP(src, lines_houghP, 1, CV_PI/180, 50, minLineLength, maxLineGap); // runs the actual detection

    double sum_a = 0, sum_b = 0;
    double count = 0.0;

    // Draw the lines for lines_houghP
    for( size_t i = 0; i < lines_houghP.size(); i++ )
    {
        Vec4i l = lines_houghP[i];
        double y2 = (double) l[3], x2 = (double) l[2], y1 = (double) l[1], x1 = (double) l[0]; 
        double a = (y2 - y1) / (x2 - x1);
        double b = y2 - a * x2; // y = ax+b => b = y - ax
        //printf("\nline %lo: y = %f x + %f ", i, a, b);

        
        // We want to find a base for further measurements, and it is horizontal
        if (abs(a) < abs(max_a) and !isinf(a)) {
            line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,i*10,255), 3, LINE_AA);
            double len = lineLength(Point(l[0], l[1]), Point(l[2], l[3]));
            sum_a += a*len;
            sum_b += b*len;
            count += len;
            //printf(", line %lo drawn with len %f", i, len);
            lines_houghP_chosen.push_back(Vec4f(l[0], l[1], l[2], l[3]));
            
        }
    }

    // LS method for find a line
    Vec4f ls_line;
    vector<Point2i> ls_points;
    for( size_t i = 0; i < lines_houghP_chosen.size(); i++ )
    {
        ls_points.push_back(Point(lines_houghP_chosen[i][0], lines_houghP_chosen[i][1]));
        ls_points.push_back(Point(lines_houghP_chosen[i][2], lines_houghP_chosen[i][3]));
    }

    // fit line that suits most to selected ones
    fitLine(ls_points, ls_line, CV_DIST_FAIR, 0, 0.01, 0.01);

    // parameters needed to define line found by fitLine() with start and end point
    float vx = ls_line[0];
    float vy = ls_line[1];
    float x0 = ls_line[2];
    float y0 = ls_line[3];

    // finding start and end point of line found by least squares, return values
    A.x = 0;
	A.y = int((-x0 * vy / vx) + y0);
    B.x = src.size().width - 1;
    B.y = int(((src.size().width - x0) * vy / vx) + y0);
}

/**
 * Find the rectangular triangle ABC with |AB| as base and |BC| as perpendicular line.
 *
 * Length of the |BC| line is equal to the height of the src image.
 *
 * @param src (In) source image
 * @param A (In) first point (x, y) of the |AB| line.
 * @param B (In) second point (x, y) of the |AB| line.
 * @param triangle (InOut) array that stores points A, B and C (in this order).
 * @param line_len (InOut) length of the |AB| line.
 */
void findTriangle(Mat src, Point A, Point B, Point2f (&triangle)[3], double& line_len) {

	// finding a and b -> y = ax+b, and finding a line perpendicular to it
    float a_ls  = float(B.y - A.y) / float(src.size().width - 1);
    float a_ls_perp = -1 / a_ls;                        //perpendicular line
    float b_ls  = A.y;
    float b_ls_perp = (a_ls - a_ls_perp)*B.x + b_ls;  //perpendicular line

    // third point to affine, to make a triangle
    // we need to rotate the img in the way that slope is eliminated
    float theta_ls = atan((B.y - A.y) / B.x);
    float offset_perp = -src.rows/cos(theta_ls);
    float x_perp = (b_ls_perp - (b_ls + offset_perp)) / (a_ls - a_ls_perp);
    float y_perp = a_ls_perp * x_perp + b_ls_perp;

    //return values
    triangle[0]  = A;
    triangle[1]  = B;
    triangle[2]  = Point(x_perp, y_perp);
    line_len = lineLength(A, B);
}


