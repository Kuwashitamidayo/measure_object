#include "measure_object.hpp"

int main(int argc, char** argv)
{
    // Declare the output variables
    Mat dst, cdst, cdstP;
    const char* default_file = "../screens/test001.png";
    const char* filename = argc >=2 ? argv[1] : default_file;

    printf("Using OpenCV, version %i.%i.%i \n", CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);
    help();

    // Loads an image
    Mat src = imread(filename, IMREAD_GRAYSCALE);
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }

    Mat res = src.clone();
    cvtColor(res, res, COLOR_GRAY2BGR);

    // Edge detection
    Canny(src, dst, 50, 200, 3);
    Mat edges = dst.clone();

    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();

    // find a base line to with measures are done 
	Point A, B;
    Point2f triangle_src[3];
    double found_line_len;

    findBaseLine(dst, A, B);
	findTriangle(dst, A, B, triangle_src, found_line_len);

    printf("\nFitLine coordinates: (%f, %f), (%f, %f), line length = %f\n\n", triangle_src[0].x, triangle_src[0].y, triangle_src[1].x, triangle_src[1].y, found_line_len);

    //for warpAffine - we need to rotate the whole pic to eliminate baseline slope
    Point2f triangle_target[3];
    triangle_target[0]  = Point(0, src.rows);
    triangle_target[1]  = Point(found_line_len, src.rows);
    triangle_target[2]  = Point(found_line_len, 0);

    Mat warp_mat = getAffineTransform(triangle_src, triangle_target);
    Mat inv_warp_mat = getAffineTransform(triangle_target, triangle_src);
    Mat warp_src = Mat::zeros(src.rows, (int)found_line_len, src.type());
    warpAffine(edges, warp_src, warp_mat, warp_src.size());

    vector<int> vector_of_heights;
    Mat measured_heights = createMapOfMaxHeights(warp_src, vector_of_heights);

    line(res, triangle_src[0], triangle_src[1], Scalar(0,255,255), 1, LINE_AA);
    line(cdstP, triangle_src[0], triangle_src[1], Scalar(0,255,255), 1, LINE_AA);

    vector<Point> local_max_rot, local_max;

    local_max_rot = findHighestWhitePixels(edges.rows, vector_of_heights);
    local_max = remapPointsToOriginalImage(local_max_rot, inv_warp_mat);

    Mat edges_color;
    cvtColor(warp_src, edges_color, COLOR_GRAY2BGR);
    paintPoints(res, local_max, 5, cv::Scalar(0, 0, 255));
    paintPoints(edges_color, local_max_rot, 5, cv::Scalar(0, 0, 255));
    
    // printf("All points:\n");
    // printVector(vector_of_heights);
    // printf("\n\n");

    // printf("Results - local maximum points on src image:\n");
    // print(local_max);
    // printf("\n\n");

    // printf("Results - local maximum points:\n");
    // print(local_max_rot);
    // printf("\n\n");

    // Show results
    imshow("Source", src);
    // imshow("Detected Line (yellow) - Probabilistic Line Transform", cdstP);
    // imshow("Edges", edges);
    imshow("Rotated edges with points", edges_color);
    // imshow("Rotated edges", warp_src);
    // imshow("Max heights map", measured_heights);
    imshow("Result", res);
    // Wait and Exit
    waitKey();
    return 0;

}
