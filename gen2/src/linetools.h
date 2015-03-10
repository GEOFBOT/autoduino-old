/**
 * Autoduino Project
 * Distributed under the MIT License.
 * (c) 2015 Geoffrey Mon
 * https://github.com/GEOFBOT/
 * https://github.com/GEOFBOT/autoduinoROS
 *
 * Line Tools
 * Utilities for processing lines and points
 *
**/
#ifndef LINETOOLS_H
#define LINETOOLS_H

#include <opencv2/opencv.hpp>
#include <vector>

// Structure that stores lines and their importance
// based on how many nearby lines were averaged to make
// that line.
struct roadline {
	std::vector<cv::Point> line;
	int score;
};

// Comparison functions for std::sort()
bool sortLineByScore(const roadline &a, const roadline &b);
bool pointVectorSort(const std::vector<cv::Point> &a, const std::vector<cv::Point> &b);

// Angle conversions
double radian(double degrees);
double degree(double radians);

// Check if value is near range
bool nearValue(double value1, double value2, double diff);
bool nearRange(double value1, double range1, double range2);

// Line operations
bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r);
void displayLine(cv::Vec2f ln, cv::Mat output, cv::Scalar color = cv::Scalar(0, 0, 0), int thickness = 1, int mode = 16);
std::vector<cv::Point> convertLine(cv::Vec2f ln);
std::vector<roadline> groupLines(std::vector<cv::Vec2f> lines, int diff, cv::Mat ROI);

// Point operations
cv::Point averagePoints(std::vector<cv::Point> points);

// Convert ROI points and lines to use coordinates of the larger image
cv::Point pointCoordROI2Img(cv::Point p, cv::Mat ROI);
std::vector<cv::Point> lineCoordROI2Img(std::vector<cv::Point> ln, cv::Mat ROI);
cv::Point getROIoffset(cv::Mat ROI);
std::vector<cv::Point> lineCropConvert(cv::Vec2f ln, cv::Mat ROI);

#endif // LINETOOLS_H