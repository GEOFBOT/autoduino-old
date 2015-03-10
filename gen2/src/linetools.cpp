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
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include "linetools.h"

struct roadline;
struct sortLineByScore;

// Used in std::sort() to sort roadline structures in
// order to find the most prominent ones.
bool sortLineByScore(const roadline &a, const roadline &b)
{
	return a.score < b.score;
}

// Used in std::sort() to sort cv::Point vectors in
// order to find the largest ones.
bool pointVectorSort(const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
{
	return a.size() > b.size();
}

// Degrees -> radians
// Used when dealing with Hough line detection output
double radian(double degrees)
{
	return degrees * CV_PI / 180;
}

// Radians -> degrees
double degree(double radians)
{
	return radians * 180 / CV_PI;
}

// Used to test if value2 - diff < value1 < value2 + diff
bool nearValue(double value1, double value2, double diff)
{
	return (value2 - diff < value1) && (value1 < value2 + diff);
}

// Used to test if range1 < value1 < range2
bool nearRange(double value1, double range1, double range2)
{
	return (value1 > range1) && (value1 < range2);
}

// Source: http://stackoverflow.com/a/7448287
// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,
	cv::Point2f &r)
{
	cv::Point2f x = o2 - o1;
	cv::Point2f d1 = p1 - o1;
	cv::Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}


// Converts lines in polar [radius, theta] form (the form
// used by Hough line detection) and displays those lines.
void displayLine(cv::Vec2f ln, cv::Mat output, cv::Scalar color, int thickness, int mode)
{
	float r = ln[0], t = ln[1];
	cv::Point pt1, pt2;
	double a = std::cos(t), o = std::sin(t);
	double x0 = r * a, y0 = r * o;
	pt1.x = cvRound(x0 + (1000 * (-o)));
	pt1.y = cvRound(y0 + (1000 * (a)));
	pt2.x = cvRound(x0 - (1000 * (-o)));
	pt2.y = cvRound(y0 - (1000 * (a)));
	cv::line(output, pt1, pt2, color, thickness, mode);
}

// Converts lines in polar [radius, theta] form to Cartesian form.
std::vector<cv::Point> convertLine(cv::Vec2f ln)
{
	float r = ln[0], t = ln[1];
	cv::Point pt1, pt2;
	double a = std::cos(t), o = std::sin(t);
	double x0 = r * a, y0 = r * o;
	pt1.x = cvRound(x0 + (1000 * (-o)));
	pt1.y = cvRound(y0 + (1000 * (a)));
	pt2.x = cvRound(x0 - (1000 * (-o)));
	pt2.y = cvRound(y0 - (1000 * (a)));

	std::vector<cv::Point> v;
	if (pt1.y > pt2.y) v = { pt1, pt2 };
	else v = { pt2, pt1 };
	return v;
}

// Joins lines that are similar
std::vector<roadline> groupLines(std::vector<cv::Vec2f> lines, int diff, cv::Mat ROI)
{
	std::vector<std::vector<cv::Point>> converted;
	std::vector<std::vector<std::vector<cv::Point>>> lines2;
	std::vector<roadline> result_lines;

	for (int i = 0; i < lines.size(); ++i) {
		converted.push_back(lineCropConvert(lines[i], ROI));
	}

	for (int i = 0; i < converted.size(); ++i) {
		bool match = false;
		if (lines2.size() == 0) {
			lines2.push_back({ converted[i] });
		}
		else {
			bool match = false;
			for (int j = 0; j < lines2.size(); j++) {
				if (nearValue(converted[i][0].x, lines2[j][0][0].x, diff)
					&& nearValue(converted[i][1].x, lines2[j][0][1].x, diff)) {
					lines2[j].push_back(converted[i]);
					match = true;
					break;
				}
			}
			if (!match) {
				lines2.push_back({ converted[i] });
			}
		}
	}

	for (int i = 0; i < lines2.size(); i++) {
		roadline rl;
		float total_0x = 0;
		float total_1x = 0;
		for (int j = 0; j < lines2[i].size(); j++) {
			total_0x += lines2[i][j][0].x;
			total_1x += lines2[i][j][1].x;
		}
		rl.line = { cv::Point(float(total_0x / lines2[i].size()), lines2[i][0][0].y),
			cv::Point(float(total_1x / lines2[i].size()), lines2[i][0][1].y) };
		rl.score = lines2[i].size();
		result_lines.push_back(rl);
	}

	sort(result_lines.begin(), result_lines.end(), sortLineByScore);

	return result_lines;
}

// Averages all the points in a vector and returns the average point
cv::Point averagePoints(std::vector<cv::Point> points)
{
	cv::Point pt;
	double x = 0, y = 0;
	for (int i = 0; i < points.size(); ++i) {
		x += points[i].x;
		y += points[i].y;
	}
	
	pt.x = x / points.size();
	pt.y = y / points.size();

	return pt;
}

// Converts Points from ROI coordinates to coordinates that work in the original image
cv::Point pointCoordROI2Img(cv::Point p, cv::Mat ROI)
{
	cv::Point ROIoffset = getROIoffset(ROI);

	return cv::Point(p.x + ROIoffset.x, p.y + ROIoffset.y);
}

// Converts lines (vector of two points) from ROI to original image coordinates
std::vector<cv::Point> lineCoordROI2Img(std::vector<cv::Point> ln, cv::Mat ROI)
{
	cv::Point p1 = pointCoordROI2Img(ln[0], ROI);
	cv::Point p2 = pointCoordROI2Img(ln[1], ROI);

	return std::vector<cv::Point>({ p1, p2 });
}

// Get the ROI offset
cv::Point getROIoffset(cv::Mat ROI)
{
	cv::Size ROIsize;
	cv::Point ROIoffset;

	ROI.locateROI(ROIsize, ROIoffset);
	return ROIoffset;
}

// Converts line from polar [radius, theta] form to Cartesian form, crops the line
// at the Mat boundaries, and converts to global coordinates
std::vector<cv::Point> lineCropConvert(cv::Vec2f ln, cv::Mat ROI)
{
	std::vector<cv::Point> ln2 = convertLine(ln);
	cv::Point2f p1, p2;
	intersection(ln2[0], ln2[1], cv::Point(0, 0), cv::Point(ROI.cols, 0), p1);
	intersection(ln2[0], ln2[1], cv::Point(0, ROI.rows), cv::Point(ROI.cols, ROI.rows), p2);
	return lineCoordROI2Img({ p1, p2 }, ROI);
}