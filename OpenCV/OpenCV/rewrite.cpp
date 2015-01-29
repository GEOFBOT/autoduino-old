/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Geoffrey Mon
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
**/

#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#ifdef __linux
#include <wiringSerial.h>
#endif

using namespace std;
using namespace cv;

// Structure that stores lines and their importance
// based on how many nearby lines were averaged to make
// that line.
struct roadline {
	Vec2f line;
	int score;
};

// Used in std::sort() to sort roadline structures in
// order to find the most prominent ones.
struct sortLineByScore {
	bool operator()(const roadline &a, const roadline &b)
	{
		return a.score < b.score;
	}
};

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
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
	Point2f &r)
{
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}


// Converts lines in polar [radius, theta] form (the form
// used by Hough line detection) and displays those lines.
void displayLine(Vec2f ln, Mat output, Scalar color = Scalar(0, 0, 0), int thickness = 1, int mode = LINE_AA)
{
	float r = ln[0], t = ln[1];
	Point pt1, pt2;
	double a = cos(t), o = sin(t);
	double x0 = r * a, y0 = r * o;
	pt1.x = cvRound(x0 + (1000 * (-o)));
	pt1.y = cvRound(y0 + (1000 * (a)));
	pt2.x = cvRound(x0 - (1000 * (-o)));
	pt2.y = cvRound(y0 - (1000 * (a)));
	line(output, pt1, pt2, color, thickness, mode);
}

// Converts lines in polar [radius, theta] form to Cartesian form.
vector<Point> convertLine(Vec2f ln)
{
	float r = ln[0], t = ln[1];
	Point pt1, pt2;
	double a = cos(t), o = sin(t);
	double x0 = r * a, y0 = r * o;
	pt1.x = cvRound(x0 + (1000 * (-o)));
	pt1.y = cvRound(y0 + (1000 * (a)));
	pt2.x = cvRound(x0 - (1000 * (-o)));
	pt2.y = cvRound(y0 - (1000 * (a)));
	vector<Point> v = { pt1, pt2 };
	return v;
}

// main() does what you think it does :P
int main()
{
	cout << "initializing Autoduino project..." << endl;
#ifdef __linux
	int arduino = serialOpen("/dev/ttyACM0", 9600);
	if (!arduino) {
		cerr << "can't open arduino" << endl;
		return -1;
	}
	cout << "arduino opened" << endl;
#endif

	// Canny threshold
	int threshold1 = 15, threshold2 = 50;

#ifdef _WIN32
	namedWindow("Camera feed");
	namedWindow("Combined feed and threshold");
	namedWindow("Canny line detection");
	namedWindow("Denoised image");
	createTrackbar("Threshold1", "Canny line detection", &threshold1, 300);
	createTrackbar("Threshold2", "Canny line detection", &threshold2, 300);
#endif

	VideoCapture camera(0);
	if (!camera.isOpened()) {
		cerr << "can't open camera" << endl;
		return -1;
	}
	cout << "camera opened" << endl;

	bool run = true;

	Mat frame, contrast, processed, combined, smoothed, canny, thresh, display;
	vector<Vec2f> lines;
	vector<Vec2f> lines_threshold;
	vector<vector<Vec2f>> lines2;
	vector<vector<Vec2f>> pre_lines2;
	vector<roadline> result_lines;

	// Range where angles are said to be the same
	int diff_t = 20;

	// Range where radii are said to be the same
	int diff_r = 100;

	while (run) {
		// Check ultrasound distance sensor first!
#ifdef __linux
		serialPrintf(arduino, "csz");
		string input = "";
		char c;
		int dist;
		while (serialDataAvail(arduino)) {
			c = serialGetchar(arduino);
			if (c != '\n') input += c;
			else if (c == '\n') break;
		}
		if(input != "") {
		  dist = stoi(input);
		  if (dist < 20) {
		    continue;
		  }
		}
#endif

		// Clear old variables
		lines.clear();
		lines2.clear();
		result_lines.clear();

		// Get frame
		camera >> frame;
		if (frame.empty()) {
		  cerr << "Empty frame?" << endl;
		  continue;
		}

		// Process image
		frame = frame(Rect(0, cvRound(0.5*frame.rows), frame.cols, cvRound(0.5*frame.rows)));
		frame.copyTo(display);
		bilateralFilter(frame, smoothed, 9, 75, 75);
		cvtColor(smoothed, smoothed, COLOR_BGR2GRAY);
		//equalizeHist(smoothed, contrast);
		smoothed.copyTo(contrast);
		addWeighted(contrast, 0.3, smoothed, 0.7, 0.0, smoothed);
		//dilate(frame, frame, getStructuringElement(MORPH_CROSS, Size(7,7)));
		threshold(smoothed, thresh, 100, 255, THRESH_BINARY);
		Canny(thresh, processed, 50, 225);
		//GaussianBlur(processed, processed, Size(9, 9), 0, 0);
		//addWeighted(processed, 0.05, frame, 0.95, 0.0, combined);
		//threshold(frame, processed, 64, 128, THRESH_BINARY);
		//GaussianBlur(processed, processed, Size(9, 9), 0, 0);
		//addWeighted(processed, 0.1, combined, 0.9, 0.0, combined);
		Canny(smoothed, canny, threshold1, threshold2);
		HoughLines(canny, lines, 1, radian(1), 90);
		HoughLines(processed, lines_threshold, 1, radian(1), 75);
		cvtColor(thresh, thresh, COLOR_GRAY2BGR);
		cvtColor(processed, processed, COLOR_GRAY2BGR);

		// Go through the threshold lines and make sure it is white on either side,
		// signifying that it is a road line and not a stray line
		// This boosts the amount of lines that match up to the correct road lines
		int m = 0;
		for (int i = 0; i < lines_threshold.size(); i++) {
			Point x = convertLine(lines_threshold[i])[0];
			Point y = convertLine(lines_threshold[i])[1];
			clipLine(Size(thresh.cols,thresh.rows), x, y);
			//cout << x << ' ' << y;
			LineIterator it(thresh, x, y);
			double good = 0;
			double total = 0;
			
			for (int j = 0; j < it.count; ++j, ++it) {
				if (nearRange(it.pos().y, 0, thresh.rows)
					&& nearRange(it.pos().x - 15, 0, thresh.cols)
					&& nearRange(it.pos().x + 15, 0, thresh.cols)) {
					++total;
					Point a = Point(it.pos().x - 15, it.pos().y);
					Point b = Point(it.pos().x + 15, it.pos().y);
					if (thresh.at<Vec3b>(a) != Vec3b(0,0,0) && thresh.at<Vec3b>(b) != Vec3b(0,0,0)) {
						circle(thresh, it.pos(), 2, Scalar(0, 0, 255));
						++good;
					}
				}
			}
			if (good / total >= 0.6) {
				lines.push_back(lines_threshold[i]);
				++m;
			}
		}
		//cout << lines_threshold.size() << " " << m << endl;

		// Group lines
		for (int i = 0; i < lines.size(); i++) {
			if (!nearRange(lines[i][1], radian(180 - diff_t), radian(0 + diff_t))
				&& !nearRange(lines[i][1], radian(90 - diff_t), radian(90 + diff_t))) {
				bool match = false;
				if (lines2.size() == 0) {
					lines2.push_back({ lines[i] });
				}
				else {
					bool match = false;
					for (int j = 0; j < lines2.size(); j++) {
						if (nearValue(lines[i][1], lines2[j][0][1], radian(diff_t))) {
							lines2[j].push_back(lines[i]);
							match = true;
							break;
						}
					}
					if (!match) {
						lines2.push_back({lines[i]});
					}
				}
				displayLine(lines[i], display, Scalar(255, 255, 255));
			}
		}

		for (int i = 0; i < lines2.size(); i++) {
			displayLine(lines2[i][0], display, Scalar(0, 255, 255));
			roadline rl;
			float total_t = 0;
			float total_r = 0;
			for (int j = 0; j < lines2[i].size(); j++) {
				total_r += lines2[i][j][0];
				total_t += lines2[i][j][1];
			}
			rl.line = { float(total_r / lines2[i].size()), float(total_t / lines2[i].size()) };
			rl.score = lines2[i].size();
			//cout << rl.score << endl;
			result_lines.push_back(rl);
		}
		sort(result_lines.begin(), result_lines.end(), sortLineByScore());

		// Draw lines
		for (int i = 2; i < result_lines.size(); i++) {
			displayLine(result_lines[i].line, display, Scalar(0, 0, 0), 3);
		}

		// Drive!
		bool drive = false;
		vector<Point> centerline;
		if (result_lines.size() > 1) {
			// Top two lines should be the road lines
			roadline rl1 = result_lines[0];
			roadline rl2 = result_lines[1];
			vector<Point> rl1_line = convertLine(rl1.line);
			vector<Point> rl2_line = convertLine(rl2.line);
			Point2f intersect, bottom1, bottom2;
			intersection(rl1_line[0], rl1_line[1], rl2_line[0], rl2_line[1], intersect);
			intersection(rl1_line[0], rl1_line[1], Point(0, frame.rows), Point(frame.cols, frame.rows), bottom1);
			intersection(rl2_line[0], rl2_line[1], Point(0, frame.rows), Point(frame.cols, frame.rows), bottom2);
			Point center(cvRound((bottom1.x + bottom2.x) / 2), frame.rows);
			line(display, center, intersect, Scalar(255, 0, 255), 3);
			centerline.push_back(center);
			centerline.push_back(intersect);
			drive = true;
		}
		else if (result_lines.size() == 1) {
			vector<Point> points = convertLine(result_lines[0].line);
			centerline = points;
			drive = true;
		}
		cout << "test" << endl;
		if (drive) {
			cout << "driving" << endl;
			Point a = centerline[0];
			Point b = centerline[1];
			if (a.y < b.y) {
				Point temp = a;
				a = b;
				b = temp;
			}
			double angle = atan2(a.y - b.y, a.x - b.x);
			angle = degree(angle);

			int ang = 0; // -1 = less than 80 degrees, 0 = between 80 to 100 degrees, 1 = more than 100 degrees
			int placement = 0; // -1 = left third of frame, 0 = middle third of frame, 1 = right third of frame
			
			if (angle <= 80) ang = -1;
			else if (angle >= 100) ang = 1;

			if (b.x < cvRound(frame.cols / 3)) placement = -1;
			else if (b.x > cvRound(2 * frame.cols / 3)) placement = 1;

			if ((placement == 0 && ang == 0) || (placement == 1 && ang == -1) || (placement == -1 && ang == 1)) {
				cout << "Go straight" << endl;
#ifdef __linux
				serialPrintf(arduino, "cf");
#endif
			}
			else if ((placement == -1 && ang == 0) || (placement == 0 && ang == -1) || (placement == -1 && ang == -1)) {
				cout << "Go left" << endl;
#ifdef __linux
				serialPrintf(arduino, "af");
#endif
			}
			else if ((placement == 1 && ang == 0) || (placement == 0 && ang == 1) || (placement == 1 && ang == 1)) {
				cout << "Go right" << endl;
#ifdef __linux
				serialPrintf(arduino, "df");
#endif
			}
		}

		// Display frames
#ifdef _WIN32
		imshow("Camera feed", display);
		imshow("Combined feed and threshold", thresh);
		imshow("Canny line detection", canny);
		imshow("Denoised image", smoothed);
#endif

#ifdef __linux
		cout << "mew" << endl;
		std::vector<uchar> buff;
		imencode(".jpg", display, buff);
		cout << reinterpret_cast<const char*>(buff.data()) << endl;
		ofstream img("/home/geoffreymon/mjpg/out.jpg");//, ofstream::binary);
		//img << "Meow";
		img.write(reinterpret_cast<const char*>(buff.data()), buff.size());
		img.close();
#endif

		if (waitKey(750) >= 0)
			break;
	}

	camera.release();

#ifdef _WIN32
	destroyAllWindows();
#endif

#ifdef __linux
	serialClose(arduino);
#endif

	return 0;
}
