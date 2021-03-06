// Fragments from http://hxr99.blogspot.com/2011/12/opencv-examples-camera-capture.html
#include <opencv2/opencv.hpp>
#ifdef __arm__
#include <wiringSerial.h>
#endif

#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

struct roadline {
	Vec2f line;
	int score;
};

struct sortLineByScore {
	bool operator()(const roadline &a, const roadline &b)
	{
		return a.score < b.score;
	}
};

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

int main(int argc, const char** argv)
{
#ifdef __arm__
	int arduino = serialOpen("/dev/ttyACM0", 9600);
	if (!arduino) {
		cerr << "can't open arduino" << endl;
		return -1;
	}
#endif

	VideoCapture capture(0);
	VideoWriter outStream;
	Mat frame, ROI, image, edges, edges2, lines;
	vector<Vec2f> hough;
	vector<vector<Vec2f>> linesVec;
	vector<roadline> roadLines;
	Point prevCenter = { 0, 0 };
	int threshold1 = 50;
	int threshold2 = 225;
	double offsetx = 0, offsety = 0.4;
	int dir, tilt; // -1 = left, 1 = right, 0 = middle
	int move[2] = { 0, 0 }; // [0] drive; [1] turn
	int distance = 0;
	bool correcting = false;
	int tries = 0;

	Point p1, p2, p3, p4;
	Point prevl = { 0, -1 }, prevr = { 0, -1 };
	roadline left, right;
	Vec2f prevleft = { 0, 0 }, prevright = { 0, 0 };

#ifdef _WIN32
	namedWindow("result");
	namedWindow("edge");
	namedWindow("line");
#endif

	if (!capture.isOpened()) {}//cout << "No camera detected" << endl;
	else {
	  //cout << "In capture ..." << endl;
		bool run = true;
		while (run) {
			linesVec.clear();
#ifdef __arm__
			serialPrintf(arduino, "cs");
#endif
			waitKey(500);
			capture >> frame;
			if (frame.empty()) {
				run = false;
				break;
			}
			else {
#ifdef __arm__
				serialPrintf(arduino, "z");
				string l = "";
				char c;
				while (true) {
					c = serialGetchar(arduino);
					if (c != '\n') l += c;
					else if (c == '\n') break;
				}
				//cout << l << endl;
				distance = stoi(l);
#endif

				if(!outStream.isOpened()) {
				  int codec = VideoWriter::fourcc('M','J','P','G');
				  //cout << "Frame size: " << frame.cols << " " << frame.rows << endl;
				  //cout << "Opening output stream..." << codec << "  " <<
				  outStream.open("out.mjpg", codec, 2, Size(frame.cols,frame.rows), true);
				}

				//cvtColor(frame, ROI, COLOR_BGR2GRAY);
				ROI = frame(Rect(cvRound(frame.cols * offsetx), cvRound(frame.rows * offsety), cvRound(frame.cols * (1 - 2 * offsetx)), cvRound(frame.rows * (1-offsety))));
				cvtColor(ROI, ROI, COLOR_BGR2GRAY);
				//equalizeHist(ROI, ROI);
				//createCLAHE()->apply(ROI, ROI);
				threshold(ROI, ROI, 100, 255, THRESH_BINARY);
				frame.copyTo(lines);
				//imwrite("lines0.jpg", lines);
				Canny(ROI, edges, threshold1, threshold2);
				HoughLines(edges, hough, 1, CV_PI / 180, 30);
				cvtColor(edges, edges2, COLOR_GRAY2BGR);
				for (int i = 0; i < hough.size(); i++) {
					float r = hough[i][0], t = hough[i][1];
					Point pt1, pt2;
					double a = cos(t), o = sin(t);
					double x0 = r * a, y0 = r * o;
					pt1.x = cvRound(x0 + (1000 * (-o)) + frame.cols * offsetx);
					pt1.y = cvRound(y0 + (1000 * (a)) + frame.rows * offsety);
					pt2.x = cvRound(x0 - (1000 * (-o)) + frame.cols * offsetx);
					pt2.y = cvRound(y0 - (1000 * (a)) + frame.rows * offsety);
					line(lines, pt1, pt2, Scalar(255, 255, 0), 1, LINE_AA);
					bool match = false;
					for (int j = 0; j < linesVec.size(); j++) {
						float r2 = linesVec[j][0][0];
						float t2 = linesVec[j][0][1];
						if ((t2 - (18 * CV_PI / 180) < t) && (t < t2 + (18 * CV_PI / 180)) ) {//&& (r2 - 10 < r) && (r < r2 + 10)) {
							linesVec[j].push_back({ r, t });
							match = true;
							break;
						}
					}
					if (!match) {
						linesVec.push_back({ { r, t } });
						line(lines, pt1, pt2, Scalar(0, 255, 255), 1, LINE_AA);
					}
				}
				for (int i = 0; i < linesVec.size(); i++) {
					vector<double> sum = { 0, 0 };
					vector<double> avg = { 0, 0 };
					for (int j = 0; j < linesVec[i].size(); j++) {
						sum[0] += linesVec[i][j][0];
						sum[1] += linesVec[i][j][1];
					}
					avg[0] = sum[0] / linesVec[i].size();
					avg[1] = sum[1] / linesVec[i].size();
					double r = avg[0], t = avg[1];
					if ((roadLines.size() == 0) && !(((t < (10 * CV_PI / 180)) && (t > 170 * CV_PI / 180)) || ((t > 80 * CV_PI / 180) && (t < 100 * CV_PI / 180)))) {
						roadline l;
						l.line = { float(r), float(t) };
						l.score = 1;
						roadLines.push_back(l);
					}
					for (int j = 0; j < roadLines.size(); j++) {
						if (t < roadLines[j].line[1] + (10 * CV_PI / 180) && t > roadLines[j].line[1] - (10 * CV_PI / 180)) {
							++roadLines[j].score;
						}
						else if (!(((t < (10 * CV_PI / 180)) && (t > 170 * CV_PI / 180)) || ((t > 80 * CV_PI / 180) && (t < 100 * CV_PI / 180)))) {
							roadline l;
							l.line = { float(r), float(t) };
							l.score = 1;
							roadLines.push_back(l);
						}
					}
				}
				sort(roadLines.begin(), roadLines.end(), sortLineByScore());
				if (roadLines.size() >= 2) {
					tries = 0;
					correcting = false;
					//cout << "not correcting" << endl;
					left = roadLines[0];
					right = roadLines[1];
					roadline temp;
					float r = left.line[0], t = left.line[1];
					Point p1, p2, p3, p4;
					double a = cos(t), o = sin(t);
					double x0 = r * a, y0 = r * o;
					int x1 = cvRound(x0 + (1000 * (-o)));

					r = right.line[0];
					t = right.line[1];
					a = cos(t);
					o = sin(t);
					x0 = r * a;
					y0 = r * o;
					int x2 = cvRound(x0 + (1000 * (-o)));

					if (x2 < x1) {
						temp = left;
						left = right;
						right = temp;
					}
					/*for (int j = 0; j < roadLines.size(); j++) {
					if (left.score == 0 && (roadLines[j].line[1] < 40 * CV_PI / 180 && roadLines[j].line[1] > 10 * CV_PI / 180)) {
					left = roadLines[j];
					}
					else if (right.score == 0 && (roadLines[j].line[1] < 160 * CV_PI / 180 && roadLines[j].line[1] > 130 * CV_PI / 180)) {
					right = roadLines[j];
					}
					}*/
					roadLines.clear();
					/*for (int j = 0; j < roadLines.size(); j++) {
					float r = roadLines[j].line[0], t = roadLines[j].line[1];
					Point p1, p2;
					double a = cos(t), o = sin(t);
					double x0 = r * a, y0 = r * o;
					p1.x = cvRound(x0 + (1000 * (-o)));
					p1.y = cvRound(y0 + (1000 * (a)) + frame.rows * offsety);
					p2.x = cvRound(x0 - (1000 * (-o)));
					p2.y = cvRound(y0 - (1000 * (a)) + frame.rows * offsety);
					line(lines, p1, p2, Scalar(0, 0, 255), 2, LINE_AA);
					}*/
					r = left.line[0], t = left.line[1];
					a = cos(t), o = sin(t);
					x0 = r * a;
					y0 = r * o;
					p1.x = cvRound(x0 + (1000 * (-o)) + frame.cols * offsetx);
					p1.y = cvRound(y0 + (1000 * (a)) + frame.rows * offsety);
					p2.x = cvRound(x0 - (1000 * (-o)) + frame.cols * offsetx);
					p2.y = cvRound(y0 - (1000 * (a)) + frame.rows * offsety);
					line(lines, p1, p2, Scalar(0, 0, 255), 2, LINE_AA);

					r = right.line[0];
					t = right.line[1];
					a = cos(t);
					o = sin(t);
					x0 = r * a;
					y0 = r * o;
					p3.x = cvRound(x0 + (1000 * (-o)) + frame.cols * offsetx);
					p3.y = cvRound(y0 + (1000 * (a)) + frame.rows * offsety);
					p4.x = cvRound(x0 - (1000 * (-o)) + frame.cols * offsetx);
					p4.y = cvRound(y0 - (1000 * (a)) + frame.rows * offsety);
					line(lines, p3, p4, Scalar(0, 255, 0), 2, LINE_AA);

					Point2f i1, il, ir;
					intersection(p1, p2, p3, p4, i1);
					intersection(p1, p2, Point(0, frame.rows), Point(frame.cols, frame.rows), il);
					intersection(p3, p4, Point(0, frame.rows), Point(frame.cols, frame.rows), ir);
					Point center = (il + ir) / 2;
					line(lines, center, i1, Scalar(255, 0, 0), 1, LINE_AA);
					if ((prevl.y == -1 && prevr.y == -1) || ((il.x > prevl.x - 0.2 * frame.cols || il.x < prevl.x + 0.2 * frame.cols) && (ir.x > prevr.x - 0.2 * frame.cols || ir.x < prevr.x + 0.2 * frame.cols))) {
						prevl = il;
						prevr = ir;
					}
					else break;

					if (prevCenter.x != 0 && prevCenter.y != 0) {
						double d = sqrt((center.x - prevCenter.x) ^ 2 + (center.y - prevCenter.y) ^ 2);
						if (d > 50) correcting = true;
					}

					if (center.x < frame.rows * 0.4) {
						dir = -1;
					}
					else if (center.x > frame.rows * 0.6) {
						dir = 1;
					}
					else {
						dir = 0;
					}
					if (i1.x > center.x + frame.rows * 0.1) {
						tilt = 1;
					}
					else if (i1.x < center.x - frame.rows * 0.1) {
						tilt = -1;
					}
					else {
						tilt = 0;
					}
#ifdef __arm__
					if (distance < 20) {
						serialPrintf(arduino, "cs");
						continue;
					}
#endif
					if ((dir == 0 && tilt == 0) || (dir == 1 && tilt == -1) || (dir == -1 && tilt == 1)) {
					  	cout << "Go straight" << endl;
						move[0] = 1;
						move[1] = 0;
#ifdef __arm__
						serialPrintf(arduino, "cf");
#endif
					}
					else if ((dir == -1 && tilt == 0) || (dir == 0 && tilt == -1) || (dir == -1 && tilt == -1)) {
					  	cout << "Go left" << endl;
						move[0] = 1;
						move[1] = -1;
#ifdef __arm__
						serialPrintf(arduino, "af");
#endif
					}
					else if ((dir == 1 && tilt == 0) || (dir == 0 && tilt == 1) || (dir == 1 && tilt == 1)) {
					  	cout << "Go right" << endl;
						move[0] = 1;
						move[1] = 1;
#ifdef __arm__
						serialPrintf(arduino, "df");
#endif
					}
					prevCenter = center;
				}
				else if (!correcting && tries < 20) {
					++tries;
				}
				else if (!correcting && tries >= 20) {
					correcting = true;
					//cout << "correcting" << endl;
#ifdef __arm__
					if (distance < 20) {
						serialPrintf(arduino, "cs");
						continue;
					}
#endif
#ifdef __arm__
					serialPrintf(arduino, "r");
#endif
					if (move[1] == -1) {
#ifdef __arm__
					  //						serialPrintf(arduino, "d");
#endif
					}
					else if (move[1] == 1) {
#ifdef __arm__
					  //	serialPrintf(arduino, "a");
#endif
					}
				}


#ifdef _WIN32
				imshow("result", ROI);
				imshow("edge", edges2);
				imshow("line", lines);
#endif

				imwrite("/home/geoffreymon/www/lines.png", lines);
				imwrite("/home/geoffreymon/mjpg/out.jpg", lines);
				outStream << lines;
				//cout << lines;
			}

			if (waitKey(75) >= 0)
				break;
		}

		capture.release();

#ifdef _WIN32
		destroyWindow("result");
		destroyWindow("edge");
		destroyWindow("line");
#endif

#ifdef __arm__
		serialClose(arduino);
#endif

		return 0;
	}
}
