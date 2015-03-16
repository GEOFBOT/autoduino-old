/**
 * Autoduino Project
 * Distributed under the MIT License.
 * (c) 2015 Geoffrey Mon
 * https://github.com/GEOFBOT/
 * https://github.com/GEOFBOT/autoduinoROS
 *
 * Main program
 *
 *******************************************************************************
 * NOTE: When run on Windows (_WIN32 defined), the program uses a webcam and   *
 * displays the images for testing.  When run on Linux (__linux defined), the  *
 * program reads off an ROS image topic and is in production mode, rather than *
 * testing mode.                                                               *
 *******************************************************************************
 *
 */
#include <string>
#include <vector>
#include <algorithm>
#include <cstring>

#include <opencv2/opencv.hpp>
#ifdef __linux
#include <wiringSerial.h>
#endif

#include "linetools.h"

using namespace std;
using namespace cv;

int arduino = -1;

/////////////////
// Main function
/////////////////
int main(int argc, char* argv[])
{
#ifdef _WIN32
	namedWindow("Camera");
	namedWindow("Sobel pre-thresh");
	namedWindow("Sobel");
	namedWindow("Canny");
#endif

#ifdef __linux
	VideoCapture cam(0);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	arduino = serialOpen("/dev/ttyACM0", 9600);
	if (!arduino) {
		cerr << "Can't open Arduino -_- (fix your hardware!)" << endl;
		return -1;
	}

	VideoWriter vid;
	vid.open("debugvid.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(1920,1080), true);

	if(!cam.isOpened()) {
		cerr << "Can't open camera -_- (fix your hardware!)" << endl;
		return -1;
	}
#endif

	while (true) {
		/////////////////////////////
		// Get camera frame
		// but flush the problematic
		// buffer first :P
		/////////////////////////////
		Mat img;
#ifdef __linux
		cam.grab(); cam.grab(); cam.grab(); cam.grab(); cam.grab();
		cam >> img;
#endif
#ifdef _WIN32
		img = imread("test.jpg");
#endif

		if (img.empty()) {
			cerr << "Frame is empty, wtf?" << endl;
			break;
		}

#ifdef __linux
		imwrite("test-img.jpg", img);
#endif

		//serialPrintf(arduino, "s");

		/////////////
		// Variables
		/////////////
		Mat sobel, sobel_pre, sobel2, sobel_canny, display;
		vector<Mat> sections;
		vector<Mat> sections_disp;
		vector< vector<roadline> > roadlines;
		vector< vector< vector<Point> > > grouped_lines;
		vector< vector<Point> > road_polylines;

		img.copyTo(display);
		cvtColor(img, img, COLOR_BGR2GRAY);

		////////////////////////////////
		// Initial image processing
		// (Gaussian, Sobel, threshold)
		////////////////////////////////
		GaussianBlur(img, img, Size(3, 3), 0);
		Sobel(img, sobel, CV_16S, 1, 0, 3);
		convertScaleAbs(sobel, sobel2);
		sobel2.copyTo(sobel_pre);
		threshold(sobel2, sobel2, 60, 255, THRESH_BINARY_INV);

		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(7, 7), Point(3, 3));
		erode(sobel2, sobel2, element);

		Canny(sobel2, sobel_canny, 50, 150);

		///////////////////////////////////////////////////////
		// CHEVP-like Image Processing
		//
		// Divide the image into ten sections, draw the lines
		// between each section (for viz purposes), and run
		// Hough line detection on each section.
		// Two sections per each of the following divisions:
		//		       10%, 15%, 20%, 25%, 30%
		// See http://www1.i2r.a-star.edu.sg/~ywang/CHEVP.htm
		///////////////////////////////////////////////////////
		{
			double height = 0.10, offset = 0;
			for (int i = 1; i <= 10; ++i) {
				sections_disp.push_back(display(Rect(0, offset * sobel2.rows, sobel2.cols, height / 2 * sobel2.rows)));
				sections.push_back(sobel_canny(Rect(0, offset * sobel2.rows, sobel2.cols, height / 2 * sobel2.rows)));

				line(display, Point(0, (offset + height / 2)*img.rows), Point(img.cols, (offset + height / 2)*img.rows), Scalar(255, 0, 0));

				vector<Vec2f> lines;
				HoughLines(sections[i - 1], lines, 1, radian(1), 25);

				for (int j = 0; j < lines.size(); ++j) {
					displayLine(lines[j], sections_disp[i-1], Scalar(255, 255, 255), 2);
				}

				roadlines.push_back(groupLines(lines, img.cols / 8, sections[i - 1]));

				offset += height / 2;
				height += i % 2 ? 0 : 0.05;
			}
		}

		/////////////////////////////////////////////////
		// Polyline formation
		// Chain together line segments between sections
		/////////////////////////////////////////////////

		// For each section from bottom to top...
		for (int sec1 = sections.size() - 1; sec1 > 0; --sec1) {
			// for each of the lines in that section...
			for (int l1 = 0; l1 < roadlines[sec1].size(); ++l1) {
				//line(display, roadlines[sec1][l1].line[0], roadlines[sec1][l1].line[1], Scalar(0, 0, 0), 2, LINE_AA);

				vector< vector<Point> > group_of_lines;

				vector<Point> line_temp = roadlines[sec1][l1].line;
				if (line_temp[0].y < line_temp[1].y) {
					Point temp = line_temp[1];
					line_temp[1] = line_temp[0];
					line_temp[0] = temp;
				}
				//line(display, line_temp[0], line_temp[1], Scalar(0, 0, 255), 3);
				group_of_lines.push_back(line_temp);

				// loop through the other sections bottom to top and chain together nearby lines.
				for (int sec2 = sections.size() - 2; sec2 >= 0; --sec2) {
					if (sec1 == sec2 || sec1 < sec2) continue;

					double difference = sections[sec1].cols;
					vector<roadline>::iterator line_index;

					Point p1s, p2s;

					vector<roadline>::iterator it;

					for (it = roadlines[sec2].begin(); it < roadlines[sec2].end(); ++it) {
						// Get the x-coordinate of the line in the next section
						Point2f p1;
						vector<Point> line1 = (*it).line;
						intersection(line1[0], line1[1], getROIoffset(sections[sec2]), getROIoffset(sections[sec2]) + Point(sections[sec2].cols, 0), p1);

						// Get the x-coordinate of the line in the current section
						Point2f p2;
						vector<Point> line2 = group_of_lines[group_of_lines.size() - 1];
						intersection(line2[0], line2[1], getROIoffset(sections[sec2 + 1]), getROIoffset(sections[sec2 + 1]) + Point(sections[sec2 + 1].cols, 0), p2);

						//circle(display, p1, 1, Scalar(255, 255, 0), 2);
						//circle(display, p2, 1, Scalar(0, 255, 255), 2);

						if (abs(p1.x - p2.x) < difference) {
							difference = abs(p1.x - p2.x);
							line_index = it;
							p1s = p1;
							p2s = p2;
						}
					}

					// Are they close? else stop the chain
					if (difference < img.cols / 8) {
						vector<Point> converted_line = (*line_index).line;

						if (converted_line[0].y < converted_line[1].y) {
							Point temp = converted_line[1];
							converted_line[1] = converted_line[0];
							converted_line[0] = temp;
						}

						circle(display, converted_line[0], 3, Scalar(255, 255, 0), 2);
						circle(display, converted_line[1], 3, Scalar(0, 255, 255), 2);

						group_of_lines.push_back(converted_line);
						roadlines[sec2].erase(line_index);
					}
					else {
						break;
					}
				}

				grouped_lines.push_back(group_of_lines);
			}
		}

		// Go through each of the groups of line segments and merge points in between line segments to form
		// one continuous polyline! :)
		for (int i = 0; i < grouped_lines.size(); ++i) {
			vector<Point> points;

			for (int j = 0; j < grouped_lines[i].size(); ++j) {
				if (j == 0) { // First point is the bottom point of the first line segment
					points.push_back(grouped_lines[i][j][0]);
					circle(display, points[points.size() - 1], 8, Scalar(0, 0, 0), 3);
				}
				if (j == grouped_lines[i].size() - 1) { // Last point is the top point of the last line segment
					points.push_back(grouped_lines[i][j][1]);
					circle(display, points[points.size() - 1], 8, Scalar(0, 0, 0), 3);
				}
				else { // Middle points are averages of the two nearby points
					points.push_back(averagePoints({ grouped_lines[i][j][1], grouped_lines[i][j + 1][0] }));
					circle(display, grouped_lines[i][j][1], 8, Scalar(0, 255, 0), 3);
					circle(display, grouped_lines[i][j + 1][0], 8, Scalar(0, 0, 255), 3);
				}
			}

			if (points.size() > 2) {
				// Display the finished polyline! (this algorithm took a while for a noob like me)
				polylines(display, points, false, Scalar(0, 0, 0), 3);
				road_polylines.push_back(points);
			}
		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////
		// Loop through road_polylines and get the two biggest lines, which should hopefully be our road lines
		// TODO: Get the two lines on the first frame and track those lines until we lose them
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		if (road_polylines.size() >= 2) {
			cout << road_polylines.size() << endl;
			vector< vector<Point> > temporary_vector_with_unnecessarily_long_name_for_storing_longest_polylines;

			sort(road_polylines.begin(), road_polylines.end(), pointVectorSort);
			cout << "Test: ";
			for (int z = 0; z < road_polylines.size(); ++z) {
				cout << road_polylines[z].size() << ' ';
			}
			cout << endl;

			temporary_vector_with_unnecessarily_long_name_for_storing_longest_polylines.push_back(road_polylines[0]);
			temporary_vector_with_unnecessarily_long_name_for_storing_longest_polylines.push_back(road_polylines[1]);
			road_polylines = temporary_vector_with_unnecessarily_long_name_for_storing_longest_polylines;

			polylines(display, road_polylines[0], false, Scalar(0, 255, 0), 3);
			polylines(display, road_polylines[1], false, Scalar(255, 0, 0), 3);
			cout << road_polylines[0].size() << ' ' << road_polylines[1].size() << endl;

			vector<Point> center_polyline;

			for (int i = 0; i < road_polylines[0].size(); ++i) {
				for (int j = 0; j < road_polylines[1].size(); ++j) {
					if (road_polylines[0][i].y == road_polylines[1][j].y) {
						center_polyline.push_back(averagePoints({ road_polylines[0][i], road_polylines[1][j] }));
						break;
					}
				}
			}

			polylines(display, center_polyline, false, Scalar(128, 255, 128), 3);

			if (center_polyline.size() >= 2) {
			  int lowy =0;
			  Point top;
			  for(int z =0; z<center_polyline.size();++z) {
			    if(center_polyline[z].y > lowy && center_polyline[z].y != img.rows){
			      cout << lowy << ' ' << center_polyline[z].y << endl;
			      lowy =center_polyline[z].y;
			      top = center_polyline[z];
			    }
			  }

//Point bottom = center_polyline[0], top = 
//				if (bottom.y < top.y) {
//					Point temp = bottom;
//					bottom = top;
//					top = temp;
//				}
//				top = bottom;
				Point bottom = Point(img.cols / 2, img.rows);
				circle(display, top, 10, Scalar(0,0,0), -1);
				circle(display, bottom, 10, Scalar(255,255,255), -1);
				//				int location = 0, tilt = 0;
				int angle = int(round(degree(atan2(bottom.y - top.y, bottom.x - top.x))));

				// Angles go in opposite directions in image and on servo
				// also, the line in the image is kind of skewed so reduce the angle
				angle = 90 - (angle - 90) * 1;

#ifdef __linux
				if (angle <= 110 && angle >= 70) serialPrintf(arduino, "a%if", angle);
				else if (angle < 70) serialPrintf(arduino, "a70f");
				else if (angle > 110) serialPrintf(arduino, "a110f");
#endif

				cout << angle << endl;
			}
		}

		if (waitKey(750) >= 0) {
			break;
		}

#ifdef __linux
		serialPrintf(arduino, "sc");
#endif

#ifdef _WIN32
		imshow("Camera", display);
		imshow("Sobel pre-thresh", sobel_pre);
		imshow("Sobel", sobel2);
		imshow("Canny", sobel_canny);
#endif

#ifdef __linux
		vid.write(display);
		imwrite("test-display.jpg", display);
		imwrite("test-sobelt.jpg", sobel2);
		imwrite("test-sobelp.jpg", sobel_pre);
#endif
	}

	//////////////////////
	// Release everything
	//////////////////////
#ifdef __linux
	cam.release();
#endif
	destroyAllWindows();

	return 0;
}
