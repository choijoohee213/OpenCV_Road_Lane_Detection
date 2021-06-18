#pragma once
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class RoadLaneDetector
{
private:
	double img_size, img_center;
	double left_m, right_m;
	Point left_b, right_b;
	bool left_detect = false, right_detect = false;

	//관심 영역 범위 계산시 사용 
	double poly_bottom_width = 0.85;  //사다리꼴 아래쪽 가장자리 너비 계산을 위한 백분율
	double poly_top_width = 0.07;     //사다리꼴 위쪽 가장자리 너비 계산을 위한 백분율
	double poly_height = 0.4;         //사다리꼴 높이 계산을 위한 백분율


public:
	Mat filter_colors(Mat img_frame);
	Mat limit_region(Mat img_edges); 
	vector<Vec4i> houghLines(Mat img_mask);
	vector<vector<Vec4i> > separateLine(Mat img_edges, vector<Vec4i> lines); 
	vector<Point> regression(vector<vector<Vec4i> > separated_lines, Mat img_input);
	string predictDir();
	Mat drawLine(Mat img_input, vector<Point> lane, string dir);
};

