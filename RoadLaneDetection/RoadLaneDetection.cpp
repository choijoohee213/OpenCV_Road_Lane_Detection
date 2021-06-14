#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

float img_size, img_center;
float left_m, right_m;
Point left_b, right_b;

//Region - of - interest vertices, 관심 영역 범위 계산시 사용 
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07;     // ditto for top edge of trapezoid
float trap_height = 0.4;         // height of the trapezoid expressed as percentage of image height

int main()
{
	Mat img_bgr, img_gray, img_edges, img_hough, img_annotated;
	VideoCapture video("challenge.mp4");

	if (!video.isOpened())
	{
		cout << "동영상 파일을 열 수 없습니다. \n" << endl;
		getchar();
		return -1;
	}

	video.read(img_bgr);
	if (img_bgr.empty()) return -1;

	VideoWriter writer;
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');
	double fps = 25.0;
	string filename = "./live.avi";  //결과 파일

	writer.open(filename, codec, fps, img_bgr.size(), CV_8UC3);
	if (!writer.isOpened()) {
		cout << "출력을 위한 비디오 파일을 열 수 없습니다. \n";
		return -1;
	}

	video.read(img_bgr);
	int cnt = 0;

	while (1) { 
		//1. 원본 영상을 읽어온다.
		if (!video.read(img_bgr))
			break;

	}
	return 0;
}





