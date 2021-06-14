#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

float img_size, img_center;
float left_m, right_m;
Point left_b, right_b;


//Region - of - interest vertices, 관심 영역 범위 계산시 사용 
float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07;     // ditto for top edge of trapezoid
float trap_height = 0.4;         // height of the trapezoid expressed as percentage of image height


Mat limit_region(Mat img_edges, Point* points) {
	/*
		마스크를 적용한다.
		polygon으로 정의된 이미지 영역만 지정한다.
		나머지 영역은 검은색으로 적용.
	*/
	Mat output;
	Mat mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	const Point* pts[1] = { points };
	int npt[] = { 4 };


	//정점으로 정의된 다각형 내부의 색상을 채운다.
	fillPoly(mask, pts, npt, 1, Scalar(255, 255, 255), LINE_8);


	//결과를 얻기 위해 edges 이미지와 mask를 곱한다.
	bitwise_and(img_edges, mask, output);
	return output;
}

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

		//2. GrayScale 영상으로 변환
		cvtColor(img_bgr, img_gray, COLOR_BGR2GRAY);

		//3. 노이즈 제거를 위해 Gaussian Blur를 사용
		GaussianBlur(img_gray, img_gray, Size(3, 3), 0, 0);

		//4. Canny Edge Detection으로 에지를 추출
		Canny(img_gray, img_edges, 50, 150);

		int width = img_gray.cols;
		int height = img_gray.rows;

		Point points[4]{
			Point((width * (1 - trap_bottom_width)) / 2, height),
			Point((width * (1 - trap_top_width)) / 2, height - height * trap_height),
			Point(width - (width * (1 - trap_top_width)) / 2, height - height * trap_height),
			Point(width - (width * (1 - trap_bottom_width)) / 2, height)
		};


		//4. 자동차의 진행방향 바닥에 존재하는 차선만을 검출하기 위한 관심 영역을 지정
		img_edges = limit_region(img_edges, points);
	}
	return 0;
}




