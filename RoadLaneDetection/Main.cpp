#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "RoadLaneDetector.h"

int main()
{
	RoadLaneDetector roadLaneDetector;
	Mat img_frame, img_filter, img_edges, img_mask, img_lines, img_result;
	vector<Vec4i> lines;
	vector<vector<Vec4i> > separated_lines;
	vector<Point> lane;
	string dir;
	
	VideoCapture video("input.mp4");  //영상 불러오기

	if (!video.isOpened())
	{
		cout << "동영상 파일을 열 수 없습니다. \n" << endl;
		getchar();
		return -1;
	}

	video.read(img_frame);
	if (img_frame.empty()) return -1;

	VideoWriter writer;
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  //원하는 코덱 선택
	double fps = 25.0;  //프레임
	string filename = "./result.avi";  //결과 파일

	writer.open(filename, codec, fps, img_frame.size(), CV_8UC3);
	if (!writer.isOpened()) {
		cout << "출력을 위한 비디오 파일을 열 수 없습니다. \n";
		return -1;
	}

	video.read(img_frame);
	int cnt = 0;

	while (1) {
		//1. 원본 영상을 읽어온다.
		if (!video.read(img_frame)) break;

		//2. 흰색, 노란색 범위 내에 있는 것만 필터링하여 차선 후보로 저장한다.
		img_filter = roadLaneDetector.filter_colors(img_frame);

		//3. 영상을 GrayScale 으로 변환한다.
		cvtColor(img_filter, img_filter, COLOR_BGR2GRAY);

		//4. Canny Edge Detection으로 에지를 추출. (잡음 제거를 위한 Gaussian 필터링도 포함)
		Canny(img_filter, img_edges, 50, 150);
		
		//5. 자동차의 진행방향 바닥에 존재하는 차선만을 검출하기 위한 관심 영역을 지정
		img_mask = roadLaneDetector.limit_region(img_edges);

		//6. Hough 변환으로 에지에서의 직선 성분을 추출
		lines = roadLaneDetector.houghLines(img_mask);

		if (lines.size() > 0) {
			//7. 추출한 직선성분으로 좌우 차선에 있을 가능성이 있는 직선들만 따로 뽑아서 좌우 각각 직선을 계산한다. 
			// 선형 회귀를 하여 가장 적합한 선을 찾는다.
			separated_lines = roadLaneDetector.separateLine(img_mask, lines);
			lane = roadLaneDetector.regression(separated_lines, img_frame);
			
			//8. 진행 방향 예측
			dir = roadLaneDetector.predictDir();

			//9. 영상에 최종 차선을 선으로 그리고 내부 다각형을 색으로 채운다. 예측 진행 방향 텍스트를 영상에 출력
			img_result = roadLaneDetector.drawLine(img_frame, lane, dir);
		}
		
		//10. 결과를 동영상 파일로 저장. 캡쳐하여 사진 저장
		writer << img_result;
		if (cnt++ == 15) 
			imwrite("img_result.jpg", img_result);  //캡쳐하여 사진 저장

		//11. 결과 영상 출력
		imshow("result", img_result);

		//esc 키 종료
		if (waitKey(1) == 27) 
			break;
	}
	return 0;
}
