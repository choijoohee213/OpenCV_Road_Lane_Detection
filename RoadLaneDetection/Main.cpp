#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "RoadLaneDetector.h"
#include "RoadLaneDetector.cpp"

int main()
{
	RoadLaneDetector roadLaneDetector;
	Mat img_frame, img_filter, img_denoise, img_edges, img_mask, img_lines, img_result;
	vector<Vec4i> lines;
	vector<vector<Vec4i> > separated_lines;
	vector<Point> lane;
	string dir;
	
	VideoCapture video("input.mp4");  //���� �ҷ�����

	if (!video.isOpened())
	{
		cout << "������ ������ �� �� �����ϴ�. \n" << endl;
		getchar();
		return -1;
	}

	video.read(img_frame);
	if (img_frame.empty()) return -1;

	VideoWriter writer;
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  //���ϴ� �ڵ� ����
	double fps = 25.0;  //������
	string filename = "./result.avi";  //��� ����

	writer.open(filename, codec, fps, img_frame.size(), CV_8UC3);
	if (!writer.isOpened()) {
		cout << "����� ���� ���� ������ �� �� �����ϴ�. \n";
		return -1;
	}

	video.read(img_frame);
	int cnt = 0;

	while (1) {
		//1. ���� ������ �о�´�.
		if (!video.read(img_frame)) break;

		//2. ���, ����� ���� ���� �ִ� �͸� ���� �ĺ��� �����Ѵ�.
		img_filter = roadLaneDetector.filter_colors(img_frame);

		//3. GrayScale �������� ��ȯ�ϰ� ������ ���Ÿ� ���� Gaussian Blur�� ���
		cvtColor(img_filter, img_filter, COLOR_BGR2GRAY);
		GaussianBlur(img_filter, img_denoise, Size(3, 3), 0, 0);

		//4. Canny Edge Detection���� ������ ����
		Canny(img_denoise, img_edges, 50, 150);
		
		//5. �ڵ����� ������� �ٴڿ� �����ϴ� �������� �����ϱ� ���� ���� ������ ����
		img_mask = roadLaneDetector.limit_region(img_edges);

		//6. Hough ��ȯ���� ���������� ���� ������ ����
		lines = roadLaneDetector.houghLines(img_mask);

		if (lines.size() > 0) {
			//7. ������ ������������ �¿� ������ ���� ���ɼ��� �ִ� �����鸸 ���� �̾Ƽ� �¿� ���� ������ ����Ѵ�. 
			// ���� ȸ�͸� �Ͽ� ���� ������ ���� ã�´�.
			separated_lines = roadLaneDetector.separateLine(img_edges, lines);
			lane = roadLaneDetector.regression(separated_lines, img_frame);
			
			//8. ���� ���� ����
			dir = roadLaneDetector.predictDir();

			//9. ���� ������ ���� ������ ������ �׸��� ���� ���� ���ڿ��� ���� ���
			img_result = roadLaneDetector.drawLine(img_frame, lane, dir);
		}
		
		writer << img_result;  //����� ������ ���Ϸ� ���

		if (cnt++ == 10) 
			imwrite("img_result.jpg", img_result);  //ĸ���Ͽ� ���� ����

		//��� ���� ���
		imshow("result", img_result);

		//esc Ű ����
		if (waitKey(1) == 27) 
			break;
	}
	return 0;
}
