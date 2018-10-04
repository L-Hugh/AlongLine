#include <opencv2/opencv.hpp>
#include "./LineProcess/LineProcess.h"
using namespace cv;
using namespace std;


int holeFilter(Mat &img, Mat &dst, Mat &source, double length)
{
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(img, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	int index = 0, x = 0, x1, x2, y1, y2;
	for (index = 0; index < contours.size(); index++)
	{
		x1 = contours[index][0].x;
		x2 = contours[index][0].x;
		y1 = contours[index][0].y;
		y2 = contours[index][0].y;
		for (x = 1; x < contours[index].size(); x++)
		{
			if (x1 > contours[index][x].x)
				x1 = contours[index][x].x;
			if (x2 < contours[index][x].x)
				x2 = contours[index][x].x;
			if (y1 > contours[index][x].y)
				y1 = contours[index][x].y;
			if (y2 < contours[index][x].y)
				y2 = contours[index][x].y;
		}
		line(dst, Point(x1, y1), Point(x1, y2), Scalar(255, 129, 0), 2, LINE_AA);
		line(dst, Point(x1, y2), Point(x2, y2), Scalar(255, 129, 0), 2, LINE_AA);
		line(dst, Point(x2, y2), Point(x2, y1), Scalar(255, 129, 0), 2, LINE_AA);
		line(dst, Point(x2, y1), Point(x1, y1), Scalar(255, 129, 0), 2, LINE_AA);
		if (x1>0&&y1>0&&(abs(x1 - x2) + abs(y1 - y2)) * 2 < 400)
		{
			Mat operate = source(Range(y1, y2), Range(x1, x2));
			operate = Scalar::all(255);
		}
	}

	return 0;
}

int main()
{
	Mat showImg, img;
	Mat dst,dst2,dst3,dst4;
	Mat element;

	VideoCapture cap("task/0.avi");
	int a = 0;
	for (;;)
	{
		cap >> showImg;
		if (showImg.empty())
			break;
		cvtColor(showImg, img, CV_BGR2GRAY);

		dst.create(img.size(), img.type());
		dst2.create(img.size(), img.type());
		dst3.create(img.size(), img.type());
		dst4.create(img.size(), img.type());
		dst = Scalar::all(0);
		dst2 = Scalar::all(0);
		dst3 = Scalar::all(0);
		dst4 = Scalar::all(0);
		//imshow("载入的图片", img);
		threshold(img, dst2, 50, 255, 4);
		//imshow("阈值化后", dst2);
		element = getStructuringElement(MORPH_RECT, Size(45, 45));
		dilate(img, dst, element);
		//imshow("膨胀", dst);
		erode(dst, dst2, element);
		//imshow("再腐蚀", dst2);
		//threshold(dst2, dst3, 115, 255, 1);
		//imshow("阈值化后", dst3);
		//holeFilter(dst3, dst4, dst2, 200);
		//imshow("轮廓", dst4);
		//imshow("轮廓处理后", dst2);
		Canny(dst2, dst, 80, 100, 3);
		imshow("canny", dst);
		//dst = dst2.clone();
		dst2 = Scalar::all(0);
		vector<Vec4i> lines, lines1;
		Point p(400, 300);
		HoughLinesP(dst, lines, 1, CV_PI / 180, 40, 30, 20);
		lines1 = lines;
		for (size_t i = 0; i < lines1.size(); i++)
		{
			Vec4i l = lines1[i];
			line(dst2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
		}
		//imshow("直线检测", dst2);


		LineProcess linesProcess(lines, p);
		linesProcess.init();
		dst2 = Scalar::all(0);
		for (int y = 0; y < 4; y++)
		{
			lines = linesProcess.line[y];
			for (size_t i = 0; i < lines.size(); i++)
			{
				Vec4i l = lines[i];
				line(showImg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 129, 0), 2, LINE_AA);
			}
		}
		if (linesProcess.Centre.x < 9999)
		{
			circle(showImg, linesProcess.Centre, 4, cv::Scalar(0, 0, 255));
			cout << "中心点" << linesProcess.Centre.x << "," << linesProcess.Centre.y << endl;
		}
		imshow("bing", showImg);	
		
		cout << "倾斜：" << linesProcess.angle[0] << ',' << linesProcess.angle[1] << endl;
		if (!linesProcess.line[0].empty())
		{
			cout << "直线1：" << linesProcess.line[0][0][0] << ',' << linesProcess.line[0][0][1] << "  " << linesProcess.line[0][0][2] << ',' << linesProcess.line[0][0][3] << endl;
		}
		if (!linesProcess.line[2].empty())
		{
			cout << "直线2：" << linesProcess.line[2][0][0] << ',' << linesProcess.line[2][0][1] << "  " << linesProcess.line[2][0][2] << ',' << linesProcess.line[2][0][3] << endl;
		}
		a++;
		if(a<167)
			waitKey(1);
		else
			waitKey(1);
	}

}