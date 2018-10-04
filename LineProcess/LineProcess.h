#pragma once
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class LineProcess
{
public:
	vector<Vec4i> linesOriginal;
	vector<Vec4i> lines[2][2]; 
	vector<Vec4i> line[4];  //合并后的4*2条直线
	double slope[2];
	double intercept[2];
	double angle[2];
	Point Centre;
	Point pointR;  //分类参照点
	int type;

public:
	LineProcess(vector<Vec4i> linesOriginal, Point pointR);
	~LineProcess();
	double slopeCount(Vec4i line); //斜率
	double angleCount(Vec4i line); //倾斜角
	double lengthCount(Vec4i line); //长度
	double interceptCount(Vec4i line); //截距
	double interceptXCount(Vec4i line); //x截距
	double distanceCount(Vec4i line, Point p); //line到一个点的距离
	double averageAngle(vector<Vec4i> lines); //平均倾斜角
	double averageIntercept(vector<Vec4i> lines); //平均截距
	double averageDistance(vector<Vec4i> lines, Point p); //平均距离
	int angleClassify(vector<Vec4i> lines, double angle, vector<Vec4i> &lines1, vector<Vec4i> &lines2); //按斜率分两类
	int distanceClassify(vector<Vec4i> lines, Point p, double distance, vector<Vec4i> &lines1, vector<Vec4i> &lines2); //再按截距分四类
	int distanceFilter(vector<Vec4i> &lines, double distance, Point point); //过滤掉远离平均距离的直线
	int angleFilter(vector<Vec4i> &lines, double angle); //过滤掉远离平均角的直线
	Vec4i projection(Vec4i line, double slope, double intercept); // 计算投影线
	int merge(Vec4i lineP, vector<Vec4i> &line); //把小直线合并到一条直线
	int patch(double width, vector<Vec4i> &line);  //补全这条直线
	int getCentre();
	int init();
};

