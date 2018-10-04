#include "LineProcess.h"



LineProcess::LineProcess(vector<Vec4i> linesOriginal, Point pointR): linesOriginal(linesOriginal),pointR(pointR)
{
}


LineProcess::~LineProcess()
{
}

double LineProcess::slopeCount(Vec4i line)
{
	if (line[1] == line[3])
		return 0;
	if (line[0] == line[2])
		return 1000000;
	double s = abs(((double)line[1] - line[3])) / (line[1] - line[3]);
	if ((line[0] - line[2]) < 0.000001&& (line[0] - line[2]) >= 0)
		return 1000000*s;
	if ((line[0] - line[2]) > -0.000001 && (line[0] - line[2]) < 0)
		return -1000000*s;
	double slope = ((double)line[1] - line[3]) / (line[0] - line[2]);
	if (slope > 1000000)
		return 1000000;
	if (slope < -1000000)
		return -1000000;
	else
		return slope;
	return 0;
}

double LineProcess::angleCount(Vec4i line)
{
	double angle;
	slopeCount(line);
	angle = atan(slopeCount(line));
	return angle;
}

double LineProcess::lengthCount(Vec4i line)
{
	return sqrt((line[0]-line[2])*(line[0] - line[2])+ (line[1] - line[3])*(line[1] - line[3]));
}

double LineProcess::interceptCount(Vec4i line)
{
	return line[1] - slopeCount(line)*(line[0]);
}

double LineProcess::interceptXCount(Vec4i line)
{
	return (line[0] - (double)(line[0]-line[2])/(line[1]-line[3])*(line[1]));
}

double LineProcess::distanceCount(Vec4i line, Point p)
{
	double ans = (slopeCount(line)*p.x - p.y + interceptCount(line)) / sqrt(slopeCount(line)*slopeCount(line)+1);
	return ans;
}

double LineProcess::averageAngle(vector<Vec4i> lines)  //返回平均角度
{
	int i = 0;
	double sum = 0, sumD = 0, angle, lastAngle = 0;
	for (i = 0; i < lines.size(); i++)
	{
		angle = angleCount(lines[i]);
		if (abs(angle - lastAngle) > 3.14159 * 140 / 180)
		{
			if (lastAngle < 0)
				angle = angle - 3.14159;
			else
				angle = angle + 3.14159;
		}
		lastAngle = angle;
		sumD += lengthCount(lines[i]);
		sum += angle*lengthCount(lines[i]);  //加权求和
	}
	return (double)sum/sumD;
}

double LineProcess::averageIntercept(vector<Vec4i> lines)
{
	if (abs(tan(averageAngle(lines)))<1)
	{
		int i = 1;
		double sum = 0, sumD = 0;
		for (i = 0; i < lines.size(); i++)
		{
			sumD += lengthCount(lines[i]);
		}
		for (i = 0; i < lines.size(); i++)
		{
			sum += interceptCount(lines[i])*lengthCount(lines[i]);
		}
		return (double)sum / sumD;
	}
	else
	{
		int i = 1;
		double sum = 0, sumD = 0;
		for (i = 0; i < lines.size(); i++)
		{
			sumD += lengthCount(lines[i]);
		}
		for (i = 0; i < lines.size(); i++)
		{
			sum += interceptXCount(lines[i])*lengthCount(lines[i]);
		}
		return -tan(averageAngle(lines)) * (double)sum / sumD;
	}
}

double LineProcess::averageDistance(vector<Vec4i> lines, Point p)  //平均距离
{
	int i = 1;
	double sum = 0,sumD = 0;
	for (i = 0; i < lines.size(); i++)
	{
		sumD += lengthCount(lines[i]);
		sum += distanceCount(lines[i], p)*lengthCount(lines[i]);
	}
	return (double)sum / sumD;
}

int LineProcess::angleClassify(vector<Vec4i> lines, double angle, vector<Vec4i>& lines1, vector<Vec4i> &lines2)
{
	int i = 0;
	double angleS, lastAngle = 0;
	for (i = 0; i < lines.size(); i++)
	{
		angleS = this->angleCount(lines[i]);
		if (abs(angleS - lastAngle) > 3.14159 * 140 / 180)
		{
			if (lastAngle < 0)
				angleS = angleS - 3.14159;
			else
				angleS = angleS + 3.14159;
		}
		lastAngle = angleS;
		if (angleS < angle)
			lines1.push_back(lines[i]);
		else
			lines2.push_back(lines[i]);
	}
	return 1;
}

int LineProcess::distanceClassify(vector<Vec4i> lines, Point p, double distance, vector<Vec4i> &lines1, vector<Vec4i> &lines2)
{
	int i = 0;
	for (i = 0; i < lines.size(); i++)
	{
		if (this->distanceCount(lines[i], p) <=  distance|| this->distanceCount(lines[i], p) - distance < 5)
			lines1.push_back(lines[i]);
		else 
			lines2.push_back(lines[i]);
	}
	return 1;
}

int LineProcess::distanceFilter(vector<Vec4i>& lines, double distance, Point point)
{
	int i = 0;
	Vec4i line;
	double distanceAverage = 0;
	if (lines.size() <= 2)       //没有办法判断那个被过滤
		return 0;
	for (i = 0; i < lines.size(); i++)
	{
		line = lines[i];
		lines.erase(lines.begin() + i);
		distanceAverage = averageDistance(lines, point);
		if (abs(distanceAverage - distanceCount(line, point)) > distance)
			i--;
		else
			lines.insert(lines.begin() + i, line);
	}
	return 1;
}

int LineProcess::angleFilter(vector<Vec4i>& lines, double angle)
{
	int i = 0;
	Vec4i line;
	double angleAverage = 0;
	if (lines.size() <= 2)
		return 0;
	for (i = 0; i < lines.size(); i++)
	{
		line = lines[i];
		lines.erase(lines.begin() + i);
		angleAverage = averageAngle(lines);
		if (abs(angleAverage - angleCount(line)) > angle)
			i--;
		else
			lines.insert(lines.begin() + i, line);
	}
	return 1;
}

Vec4i LineProcess::projection(Vec4i line, double slope, double intercept)  //计算投影
{
	Vec4i lineP;
	double x1, x2, x;
	x1 = ((double)line[0] + slope * (line[1] - intercept)) / (1 + slope * slope);
	x2 = ((double)line[2] + slope * (line[3] - intercept)) / (1 + slope * slope);
	if (x1 > x2)
	{
		x = x1;
		x1 = x2;
		x2 = x;
	}
	lineP[0] = x1;
	lineP[1] = slope * x1 + intercept;
	lineP[2] = x2;
	lineP[3] = slope * x2 + intercept;
	return lineP;
}

int LineProcess::merge(Vec4i lineP, vector<Vec4i> &line) //把lineP合并到line里面
{
	int i = 0, y = 0;
	int start = 0, typeS = 0;
	int end = 0, typeE = 0;
	if (abs(slopeCount(lineP))<1)
	{
		for (i = 0; i < line.size(); i++)
		{
			if (lineP[0] < line[i][0])
			{
				break;
			}
			if (lineP[0] < line[i][2])
			{
				lineP[0] = line[i][0];
				lineP[1] = line[i][1];
				break;
			}
		}
		if (i >= line.size())
		{
			line.push_back(lineP);
			return 1;
		}
		for (y = i; y < line.size(); y++)
		{
			if (lineP[2] < line[y][0])
			{
				line.insert(line.begin() + y, lineP);
				break;
			}
			if (lineP[2] < line[y][2])
			{
				lineP[2] = line[y][2];
				lineP[3] = line[y][3];
				line.erase(line.begin() + y);
				line.insert(line.begin() + y, lineP);
				break;
			}
			if (y == (line.size() - 1))
			{
				line.erase(line.begin() + y);
				line.insert(line.begin() + y, lineP);
				break;
			}
			line.erase(line.begin() + y);
			y--;
		}
	}
	else
	{
		for (i = 0; i < line.size(); i++)
		{
			if (lineP[1] < line[i][1])
			{
				break;
			}
			if (lineP[1] < line[i][3])
			{
				lineP[1] = line[i][1];
				lineP[0] = line[i][0];
				break;
			}
		}
		if (i >= line.size())
		{
			line.push_back(lineP);
			return 1;
		}
		for (y = i; y < line.size(); y++)
		{
			if (lineP[3] < line[y][1])
			{
				line.insert(line.begin() + y, lineP);
				break;
			}
			if (lineP[3] < line[y][3])
			{
				lineP[2] = line[y][2];
				lineP[3] = line[y][3];
				line.erase(line.begin() + y);
				line.insert(line.begin() + y, lineP);
				break;
			}
			if (y == (line.size() - 1))
			{
				line.erase(line.begin() + y);
				line.insert(line.begin() + y, lineP);
				break;
			}
			line.erase(line.begin() + y);
			y--;
		}
	}
	return 1;
}

int LineProcess::patch(double width, vector<Vec4i>& line)
{
	int i,x,y;
	Vec4i lineP;
	for (i = 0; i < line.size()-1; i++)
	{
		x = line[i][2] - line[i + 1][0];
		y = line[i][3] - line[i + 1][1];
		if (sqrt((double)x * x + y * y) < width)
		{
			lineP[0] = line[i][0];
			lineP[1] = line[i][1];
			lineP[2] = line[i+1][2];
			lineP[3] = line[i+1][3];
			line.erase(line.begin() + i);
			line.erase(line.begin() + i);
			line.insert(line.begin() + i, lineP);
			i--;
		}
	}
	return 0;
}

int LineProcess::getCentre()
{
	if (!line[2].empty())
	{
		double d1, d2, k1, k2, a1, a2;
		if (!line[1].empty())
		{
			k1 = tan((averageAngle(line[0]) + averageAngle(line[1])) / 2);
			d1 = (averageIntercept(line[0]) + averageIntercept(line[1])) / 2;
		}
		else
		{
			k1 = tan(averageAngle(line[0]));
			d1 = averageIntercept(line[0]);
		}
		if (!line[3].empty())
		{
			k2 = tan((averageAngle(line[2]) + averageAngle(line[3])) / 2);
			d2 = (averageIntercept(line[2]) + averageIntercept(line[3])) / 2;
		}
		else
		{
			k2 = tan(averageAngle(line[2]));
			d2 = averageIntercept(line[2]);
		}
		Centre.x = (d2 - d1) / (k1 - k2);
		Centre.y = (k1*Centre.x) + d1;
	}
	else
	{
		Centre.x = 9999;
		Centre.y = 9999;
	}

	return 0;
}

int LineProcess::init()
{
	int x = 0, y = 0;
	vector<Vec4i> linesTmp;
	this->angle[0] = 999;
	this->angle[1] = 999;
	double lastAngle = 0, angle = 0, angleAverage = 0;
	angleAverage = averageAngle(linesOriginal);
	double sum=0;
	for (x = 0; x < linesOriginal.size(); x++)
	{
		angle = angleCount(linesOriginal[x]);
		if (abs(angle - lastAngle) > 3.14159 * 140 / 180)    //把-180°和+180°转换到一起
		{
			if (lastAngle < 0)
				angle = angle - 3.14159;
			else
				angle = angle + 3.14159;
		}
		lastAngle = angle;
		sum += abs(angleAverage - angle);   //和平均角度的总角度差
	}
	if (sum / linesOriginal.size() > 0.3)       //存在两组接近垂直的直线，分两组
	{
		angleClassify(linesOriginal, angleAverage, lines[0][0], lines[1][0]);
		angleFilter(lines[0][0], 1);  //过滤距离相差过大的角度
		angleFilter(lines[1][0], 1);
		distanceFilter(lines[0][0], 120, pointR);  //过滤距离相差过大的直线
		distanceFilter(lines[1][0], 120, pointR);
		this->angle[0] = averageAngle(lines[0][0]);
		this->angle[1] = averageAngle(lines[1][0]);
	}
	else  //分一组
	{
		lines[0][0] = linesOriginal;
		angleFilter(lines[0][0], 1);  //过滤距离相差过大的角度
		distanceFilter(lines[0][0], 120, pointR);
		this->angle[0] = averageAngle(lines[0][0]);
	}

	double intercepta = averageAngle(lines[0][0]);
	double interceptaa = averageAngle(lines[1][0]);

	linesTmp = lines[0][0];          //通过比较距离再各分两组
	lines[0][0].clear();
	double distance = averageDistance(linesTmp, pointR);
	distanceClassify(linesTmp, pointR, distance, lines[0][0], lines[0][1]);  
	//distanceFilter(lines[0][0], 40, pointR);  //过滤距离相差过大的直线
	//distanceFilter(lines[0][1], 40, pointR);

	linesTmp = lines[1][0];
	lines[1][0].clear();
	distance = averageDistance(linesTmp, pointR);
	distanceClassify(linesTmp, pointR, distance, lines[1][0], lines[1][1]);
	//distanceFilter(lines[1][0], 40, pointR);  //过滤距离相差过大的直线
	//distanceFilter(lines[1][1], 40, pointR);

	for (x = 0; x < 4; x++)        //合并成一条直线
	{
		double slope = tan(averageAngle(lines[x / 2][x % 2]));
		double intercept = averageIntercept(lines[x / 2][x % 2]);
		for (y = 0; y < lines[x / 2][x % 2].size(); y++)
		{
			merge(projection(lines[x / 2][x % 2][y], slope, intercept),line[x]);
		}

	}
	//if (!(line[1].empty() || line[0].empty() )) //有可能0空但是后面还有
	//{
		//Point p(line[1][0][0], line[1][0][1]);    //补全
		for (x = 0; x < 4; x++)
		{
			if (line[x].empty())
				continue;
			//patch(0.5*distanceCount(line[0][0], p), line[x]);
			patch(400, line[x]);
		}
	//}

	for (x = 0; x < 4; x++)     //过滤总长过短的直线
	{
		sum = 0;
		for (y = 0; y < line[x].size(); y++)
		{
			sum += lengthCount(line[x][y]);
		}
		if (sum < 50)
			line[x].clear();
	}
	getCentre();
	return 1;
}

