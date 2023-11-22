#include <algorithm>
#include <vector>
#include <stack>
#include <Eigen/Dense>



struct Point 
{
	double x, y;
};

Point p0;

Point nextToTop(std::stack<Point> &S) 
{
	Point p = S.top();
	S.pop();
	Point res = S.top();
	S.push(p);

	return res;
}

int distSq(Point p1, Point p2) 
{
	return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
}

int orientation(Point p, Point q, Point r) 
{
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) return 0;
	return (val > 0)? 1: 2;
}

bool compare(const Point &p1, const Point &p2) 
{
	int o = orientation(p0, p1, p2);
	if (o == 0)
	return distSq(p0, p2) >= distSq(p0, p1);
	return o == 2;
}


vector<Eigen::Vector3d> convexHull(vector<Eigen::Vector3d> points_in, int n) 
{
	vector<Eigen::Vector3d> points_out(100);
	vector<Point> points(100);

	for (int i=0; i<points_in.size(); i++)
	{
		points[i].x = points_in[i].x();
		points[i].y = points_in[i].y();
	}
	int ymin = points[0].y, min = 0;

	for (int i = 1; i < n; i++) 
	{
		int y = points[i].y;

		if ((y < ymin) || (ymin == y && points[i].x < points[min].x))
			
			{
				ymin = points[i].y, min = i;
			}
		
	}

	std::swap(points[0], points[min]);
	p0 = points[0];
	std::sort(points.begin() + 1, points.end(), compare);
	int m = 1;

	for (int i=1; i<n; i++) 
	{
		while (i < n-1 && orientation(p0, points[i], points[i+1]) == 0)
		i++;
		points[m] = points[i];
		m++;
	}

	if (m < 3){return points_out;} 

	std::stack<Point> S;

	S.push(points[0]);
	S.push(points[1]);
	S.push(points[2]);

	for (int i = 3; i < m; i++) 
	{
		while (orientation(nextToTop(S), S.top(), points[i]) != 2)
		S.pop();
		S.push(points[i]);
	}


	while (!S.empty()) 
	{
		Point p = S.top();
		points_out.push_back(Eigen::Vector3d(p.x, p.y, points_in[0].z()));
		//std::cout << "(" << p.x << ", " << p.y <<")" << std::endl;
		S.pop();
	}
	return points_out;
}