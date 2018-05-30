#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

using namespace cv;
using namespace std;
float max_step_size = 2.5;
float neighbour_hood = 10;

typedef struct point
{
	float x,y;
}point;

typedef struct parent_node
{
	Point_<float> curr;
	int index;
}parent_node;

typedef struct node
{
	point curr;
	parent_node mommy;
	int my_index;
	float cost;
	vector< parent_node > neighbours;
}node;

float dist(node p1, node p2)
{
	float d = sqrt(pow(p1.curr.x-p2.curr.x,2)+pow(p1.curr.y-p2.curr.y,2));
	return d;
}

float distance(float x1, float y1, float x2, float y2)
{
	float result = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
	return result;
}

int isValid(int i, int j, Mat a)
{
	if(i < 0 || j < 0 || i >= a.cols || j >= a.rows)
		return 0;
	return 1;
}
Mat makeobs(Mat a, int i, int j, int r)
{
	int p,q;
	for(p=i-r;p<i+r;p++)
		for(q=j-sqrt(pow(r,2)-pow(p-i,2));q<j+sqrt(pow(r,2)-pow(p-i,2));q++)
			a.at<uchar>(p,q)=255;

	return a;
}

vector< node > srctree;

Mat bin(Mat a)
{
	int p,q;
	for(p=0;p<a.rows;p++)
		for(q=0;q<a.cols;q++)
			if(a.at<uchar>(p,q)>127) a.at<uchar>(p,q)=255;
			else a.at<uchar>(p,q)=0;

	return a;
}

int main()
{
	//Mat a = imread("Untitled1.png", 0);
	Mat a(500, 500, CV_8UC1, Scalar(0));
	int i = 0,j = 0,k = 0, curr_tree_size = 0, m, n;
	j = 250; k = 250;
	/*for(m = j - 80; m <= j + 80; m++)
	{
		for(n = k - 80; n <= k + 80; n++)
		{
			if(isValid(m,n,a))
			{
				if(distance(m,n,j,k) <= 80 && distance(m,n,j,k) >= 50)
				{
					a.at<uchar>(m,n) = 255;
					//tempobs.i = j;
					//tempobs.j = k;
					//obstacles.push_back(tempobs);
				}
			}
		}
	}*/
	for(m = j - 80; m <= j + 80; m++)
	{
		for(n = k - 10; n <= k + 10; n++)
		{
			if(isValid(m,n,a))
			{
				a.at<uchar>(m,n) = 0;
			}
		}
	}
	int OPP=25;
	int r=10;
	for(i=a.rows/OPP;i<a.rows-a.rows/OPP;i+=a.rows/OPP)
	{
		m = a.rows*2/10 + rand()%(a.rows*6/10);
		n = a.cols*2/10 + rand()%(a.cols*6/10);
		//obstacles.push_back(tempobs);
		a=makeobs(a,m,n,r);
	}
	a=bin(a);
	srand(time(0));
	node source, dest;
	cout<<"Enter the co-ordinates of the source"<<endl;
	cin>>source.curr.x>>source.curr.y;
	int o = source.curr.x;
	int p = source.curr.y;
	if(a.at<uchar>(p,o)>150) {cout<<"source is on the obstacle\n"; return 0;}
	a.at<uchar>(p,o) = 255;
	cout<<"Enter the co-ordinates of the destination1: "<<endl;
	cin>>dest.curr.x>>dest.curr.y;
	o = dest.curr.x;
	p = dest.curr.y;
	if(a.at<uchar>(p,o)>150) {cout<<"destination is on the obstacle\n"; return 0;}
	a.at<uchar>(p,o) = 255;
	source.mommy.curr.x = source.curr.x;
	source.mommy.curr.y = source.curr.y;
	source.mommy.index = 0;
	source.my_index = 0;
	source.cost = 0;
	node qnear, qrand;
	float d = 0, d1 = 0, mindist = 9999999, mincost = 9999999;
	Point_<float> p1,p2,p3,p4;
	parent_node temp1, temp2, temp3, temp4;
	qnear = source;
	srctree.push_back(source);
	curr_tree_size = srctree.size();
	namedWindow("rrt_star",WINDOW_NORMAL);
    imshow("rrt_star",a);
	while(1)
	{
		node qnew;
		if(curr_tree_size == 1)
		{
			qrand.curr.x = rand()%a.cols;
			qrand.curr.y = rand()%a.rows;
			d = dist(qnear, qrand);
			if(d > max_step_size)
			{
				qnew.curr.x = (max_step_size*qrand.curr.x + (d - max_step_size)*qnear.curr.x)/d;
				qnew.curr.y = (max_step_size*qrand.curr.y + (d - max_step_size)*qnear.curr.y)/d;
				qnew.cost = max_step_size;
			}
			else
			{
				qnew = qrand;
				qnew.cost = d;
			}
			qnew.mommy.curr.x = qnear.curr.x;
			qnew.mommy.curr.y = qnear.curr.y;
			qnew.mommy.index = 0;
			qnew.my_index = srctree.size();
			srctree.push_back(qnew);
			curr_tree_size++;
		}
		curr_tree_size = srctree.size();
		here:
		qrand.curr.x = rand()%a.cols;
		qrand.curr.y = rand()%a.rows;
		mindist = 9999999;
		for(i = 0; i < curr_tree_size; i++)
		{
			d = dist(srctree[i], qrand);
			if(d < mindist)
			{
				qnear = srctree[i];
				srctree[i].my_index = i;
				qnear.my_index = i;
				mindist = d;
			}
		}
		d = dist(qnear, qrand);
		for(i = 0; i < d; i++)
		{
			node q;
			q.curr.x = (i*qrand.curr.x + (d-i)*qnear.curr.x)/d;
			q.curr.y = (i*qrand.curr.y + (d-i)*qnear.curr.y)/d;
			int e = q.curr.y;
			int f = q.curr.x;
			if(a.at<uchar>(e,f) > 150) goto here;
		}
		if(d > max_step_size)
		{
			qnew.curr.x = (max_step_size*qrand.curr.x + (d - max_step_size)*qnear.curr.x)/d;
			qnew.curr.y = (max_step_size*qrand.curr.y + (d - max_step_size)*qnear.curr.y)/d;
		}
		else
		{
			qnew = qrand;
		}
		for(i = 0; i < curr_tree_size; i++)
		{
			d1 = dist(qnew, srctree[i]);
			if(d1 <= neighbour_hood)
			{
				//temp1.curr.x = qnew.curr.x; temp1.curr.y = qnew.curr.y;
				temp2.curr.x = srctree[i].curr.x; temp2.curr.y = srctree[i].curr.y;
				temp2.index = i;
				//temp1.index = -1;
				//srctree[i].neighbours.push_back(temp1);
				qnew.neighbours.push_back(temp2);
			}
		}
		mincost = 9999999;
		for(i = 0; i < qnew.neighbours.size(); i++)
		{
			d1 = distance(qnew.curr.x, qnew.curr.y, qnew.neighbours[i].curr.x, qnew.neighbours[i].curr.y);
			if(srctree[qnew.neighbours[i].index].cost + d1 < mincost)
			{
				mincost = srctree[qnew.neighbours[i].index].cost + d1;
				qnew.mommy.curr.x = srctree[qnew.neighbours[i].index].curr.x;
				qnew.mommy.curr.y = srctree[qnew.neighbours[i].index].curr.y;
				qnew.mommy.index = qnew.neighbours[i].index;
				qnew.cost = mincost;
			}
		}
		temp3.curr.x = qnew.curr.x; temp3.curr.y = qnew.curr.y;
		line(a, qnew.mommy.curr, temp3.curr, Scalar(128), 1, 8, 0);
		mincost = 9999999;
		srctree.push_back(qnew);
		curr_tree_size++;
		qnew.my_index = curr_tree_size - 1;
		/*for(i = 0; i < curr_tree_size - 1; i++)
		{
			d1 = dist(qnew, srctree[i]);
			if(d1 < neighbour_hood)
			{
				temp1.curr.x = qnew.curr.x; temp1.curr.y = qnew.curr.y;
				temp1.index = qnew.my_index;
				srctree[i].neighbours.push_back(temp1);
			}
		}*/
		int flag = 0;
		for(i = 0; i < qnew.neighbours.size(); i++)
		{
			d1 = distance(qnew.curr.x, qnew.curr.y, qnew.neighbours[i].curr.x, qnew.neighbours[i].curr.y);
			for(j = 0; j < d1; j++)
			{
				node q;
				q.curr.x = (j*qrand.curr.x + (d1-j)*qnear.curr.x)/d1;
				q.curr.y = (j*qrand.curr.y + (d1-j)*qnear.curr.y)/d1;
				int e = q.curr.y;
				int f = q.curr.x;
				if(a.at<uchar>(e,f) > 150) flag = 1;
			}	
			if(flag == 1) continue;	
			if(srctree[qnew.neighbours[i].index].cost > qnew.cost + d1)
			{
				srctree[qnew.neighbours[i].index].mommy.curr.x = qnew.curr.x;
				srctree[qnew.neighbours[i].index].mommy.curr.y = qnew.curr.y;
				srctree[qnew.neighbours[i].index].mommy.index = qnew.my_index;
				srctree[qnew.neighbours[i].index].cost = qnew.cost + d1;
				temp3.curr.x = srctree[qnew.neighbours[i].index].curr.x;
				temp3.curr.y = srctree[qnew.neighbours[i].index].curr.y;
				line(a, srctree[qnew.neighbours[i].index].mommy.curr, temp3.curr, Scalar(128), 1, 8, 0);
			}

		}

		imshow("rrt_star",a);
		waitKey(2);
		if(dist(qnew, dest) <= 5*max_step_size)
		{
			dest.mommy.curr.x = qnew.curr.x; dest.mommy.curr.y = qnew.curr.y;
			dest.mommy.index = qnew.my_index;
			srctree.push_back(dest);
			dest.my_index = srctree.size() - 1;
			temp3.curr.x = dest.curr.x; temp3.curr.y = dest.curr.y;
			line(a, dest.mommy.curr, temp3.curr, Scalar(128), 1, 8, 0);
			break;
		}

	}
    imshow("rrt_star",a);
    namedWindow("Final", WINDOW_NORMAL);
	Mat b(a.rows, a.cols, CV_8UC1, Scalar(0));
	k = srctree.size() - 1;
	while(k !=0 )
	{
		temp3.curr.x = srctree[k].curr.x;
		temp3.curr.y = srctree[k].curr.y;
		line(b, temp3.curr, srctree[k].mommy.curr, Scalar(255), 1 ,8, 0);
		k = srctree[k].mommy.index;
	}
	imshow("Final", b);
	while(waitKey(0)!=27){}
	return 0;
}
