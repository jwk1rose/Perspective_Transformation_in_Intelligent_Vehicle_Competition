#ifndef UNDIS_UNDIS_H
#define UNDIS_UNDIS_H

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <stdio.h>
#include <fstream>
#include "param.h"

using namespace std;
using namespace cv;
struct point
{
    int y;
    int x;
};


static struct point undis_map[120][188];//数组内存放结构体元素 数组内的每一个元素代表畸变图像上点对应的无畸变图像上的点的坐标
void get_undis_map(const char* undis_path,const char* pic_path,bool write_pic,int* x_offset,int* y_offset);
void get_rotation(const Mat& cameraMatrix,const vector<Point2d>& Pi,const vector<Point3d>& Pc,Mat& RMat);
void write_undis_pic(const char* pic_path,const int cols,const int rows,const int x_offset,const int y_offset);
void get_pi(const char* pnp_pic,vector<Point2d>& point);
void onMouse(int event,int x,int y,int flags,void* param);
Mat angle2rotation(int axis,float angle);
Mat roi_tf(int min_x,int min_y,int max_x,int max_y,int rows,int cols,const Mat& H, Mat& pic);
Point2d point_tf(int u,int v,const Mat& cv_H);
void write_maptabel(const char* tabel_path,const char* pic_path,const Mat& H,int x_offset,int y_offset);
Mat tabel_test(const char* sourse_path,int rows,int cols);
#endif//UNDIS_UNDIS_H