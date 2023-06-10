#include "undis.h"
/*!
 * @note 这个函数作用是将matlab生成的去畸变映射保存到 undis_map[120][188]这个结构体数组里 再将图片保存
 * @param undis_path 存放畸变图像到去畸变图像映射的txt文件地址 由matlab获得
 * @param pic_path 畸变图像文件夹地址
 * @param x_offset x的偏移 matlab得到的映射表存在负值
 * @param y_offset y的偏移 matlab得到的映射表存在负值
 * @param write_pic 是否输出去畸变图
 */
void get_undis_map(const char* undis_path,const char* pic_path,bool write_pic,int* x_offset,int* y_offset)
{
    float row[4];
    int min_x=1000,min_y=1000,max_x=-1000,max_y=-1000;//原图去畸变之后得到的最小（大）x,y 用于生成去畸变图片
    FILE *fp = NULL;
    char buff[255];
    fp = fopen(undis_path, "r");
    int line=0;
    while(line<4*120*188)
    {
        fscanf(fp, "%s", buff);
        float f=atof(buff);
        row[line%4]=f;
        line++;
        if(line%4==0)
        {
            int x=lround(row[2]),y=lround(row[3]);
            point p = {y,x};
            if(x<min_x)min_x=x;
            if(x>max_x)max_x=x;
            if(y<min_y)min_y=y;
            if(y>max_y)max_y=y;
            undis_map[int(row[1])-1][int(row[0])-1] = p;
        }
    }
    fclose(fp);
    cout<<" get_undis_map success!"<<endl;
    *x_offset=min_x;
    *y_offset=min_y;
    if(write_pic)
        write_undis_pic(pic_path,max_x-min_x,max_y-min_y,min_x,min_y);
}

/*!
 *
 * @param pic_path 畸变图片文件夹的地址
 * @param cols 去畸变图片的列数
 * @param rows 去畸变图片的行数
 * @param x_offset 映射关系的平移 (matlab 获得的存在负数)
 * @param y_offset 如上
 * @note 根据 undis_map 描述的映射关系 将畸变图片去畸变之后保存起来 最好做一下无畸变图片中间黑色像素点的插值 无畸变图片模糊的话标定得到内参矩阵可能也不是特别精准
 */

void write_undis_pic(const char* pic_path,const int cols,const int rows,const int x_offset,const int y_offset)
{
    for (auto& i : filesystem::directory_iterator(pic_path))
    {
        Mat img=imread(i.path().string());
        cvtColor(img,img,COLOR_BGR2GRAY);
        Mat img_undis=Mat::zeros(rows,cols,CV_8UC1);
        for(int y=0;y<img.rows;y++)
            for(int x=0;x<img.cols;x++)
                img_undis.at<uchar>(undis_map[y][x].y-y_offset,undis_map[y][x].x-x_offset)=img.at<uchar>(y,x);
        imwrite(string(i.path().string().insert(16,"_undis")),img_undis);
        cout<<string(i.path().string().insert(15,"_undis"))<<"is writen"<<endl;
    }
}

/*!
 *
 * @param cameraMatrix 相机的内参矩阵
 * @param Pi 在图像上点的位置
 * @param Pc 在相机坐标系下点的位置
 * @param RMat 旋转矩阵
 */

void get_rotation(const Mat& cameraMatrix,const vector<Point2d>& Pi,const vector<Point3d>& Pc,Mat& RMat)
{
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    Mat rVec = Mat::zeros(3, 1, CV_64FC1);
    Mat tVec = Mat::zeros(3, 1, CV_64FC1);
    solvePnP(Pc,Pi,cameraMatrix,distCoeffs,rVec,tVec,false,SOLVEPNP_ITERATIVE);
    Rodrigues(rVec, RMat);
}
/*!
 *
 * @param pnp_pic 获取点在图片上的地址
 * @param point pi 图像上的2d点
 */
void get_pi(const char* pnp_pic,vector<Point2d>& point)
{
    Mat img=imread(pnp_pic);
    imshow("img",img);
    setMouseCallback("img",onMouse,reinterpret_cast<void*>(& point));//鼠标响应函数
    waitKey(0);
}
/*!
 *
 * @param event 表示不同的鼠标状态
 * @param x 鼠标在图片上的位置的横坐标
 * @param y 鼠标在图片上的位置的纵坐标
 * @param flags 没用到
 * @param param 用户数据 传入时要转成 void* 使用时再转回去
 * @note setMouseCallback 的回调函数 这里实现的功能是将鼠标左击的点保存下来
 */
void onMouse(int event,int x,int y,int flags,void* param)
{
    auto* point = reinterpret_cast<vector<Point2d>*>(param);
    if(event==EVENT_LBUTTONDOWN)
    {
        point->push_back(Point2d(x,y));
        cout<<"(x,y) == ("<<x<<","<<y<<")"<<endl;
    }
}
/*!
 *
 *
 * @param axis 0 1 2 分别代表生成绕 x y z 轴旋转angle角度的旋转矩阵
 * @param angle 角度制
 * @return 旋转矩阵 
 */
Mat angle2rotation(int axis,float angle)
{
    cv::Mat R;
    angle=angle/180.0*M_PI;
    if(axis==0)
        R = (cv::Mat_<double> ( 3,3 ) <<1.0,0.0,0.0,0.0,cos(angle),-sin(angle),0.0,sin(angle),cos(angle));
    else if(axis==1)
        R = (cv::Mat_<double> ( 3,3 ) <<cos(angle),0.0,sin(angle),0.0,1.0,0.0,-sin(angle),0.0,cos(angle));
    else if(axis==2)
        R = (cv::Mat_<double> ( 3,3 ) <<cos(angle),-sin(angle),0.0,sin(angle),cos(angle),0.0,0.0,0.0,1.0);
    return R;
}

/*!
 *
 * @param min_x 在去畸变图像上选取的最小x
 * @param min_y 在去畸变图像上选取的最小y
 * @param max_x 在去畸变图像上选取的最大x
 * @param max_y 在去畸变图像上选取的最大y
 * @param rows 俯视图的高
 * @param cols 俯视图的宽
 * @param H 单应矩阵
 * @param pic 去畸变的斜视图像
 * @return 生成的俯视图像
 */
Mat roi_tf(int min_x,int min_y,int max_x,int max_y,int rows,int cols,const Mat& H, Mat& pic)
{
    if(pic.channels()==3)
        cvtColor(pic,pic,COLOR_BGR2GRAY);
    Mat img=Mat::zeros(rows,cols,CV_8UC1);
    for (int x=min_x;x<max_x;x++)
        for (int y=min_y;y<max_y;y++)
        {
            Point2d p=point_tf(x,y,H);
            if(p.x>0 &&p.y>0 && p.x<cols && p.y<rows)
                img.at<uchar>(lround(p.y), lround(p.x))=pic.at<uchar>(y,x);
            else
                cout<<"x="<<x<<"y="<<y<<" point"<<p<<"out of range"<<endl;
        }
    return img;
}

/*!
 *
 * @param u 无畸变图中点的 x
 * @param v 无畸变图中点的 y
 * @param cv_H 单应矩阵
 * @return 二维点
 */
Point2d point_tf(int u,int v,const Mat& cv_H)
{
    double H[3][3];
    for(int row=0;row<cv_H.rows;row++)
        for(int col=0;col<cv_H.cols;col++)
            H[row][col]=cv_H.at<double>(row,col);
    Point2d pout;
    double z=(u*H[2][0]+v*H[2][1]+H[2][2]);
    pout.x= (u*H[0][0]+v*H[0][1]+H[0][2])/z+500;
    pout.y=(u*H[1][0]+v*H[1][1]+H[1][2])/z+500;
    return pout;
}

/*!
 *
 * @param tabel_path 保存的文件地址
 * @param pic_path 用于测试的畸变原图
 * @param note 完成从畸变原图到俯视图的变换 将结果保存成二维数组的形式 并且写入txt中
 * @param x_offset 逆透视x的偏移 matlab 去畸变
 * @param y_offset 逆透视y的偏移
 * @param H 单应矩阵
 */
void write_maptabel(const char* tabel_path,const char* pic_path,const Mat& H,int x_offset,int y_offset)
{
    Mat img= imread(pic_path);
    if(img.rows!=120||img.cols!=188)
        cerr<<"img size wrong"<<endl;
    ofstream outfile;
    outfile.open(tabel_path);
    outfile << "#ifndef PARAM_H\n"
               "#define PARAM_H"<< endl;
    outfile<<"\nstatic const float map_x[120][188]={\n";
    float map_x;
    for(int y=0;y<img.rows;y++)
    {
        outfile<<"{";
        for(int x=0;x<img.cols;x++)
        {
            map_x= point_tf(undis_map[y][x].x-x_offset,undis_map[y][x].y-y_offset,H).x;
            outfile<<map_x<<",";
        }
        outfile<<"},\n";
    }
    outfile<<"};\nstatic const float map_y[120][188]={\n";
    float map_y;
    for(int y=0;y<img.rows;y++)
    {
        outfile<<"{";
        for(int x=0;x<img.cols;x++)
        {
            map_y= point_tf(undis_map[y][x].x-x_offset,undis_map[y][x].y-y_offset,H).y;
            outfile<<map_y<<",";
        }
        outfile<<"},\n";
    }
    outfile<<"};\n#endif"<<endl;
    outfile.close();
}
/*!
 *
 * @param sourse_path 未处理过的原始图片
 * @param rows 映射图片的行数
 * @param cols 映射图片的列数
 * @return 映射图片
 */
Mat tabel_test(const char* sourse_path,int rows,int cols)
{
    Mat img= imread(sourse_path);
    cvtColor(img,img,COLOR_BGR2GRAY);
    Mat out_img=Mat::zeros(rows,cols,CV_8UC1);
    int out_x,out_y;
    for(int x=0;x<188;x++)
        for(int y=0;y<120;y++)
        {
            out_x= lround(map_x[y][x]);
            out_y= lround(map_y[y][x]);
            if(out_x>cols||out_x<0||out_y<0||out_y>rows)
            {
//                cout<<"("<<out_x<<","<<out_y<<")"<<endl;
                continue ;
            }
            out_img.at<uchar>(out_y,out_x)=img.at<uchar>(y,x);
        }
    return out_img;
}


