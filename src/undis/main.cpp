#include "undis.h"
#define BOARD_W 225
#define BOARD_H 225
#define FX 60.9244
#define FY 61.6919
#define CX 117.3184
#define CY 83.2287
#define UNDIS_PATH "D:/test/undis.txt"
#define PIC_PATH "D:/picture/PIC22/"
#define PNP_PIC_PATH "D:/picture/PIC19_undis/1.BMP"
#define S_PIC_PATH "D:/picture/PIC19/1.BMP"
#define TABEL_PATH "D:/ros_race/yangyi cources/c++/undis/param.h"

float alpha=-43.1328311;
float beita= -0.5776444;
float gamma=-2.4437045;//对应的绕 x y z 旋转的角度
int x_off,y_off;
int main()
{
    get_undis_map(UNDIS_PATH,PIC_PATH,true,&x_off,&y_off);
    const cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0 );
    const vector<Point3d> Pc={
            Point3d(-BOARD_W,-BOARD_H,30),
            Point3d(BOARD_W,-BOARD_H,30),
            Point3d(BOARD_W,BOARD_H,30),
                Point3d(-BOARD_W,BOARD_H,30)};
    vector<Point2d> Pi;
    get_pi(PNP_PIC_PATH,Pi);
    Mat origin_R;
    get_rotation(K,Pi,Pc,origin_R);
    cout<<origin_R.inv()<<endl;
    Mat deal_R=angle2rotation(0,alpha)*angle2rotation(1,beita);
    cout<<deal_R<<endl;
    Mat R=deal_R.inv();
    Mat H=K*R*K.inv();
    Mat img_s =imread(PNP_PIC_PATH);
//    Mat pic=roi_tf(0,0,img_s.cols,img_s.rows,1000,1500,H,img_s);
    write_maptabel(TABEL_PATH,S_PIC_PATH,H,x_off,y_off);
    Mat test_img=tabel_test(S_PIC_PATH,1000,1500);
    imshow("pic",test_img);
    waitKey(0);
    return 0;
}
