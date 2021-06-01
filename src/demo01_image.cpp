#include "ros/ros.h" 
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "nav_image");
    ros::NodeHandle nh;
    Mat src,dst;
    src = imread("/home/rostan/桌面/shiyan1.pgm",0); //不加0，默认读入的图像是三通道的
    if(!src.data)
    {
        ROS_INFO("不能打开该图片");
        return -1;
    }
    namedWindow("input_title",0);//参数0可以改变图像大小
    imshow("input_title",src);
    
    
    //使用拷贝构造函数Mat(constMat& m, const Rect& roi )，矩形roi指定了兴趣区
    Mat srcROI(src, Rect(70,70,60,60));//需要改进的地方1  
    Mat matTmp = srcROI.clone();
    cv::threshold(matTmp, matTmp, 80, 255, CV_THRESH_BINARY);//对图形进行二值化处理
    namedWindow("output_gray_image",0);
    imshow("output_gray_image",srcROI);
    namedWindow("output_binary_image",0);
    imshow("output_binary_image",matTmp);
    
    int height = matTmp.rows, width = matTmp.cols;//图像的高和宽
    ROS_INFO("height行=%d, width列=%d",src.rows,src.cols);
    int tmp = 0;//保存当前行的黑色像素点数目
    int *projArray = new int[height];//保存每一行255数目的数组
    
//求行端y坐标*********************************
    //循环访问图像数据，查找每一行的255点的数目
    for (int i = 0; i < height; ++i)  //列
    {
        tmp = 0;
        for (int j = 0; j < width; ++j)  //行
        {
            if (matTmp.at<uchar>(i, j) == 0) //黑色的值为0，白色为255
            {
                ++tmp;
            }
        }
        projArray[i] = tmp;//获取每行的黑色像素个数
        //ROS_INFO("projArray[%d] = %d",i,projArray[i]);     
    }

    int Max_y_index[10];//数组极值
    int index_y = 0;
    for(int i=1;i < height-1;i++)
    {       
        if(projArray[i-1] < projArray[i] && projArray[i] > projArray[i+1] && projArray[i] > 8) //遍历所有行，判断是否极值点
        {
            Max_y_index[index_y] = i;
            index_y++;            
        }       
    }    
    // for(int n=0;n < index_y;n++)
    // {
    //     ROS_INFO("%d",Max_y_index[n]); //打印极值点的行数，也即行端坐标y
    // }

//求行端x坐标***********
    int *projArray2 = new int[width];//保存每一3*3掩膜算子黑色像素点的个数
    int Min_x_index[3];
    int min_index = 0; //最小行端的数组下标
    int Max_x_index[3];
    int max_index = 0; //最小行端的数组下标
    for(int i=0;i < index_y;i++)//遍历所有的作物行
    {
        tmp = 0;
        for(int j=1;j < width ;j++)//遍历该行所有位置掩膜算子的值
        {
           if (matTmp.at<uchar>(Max_y_index[i]-1, j) == 0) tmp++;   //at(x1,x2) x1代表y轴朝下，x2代表x轴朝右
           if (matTmp.at<uchar>(Max_y_index[i]+1, j) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i], j+1) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i], j-1) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i]-1, j+1) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i]-1, j-1) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i]+1, j+1) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i]+1, j-1) == 0) tmp++;
           if (matTmp.at<uchar>(Max_y_index[i], j) == 0) tmp++;               
            projArray2[j-1] = tmp; 
            tmp = 0;
            //ROS_INFO("%d,%d,%d,%d,",Max_y_index[i]-1, j,Max_y_index[i], j+1);
        }        

        //求每个行的左边行端x坐标
        for(int j=0;j<width - 1;j++)
        {
            //ROS_INFO("数组index%d：%d",j,projArray2[j]);
            if(projArray2[j] == 0 && projArray2[j+1] >= 1)
            {
                //ROS_INFO("hh=%d",j);
                Min_x_index[i] = j;
                //ROS_INFO("min=%d",Min_x_index[i]);
                break;
            }                       
        } 

        //求每个行的右边行端x坐标
        for(int j=width-1;j > 0;j--)
        {
        //ROS_INFO("数组index%d：%d",j,projArray2[j]);
            if(projArray2[j] == 0 && projArray2[j-1] >= 1)
            {
                //ROS_INFO("hh=%d",j);
                Max_x_index[i] = j;
                //ROS_INFO("max=%d",Max_x_index[i]);
                break;
            }                       
        }        
    }  
    //求最小的行端   
    for(int i = 0;i < sizeof(Min_x_index)/sizeof(Min_x_index[0]);i++)
    {
        if(Min_x_index[i] < Min_x_index[min_index])
        min_index = i;
    }
    //ROS_INFO("最小的x=%d",Min_x_index[min_index]);
    //求最大的行端
    for(int i = 0;i < sizeof(Max_x_index)/sizeof(Max_x_index[0]);i++)
    {
        if(Max_x_index[i] > Max_x_index[max_index])
        max_index = i;
    }
    //ROS_INFO("最大的x=%d",Max_x_index[max_index]);

    //ROI区域下的八个坐标：
    for(int i=0;i < 4;i++)
    {
        ROS_INFO("坐标：（%d,%d） (%d,%d)",Min_x_index[min_index],Max_y_index[i],Max_x_index[max_index],Max_y_index[i]);
    }

    ROS_INFO("**变换为原图片的坐标**");
    for(int i=0;i < 4;i++)//需要改进的地方1 传参
    {
        ROS_INFO("坐标：（%d,%d） (%d,%d)",Min_x_index[min_index]+70,Max_y_index[i]+70,Max_x_index[max_index]+70,Max_y_index[i]+70);
    }   
    
    waitKey(0);
    return 0;
}