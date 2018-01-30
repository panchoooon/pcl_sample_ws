/*

table_scene_lms400_extracted.pcdについてクラスタリングをし、
各クラスタの格納されたpcdファイル(白黒)を2次元の画像に変換する。


*/



#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h> //PCL(可視化)


using namespace std;


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[1], *cloud);

	
	double x_min = cloud->points[0].x;
	double x_max = cloud->points[0].x;
	double y_min = cloud->points[0].y;
	double y_max = cloud->points[0].y;
	double z_min = cloud->points[0].z;
	double z_max = cloud->points[0].z;
	 
	


	//各軸における最大値、最小値を求める
	for(size_t i = 0; i < cloud->points.size(); i++){
		if(x_min > cloud->points[i].x)	x_min = cloud->points[i].x;
		if(x_max > cloud->points[i].x)	x_max = cloud->points[i].x;
		if(y_min > cloud->points[i].y)	y_min = cloud->points[i].y;
		if(y_max > cloud->points[i].y)	y_max = cloud->points[i].y;
		if(z_min > cloud->points[i].z)	z_min = cloud->points[i].z;
		if(z_max > cloud->points[i].z)	z_max = cloud->points[i].z;
	}

	// オブジェクトの各座標における中心座標を求める

	double x_ave = (x_min + x_max) / 2.0;
	double y_ave = (y_min + y_max) / 2.0;
	double z_ave = (z_min + z_max) / 2.0;

	//最初に読み込んだクラスタを含む全体の点群画像をカラー点群として読み込む
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_lms400_extracted.pcd", *tmp);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_entire (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*tmp, *cloud_entire);

	//クラスタをカラーに変換
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, *cloud_color);

	for(size_t i = 0; i < cloud_color->points.size(); i++){
            if(x_min < cloud_entire->points[i].x &&
               x_max > cloud_entire->points[i].x &&
               y_min < cloud_entire->points[i].y &&
               y_max > cloud_entire->points[i].y &&
               y_max < cloud_entire->points[i].z &&
               z_max > cloud_entire->points[i].z){
            }
            else{
            // オブジェクト以外(緑)
                cloud_entire->points[i].r = 0;
                cloud_entire->points[i].g = 255;
                cloud_entire->points[i].b = 0;
            }               
        }

	pcl::visualization::CloudViewer viewer ("viewer_clustered");
    viewer.showCloud (cloud_entire);
    while (!viewer.wasStopped ())
    {
      	boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }

	 



	


	return(0);
	
}




