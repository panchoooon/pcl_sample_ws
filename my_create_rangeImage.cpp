#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>//ダウンサンプリング


using namespace std;

int main (int argc, char** argv) {
  //pcl::PointCloud<pcl::PointXYZRGB> pointCloudtype;

  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  
  // Generate the data
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point);
    }
  }

    //0. オリジナルの点を格納するインスタンスを定義
    pcl::PCLPointCloud2::Ptr cloud_origin (new pcl::PCLPointCloud2()); 

    //1.pcdファイルから読込
    pcl::io::loadPCDFile (argv[1], *cloud_origin); 

	string argv1 = argv[1];//保存のために argv[1]を文字列型として受け取る。

    //2.ダウンサンプリングをする
    //C++の標準出力. ダウンサンプリング前の点群を表示
    std::cerr << "PointCloud before downsampling: " << cloud_origin->width * cloud_origin->height 
       << " data points (" << pcl::getFieldsList (*cloud_origin) << ").\n";

    //ダウンサンプリング後の点群を入れる
    pcl::PCLPointCloud2::Ptr cloud_downsampled (new pcl::PCLPointCloud2()); 
     

    //ダウンサンプリングの為にVoxcelGridのオブジェクトをsorとして定義
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    //cloudを処理する点群として指定
    sor.setInputCloud (cloud_origin);
    //ダウンサンプリングする間隔([m]単位)を指定
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    //フィルタリングを実行
    sor.filter(*cloud_downsampled);


    //保存&可視化&後の処理ために変換
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_downsampled, *cloud_filtered);

/*
 	pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
	*test = createPointCloud(point);

  	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
  	float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  	float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  	float noiseLevel=0.00;
  	float minRange = 0.0f;
  	int borderSize = 1;
  
  	pcl::RangeImage rangeImage;
  	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  	std::cout << rangeImage << "\n";
*/
	return(0);
}





/*

table_scene_lms400_extracted.pcdについてクラスタリングをし、
各クラスタの格納されたpcdファイル(白黒)を2次元の画像に変換する。


*/




/*

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/cloud_viewer.h> //PCL(可視化)
#include <pcl/range_image/range_image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef INFINITY
#define INFINITY HUGE_VAL
#endif

#ifndef NAN
#define NAN 0xffc00000
#endif

using namespace std;


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *cloud);

	// レンジデータパラメータ
	float angularResolution = (float) (  0.5f * (M_PI/180.0f));  //   解像度0.5度（ラジアン表記）
	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 横の最大角度幅
	float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 縦の最大角度幅
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);  // 移動
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;  // カメラ座標原点を原点とする
	float noiseLevel=0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	// ポイントクラウドからレンジデータを生成
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	// レンジデータの最大値と最小値を取得
	//float min_range, max_range;
	//rangeImage.getMinMaxRanges(min_range, max_range);
	//float min_max_range = max_range - min_range;

	// 結果画像領域を生成
	//cv::Mat& image(rangeImage.height, rangeImage.width, CV_8UC3);



	return(0);
	
}

*/


