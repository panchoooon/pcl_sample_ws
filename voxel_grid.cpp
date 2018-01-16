//voxcel_gridによるダウンサンプリング
//入力 pcdファイル 出力. pcdファイル

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h> //ダウンサンプリング用

int
main (int argc, char** argv)
{
  //ダウンサンプリング前の点群を入れる Ptr cloudのcloudが点群の名前
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ()); 

  //ダウンサンプリング後の点群を入れる
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ()); 

  // Fill in the cloud data
  pcl::PCDReader reader;
  //上で定義した"cloud" に点群を格納
  reader.read ("table_scene_lms400.pcd", *cloud);

  //C++の標準出力. ダウンサンプリング前の点群を表示
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  //ダウンサンプリングの為にVoxcelGridのオブジェクトをsorとして定義
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //cloudを処理する点群として指定
  sor.setInputCloud (cloud);
  //ダウンサンプリングする間隔([m]単位)を指定
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //フィルタリングを実行
  sor.filter(*cloud_filtered);

  //フィルタリング後の点群を表示
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  //点群保存モジュール pcd writerを定義
  pcl::PCDWriter writer;
  //"table_scene_lms400_downsampled.pcd"としてダウンサンプリングした点群を保存
  //Eigen::Vector4f::Zero (). float型の4次元ベクトル 
  //Eigen::Quaternionf::Identity (). クオータニオン
  //false. binary = false. 二進数でない
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);




  return (0);
}