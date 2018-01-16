#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h> //for 平面検出
#include <pcl/sample_consensus/model_types.h> //for 平面検出
#include <pcl/segmentation/sac_segmentation.h> //for 平面検出 
#include <pcl/filters/voxel_grid.h> //for ダウンサンプリング
#include <pcl/filters/extract_indices.h> //for 平面除去

int
main (int argc, char** argv)
{  
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  //点群データを読み込む
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  //フィルタリング前の点群の数を標準出力
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  //ダウンサンプリングを0.01m間隔で行う
  //pass.setFilterLimitsNegative (true)がないので, 0.01mか
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  //ダウンサンプリングここまで

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  //ダウンサンプリング後の点群の数を表示
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  //ダウンサンプリングした後の点群を保存する
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
  


  //セグメンテーション(平面検出)に必要なコードここから
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  //セグメンテーション用(平面検出用)のインスタンスsegを定義
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // あくまでオプション
  seg.setOptimizeCoefficients (true);
  // 必須for平面検出 ここから
  //検出するモデルを指定(平面を検出するか,線を検出するか,など)
  //pcl::SACMODEL_PLANEは最も大きな平面を自動抽出
  seg.setModelType (pcl::SACMODEL_PLANE);
  //検出方法を指定
  seg.setMethodType (pcl::SAC_RANSAC);
  //おそらく最大どれだけ繰り返すかを指定.なくても良いっぽい.
  //田中さんのも1000なのでとりあえず1000で
  //seg.setMaxIterations (1000);
  //閾[m]値を指定
  seg.setDistanceThreshold (0.01);
  //必須for平面検出ここまで
  





  // 平面除去用のインスタンスを作成
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = cloud_filtered->points.size ();
  //std::cout << cloud_filtered->points.size () << "\n";
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points) //1/3程度のサイズになるまで除去を繰り返す
  {
    // Segment the largest planar component from the remaining cloud
    //最大の平面をセグメンテーションすることで検出する
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    //平面除去ここから
    //処理する点群を指定
    extract.setInputCloud (cloud_filtered);
    //検出した平面を,除去する点群のインデックスとして指定する
    extract.setIndices (inliers);
    //平面以外を除去するように指定
    extract.setNegative (false);
    //平面除去を実行
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    //検出した平面にあたる点群データ(pcd)を保存
    std::stringstream ss;
    ss << "table_scene_lms400_plane_2" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    //setNegativeをtrueにすることで平面を除去する
    extract.setNegative (true);
    extract.filter (*cloud_f);
    //平面除去ここまで
    cloud_filtered.swap (cloud_f);
    i++;
  }

  return (0);
}
