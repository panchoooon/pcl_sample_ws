#include <iostream>
#include <pcl/point_types.h> 
#include <pcl/filters/passthrough.h>// for パススルーフィルタ

int
 main (int argc, char** argv)
{
  //オリジナルと、passthrough後でそれぞれ点群を入れる箱を用意する
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //乱数を用いてオリジナルの点群を作成
  cloud->width  = 5; //幅指定
  cloud->height = 1; //高さ指定
  cloud->points.resize (cloud->width * cloud->height); //点群のサイズを指定

  for (size_t i = 0; i < cloud->points.size (); ++i)//5回分繰り返す。
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  //オリジナルの点群(5個)を表示
  std::cerr << "Cloud before filtering: " << std::endl; //標準エラー出力
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    //標準出力
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
  }

  // filterモジュールのインスタンスpassを作成
  pcl::PassThrough<pcl::PointXYZ> pass;
  // 処理する点群を"cloud"と指定
  pass.setInputCloud (cloud);
  //フィルタをかける座標を指定
  pass.setFilterFieldName ("z");
  //フィルタをかける範囲を指定
  pass.setFilterLimits (0.0, 1.0);
  //上記の設定の点群を除去したい場合は, 以下のモジュールでtrueを指定する。
  //pass.setFilterLimitsNegative (true);
  //上記の設定で実際にフィルタリングをする
  pass.filter (*cloud_filtered);

  //フィルタリング後の点群を標準出力
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "   " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

  return (0);
}

