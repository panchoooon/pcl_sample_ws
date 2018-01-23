/*
pcdファイルから点群データを読み込んでkdtreeモジュールでクラスタリングする

流れ
1.pcdファイルから読込
2.ダウンサンプリングをする
3.注目領域抽出 
4.平面検出
5.平面除去
6.kd-treeによるクラスタリング

*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h> //ダウンサンプリング用
#include <pcl/filters/passthrough.h>// for パススルーフィルタ
#include <pcl/PCLPointCloud2.h> // PCL2 to PCL<T
#include <pcl/ModelCoefficients.h>              // PCL(平面検出)
#include <pcl/segmentation/sac_segmentation.h>  // PCL(平面検出)
#include <pcl/filters/extract_indices.h>        // PCL(平面除去)
#include <pcl/kdtree/kdtree.h>                  // PCL(クラスタリング)
#include <pcl/segmentation/extract_clusters.h>  // PCL(クラスタリング)
//convert用
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h> //PCL(可視化)



//opencv用
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int
main (int argc, char** argv)
{
//0. オリジナルの点を格納するインスタンスを定義
    pcl::PCLPointCloud2::Ptr cloud_origin (new pcl::PCLPointCloud2()); 
//1.pcdファイルから読込
    
    pcl::io::loadPCDFile ("table_scene_lms400.pcd", *cloud_origin);   

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


    //変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_downsampled, *cloud_filtered);

    
    std::cerr << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";



    //保存

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered);
/*
    //可視化for downsampled
    pcl::visualization::CloudViewer viewer_down ("viewer_downsampled");
    viewer_down.showCloud (cloud_filtered2);
    while (!viewer_down.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }

*/

/* うまく領域を指定できないのでとりあえずコメントアウト 17/01/18 pancho
//3.注目領域抽出   

    //passthrough filter用
    pcl::PCLPointCloud2::Ptr cloud_passthroughed (new pcl::PCLPointCloud2());

    // filterモジュールのインスタンスpassを作成
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // 処理する点群を"cloud"と指定
    pass.setInputCloud (cloud_origin);
    //フィルタをかける座標を指定
    pass.setFilterFieldName ("x");
    //フィルタをかける範囲を指定
    pass.setFilterLimits (0.0, 0.1);
    //上記の設定の点群を除去したい場合は, 以下のモジュールでtrueを指定する。
    //pass.setFilterLimitsNegative (true);
    //上記の設定で実際にフィルタリングをする
    pass.filter(*cloud_passthroughed);

    std::cerr << "PointCloud after passthrough filter: " << cloud_passthroughed->width * cloud_passthroughed->height
       << " data points (" << pcl::getFieldsList (*cloud_passthroughed) << ").\n";

    //セグメンテーションの為に型変換toPointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthroughed2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_passthroughed, *cloud_passthroughed2);
   


*/  



  
//4.平面検出
 //セグメンテーション(平面検出)に必要なコードここから

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
 
   

    //セグメンテーション用
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE); //平面を検出するモデル
    seg.setMethodType (pcl::SAC_RANSAC);//RANSAC法
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    seg.setInputCloud(cloud_filtered);
    seg.segment (*inliers, *coefficients); 

//5.平面除去
    //平面除去用	
    pcl::ExtractIndices<pcl::PointXYZ> extract;  
    //平面除去後の点群を入れる
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);


    //保存
    pcl::PCDWriter writer2;
    writer2.write<pcl::PointXYZ> ("table_scene_lms400_extracted.pcd", *cloud_filtered);

    //平面除去後の物体の点群を表示
    std::cerr << "After extracted plane " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
    pcl::visualization::CloudViewer viewer ("viewer_extracted");
    viewer.showCloud (cloud_filtered);
    while (!viewer.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }


//6.kd-treeクラスタリング

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    //tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    tree->setInputCloud(cloud_filtered);
    ec.setClusterTolerance (0.02); // Tolerance(同じクラスタと見なす許容範囲) 2cm
    ec.setMinClusterSize (100); //最低でも100個の点がないと1つのクラスタと見なさない
    ec.setMaxClusterSize (20000); // 1クラスタは5000より多くの点を持たない
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract (cluster_indices);

    //std::cerr << "cluster_indice[0]" << cluster_indices[0] << "\n" << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        	cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;

 	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
 	//可視化
   	pcl::visualization::CloudViewer viewer_kdtree ("viewer_kd-tree");
    	viewer_kdtree.showCloud (cloud_cluster);
    	while (!viewer_kdtree.wasStopped ())
    	{
      	     boost::this_thread::sleep (boost::posix_time::microseconds (100));
    	}
 
	
	//保存
	std::stringstream ss;
    	ss << "my_cloud_cluster_" << j << ".pcd";
    	writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    	j++;

	//ここから、画像に直して出力するコード
        // オブジェクトの各座標における最小、最大値を求める
        double obj_pc_x_min = cloud_cluster->points[*it->indices.begin()].x;
        double obj_pc_x_max = cloud_cluster->points[*it->indices.begin()].x;
        double obj_pc_y_min = cloud_cluster->points[*it->indices.begin()].y;
        double obj_pc_y_max = cloud_cluster->points[*it->indices.begin()].y;
        double obj_pc_z_min = cloud_cluster->points[*it->indices.begin()].z;
        double obj_pc_z_max = cloud_cluster->points[*it->indices.begin()].z;

	for(std::vector<int>::const_iterator it2 = it->indices.begin(); it2 != it->indices.end(); it2++){
            if(obj_pc_x_min > cloud_cluster->points[*it2].x) obj_pc_x_min = cloud_cluster->points[*it2].x;
            if(obj_pc_x_max < cloud_cluster->points[*it2].x) obj_pc_x_max = cloud_cluster->points[*it2].x;
            if(obj_pc_y_min > cloud_cluster->points[*it2].y) obj_pc_y_min = cloud_cluster->points[*it2].y;
            if(obj_pc_y_max < cloud_cluster->points[*it2].y) obj_pc_y_max = cloud_cluster->points[*it2].y;
            if(obj_pc_z_min > cloud_cluster->points[*it2].z) obj_pc_z_min = cloud_cluster->points[*it2].z;
            if(obj_pc_z_max < cloud_cluster->points[*it2].z) obj_pc_z_max = cloud_cluster->points[*it2].z;            
         }

	// オブジェクトの各座標における中心座標を求める
        double obj_pc_x_ave = (obj_pc_x_min + obj_pc_x_max) / 2.0;
        double obj_pc_y_ave = (obj_pc_y_min + obj_pc_y_max) / 2.0;
        double obj_pc_z_ave = (obj_pc_z_min + obj_pc_z_max) / 2.0;



        
    }


    

    
    



 return(0);
}

