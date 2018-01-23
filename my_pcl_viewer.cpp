#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


int 
main (int argc, char** argv){

    //0. pcdファイルから点群を格納
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("table_scene_lms400.pcd", *cloud_origin);

    pcl::PCLPointCloud2::Ptr cloud_cluster_0 (new pcl::PCLPointCloud2()); 
    pcl::io::loadPCDFile ("cloud_cluster_0.pcd", *cloud_cluster_0);

    pcl::PCLPointCloud2::Ptr cloud_cluster_1 (new pcl::PCLPointCloud2()); 
    pcl::io::loadPCDFile ("cloud_cluster_1.pcd", *cloud_cluster_1); 
 
    pcl::PCLPointCloud2::Ptr cloud_cluster_2 (new pcl::PCLPointCloud2()); 
    pcl::io::loadPCDFile ("cloud_cluster_2.pcd", *cloud_cluster_2);

    pcl::PCLPointCloud2::Ptr cloud_cluster_3 (new pcl::PCLPointCloud2()); 
    pcl::io::loadPCDFile ("cloud_cluster_3.pcd", *cloud_cluster_3);

    pcl::PCLPointCloud2::Ptr cloud_cluster_4 (new pcl::PCLPointCloud2()); 
    pcl::io::loadPCDFile ("cloud_cluster_4.pcd", *cloud_cluster_4);


    //PointXYZへ変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_0_PointXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_cluster_0, *cloud_cluster_0_PointXYZ);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_1_PointXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_cluster_1, *cloud_cluster_1_PointXYZ);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_2_PointXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_cluster_2, *cloud_cluster_2_PointXYZ);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_3_PointXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_cluster_3, *cloud_cluster_3_PointXYZ);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_4_PointXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_cluster_4, *cloud_cluster_4_PointXYZ);


    //点群表示
    pcl::visualization::CloudViewer viewer ("viewer1");
    viewer.showCloud (cloud_origin);
    //viewer.showCloud (cloud_cluster_0_PointXYZ);
    //viewer.showCloud (cloud_cluster_1_PointXYZ);
    //viewer.showCloud (cloud_cluster_2_PointXYZ);
    //viewer.showCloud (cloud_cluster_3_PointXYZ);
    //viewer.showCloud (cloud_cluster_4_PointXYZ);
    while (!viewer.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
  
    pcl::PCDWriter writer;
    return(0);

}
