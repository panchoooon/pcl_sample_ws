
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>


#include <pcl/filters/voxel_grid.h>//ダウンサンプリング

#include <pcl_conversions/pcl_conversions.h>

//rosメッセージを扱うときのやつ
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <limits>



using namespace std;

int main (int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile ("2Tshirt/0_ex1.pcd", *cloud);

	

	pcl::PointXYZRGB point;

	uint8_t r = 255;
	uint8_t g = 0;
	uint8_t b = 0;
	int32_t rgb = (r << 16) | (g << 8) | b; 

	point.rgb = *(float *)(&rgb);

	cout <<"point.rgb:" << point.rgb << endl;

	cout <<"points[0]:" << cloud->points[150].rgb << endl;

	uint32_t rgb2 = *reinterpret_cast<int*>(&cloud->points[150].rgb);
		
	int r2 = ((0x0000FF00 & rgb2) >>  16);
	int g2 = ((0x0000FF00 & rgb2) >>  8);
	int b2 = (rgb2)       & 0x0000ff;

	//cout << rgb2 <<endl;
	cout << r2 <<endl;
	cout << g2 <<endl;
	cout << b2 <<endl;

	//cout << " " <<	
	
	
	//cout << "width:"<< cloud->width << endl;
	//cout << "height:" << cloud->height << endl;
	int i = 0;
	int flag = 0;
/*
	while(flag==0){
		
		//uint8_t r =	cloud->points[i].r & 0x0000ff;
		cout << "points[i].z: "  << cloud->points[i].z << endl;
		//cout << "points["<< i <<"]:.r" << cloud->points[i].r << endl;
	
		i++;
		if(isnan(cloud->points[i].x))
			flag = 0;
		else
			flag = 1;
		
	}
*/

	return(0);


}

/*
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("0_ex1.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;

  return (0);
}
*/


