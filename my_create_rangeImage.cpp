#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/range_image/range_image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
	//my_clusteringでクラスタリングした各pcdデータを取得

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("my_cloud_cluster.pcd", *cloud_cluster);

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
	rangeImage.createFromPointCloud(cloud_cluster, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	

	// レンジデータの最大値と最小値を取得
	float min_range, max_range;
	rangeImage.getMinMaxRanges(min_range, max_range);
	float min_max_range = max_range - min_range;

	// 結果画像領域を生成
	cv::Mat& image(rangeImage.height, rangeImage.width,CV8U);
	unsigned char r,g,b;
	for(int y=0; y < rangeImage.height; y++){
		for(int x=0; x<rangeImage.width; x++){
			pcl::PointWithRange rangePt = rangeImage.getPoint(x,y);
			if(rangePt.range == INFINITY || rangePt.range == -INFINITY || rangePt.range == NAN){
				// レンジが無限遠や無効なデータの場合
				pcl::visualization::FloatImageUtils::getColorForFloat (rangePt.range, r, g, b);
			}
			else{
				// レンジを0-1の範囲に正規化し、カラー値へ変換
				float value = (rangePt.range - min_range) / min_max_range;
				pcl::visualization::FloatImageUtils::getColorForFloat (value, r, g, b);
			}
			// ピクセルにカラー値を格納
			image.at<cv::Vec3b>(y,x)[0] = b;
			image.at<cv::Vec3b>(y,x)[1] = g;
			image.at<cv::Vec3b>(y,x)[2] = r;
		}
	}
	// 画像を保存
	cv::imwrite("my_rangeImage.png", image);

	return(0);
}

*/


