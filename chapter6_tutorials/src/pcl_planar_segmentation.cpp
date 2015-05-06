#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_segmented", 1);
        ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
        coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented;

        pcl::fromROSMsg(input, cloud);

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.01);
        segmentation.setInputCloud(cloud.makeShared());
        segmentation.segment(*inliers, coefficients);

        // Publish the model coefficients
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);
        coef_pub.publish(ros_coefficients);

        // Publish the Point Indices
        pcl_msgs::PointIndices ros_inliers;
        pcl_conversions::fromPCL(*inliers, ros_inliers);
        ind_pub.publish(ros_inliers);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(cloud_segmented);

        //Publish the new cloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_segmented, output);
        pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub, ind_pub, coef_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_planar_segmentation");

    cloudHandler handler;

    ros::spin();

    return 0;
}


