#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_plane", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        // initialize PointClouds
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> final;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        // populate our PointCloud with points
        cloud.width    = 500;
        cloud.height   = 1;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        std::vector<int> inliers;

        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud.makeShared()));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);

        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(cloud, inliers, final);
        pcl::toROSMsg(final, output);
        pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_model_estimation");

    cloudHandler handler;

    ros::spin();

    return 0;
}

