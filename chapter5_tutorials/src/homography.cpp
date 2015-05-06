
// Based on the OpenCV samples.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <chapter5_tutorials/HomographyConfig.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <string>
#include <vector>
#include <iostream>

class Homography
{
public:
    Homography()
        : nh_()
        , it_(nh_)
        , feature_detector_("SURF")
        , descriptor_extractor_("SURF")
        , descriptor_matcher_("FlannBased")
        , matcher_filter_("CrossCheckFilter")
        , threshold_(1.0)
    {
        ros::NodeHandle nh_priv("~");
        nh_priv.param("detector", feature_detector_, feature_detector_);
        nh_priv.param("matcher", descriptor_matcher_, descriptor_matcher_);
        nh_priv.param("filter", matcher_filter_, matcher_filter_);
        nh_priv.param("threshold", threshold_, threshold_);

        detector_ = cv::FeatureDetector::create(feature_detector_);
        if (detector_ == NULL)
        {
            ROS_FATAL_STREAM(
                    "Feature detector " << feature_detector_ <<
                    " not available!");
        }

        extractor_ = cv::DescriptorExtractor::create(descriptor_extractor_);
        if (extractor_ == NULL)
        {
            ROS_FATAL_STREAM(
                    "Descriptor extractor " << descriptor_extractor_ <<
                    " not available!");
        }

        matcher_ = cv::DescriptorMatcher::create(descriptor_matcher_);
        if (matcher_ == NULL)
        {
            ROS_FATAL_STREAM(
                    "Descriptor matcher " << descriptor_matcher_ <<
                    " not available!");
        }

        filter_ = getMatcherFilterType(matcher_filter_);
        if (filter_ == -1)
        {
            ROS_FATAL_STREAM(
                    "Matcher filter " << matcher_filter_ <<
                    " not available!");
        }

        cv::namedWindow("correspondences", cv::WINDOW_NORMAL);
        cv::namedWindow("homography", cv::WINDOW_NORMAL);

        pub_ = it_.advertise("homography", 1);
        sub_ = it_.subscribe("camera/image_raw", 1, &Homography::callback, this);
    }

private:
    // Matcher filters:
    enum
    {
        NONE_FILTER = 0,
        CROSS_CHECK_FILTER = 1
    };

    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    image_transport::Subscriber sub_;

    cv_bridge::CvImagePtr      image_0_;
    cv_bridge::CvImageConstPtr image_1_;
    cv_bridge::CvImagePtr      warped_image_;

    std::string feature_detector_;
    std::string descriptor_extractor_;
    std::string descriptor_matcher_;
    std::string matcher_filter_;
    double threshold_;

    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    int filter_;

    std::vector<cv::KeyPoint> keypoints_0_;
    cv::Mat descriptors_0_;

    void callback(const sensor_msgs::ImageConstPtr& msg)
    {
        if (image_0_ == NULL)
        {
            // Take first image:
            try
            {
                image_0_ = cv_bridge::toCvCopy(msg,
                        sensor_msgs::image_encodings::isColor(msg->encoding) ?
                        sensor_msgs::image_encodings::BGR8 :
                        sensor_msgs::image_encodings::MONO8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR_STREAM("Failed to take first image: " << e.what());
                return;
            }

            ROS_INFO("First image taken");

            // Detect keypoints:
            detector_->detect(image_0_->image, keypoints_0_);
            ROS_INFO_STREAM(keypoints_0_.size() << " points found.");

            // Extract keypoints' descriptors:
            extractor_->compute(image_0_->image, keypoints_0_, descriptors_0_);
        }
        else
        {
            // Take second image:
            try
            {
                image_1_ = cv_bridge::toCvShare(msg,
                        sensor_msgs::image_encodings::isColor(msg->encoding) ?
                        sensor_msgs::image_encodings::BGR8 :
                        sensor_msgs::image_encodings::MONO8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR_STREAM("Failed to take image: " << e.what());
                return;
            }

            // Detect keypoints:
            std::vector<cv::KeyPoint> keypoints_1;
            detector_->detect(image_1_->image, keypoints_1);
            ROS_INFO_STREAM(keypoints_1.size() << " points found on the new image.");

            // Extract keypoints' descriptors:
            cv::Mat descriptors_1;
            extractor_->compute(image_1_->image, keypoints_1, descriptors_1);

            // Compute matches:
            std::vector<cv::DMatch> matches;
            match(descriptors_0_, descriptors_1, matches);

            // Compute homography:
            cv::Mat H;
            homography(keypoints_0_, keypoints_1, matches, H);

            // Draw matches:
            const int s = std::max(image_0_->image.rows, image_0_->image.cols);
            cv::Size size(s, s);
            cv::Mat draw_image;
            warped_image_ = boost::make_shared<cv_bridge::CvImage>(
                    image_0_->header, image_0_->encoding,
                    cv::Mat(size, image_0_->image.type()));
            if (!H.empty()) // filter outliers
            {
                std::vector<char> matchesMask(matches.size(), 0);

                const size_t N = matches.size();
                std::vector<int> queryIdxs(N), trainIdxs(N);
                for (size_t i = 0; i < N; ++i)
                {
                    queryIdxs[i] = matches[i].queryIdx;
                    trainIdxs[i] = matches[i].trainIdx;
                }

                std::vector<cv::Point2f> points1, points2;
                cv::KeyPoint::convert(keypoints_0_, points1, queryIdxs);
                cv::KeyPoint::convert(keypoints_1, points2, trainIdxs);

                cv::Mat points1t;
                cv::perspectiveTransform(cv::Mat(points1), points1t, H);

                double maxInlierDist = threshold_ < 0 ? 3 : threshold_;
                for (size_t i1 = 0; i1 < points1.size(); ++i1)
                {
                    if (cv::norm(points2[i1] - points1t.at<cv::Point2f>((int)i1,0)) <= maxInlierDist ) // inlier
                        matchesMask[i1] = 1;
                }
                // draw inliers
                cv::drawMatches(
                        image_0_->image, keypoints_0_,
                        image_1_->image, keypoints_1, matches,
                        draw_image, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
                        matchesMask,
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                // draw outliers
                for (size_t i1 = 0; i1 < matchesMask.size(); ++i1)
                    matchesMask[i1] = !matchesMask[i1];
                cv::drawMatches(
                        image_0_->image, keypoints_0_,
                        image_1_->image, keypoints_1, matches,
                        draw_image, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0),
                        matchesMask,
                        cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                ROS_INFO_STREAM("Number of inliers: " << cv::countNonZero(matchesMask));

                // Wrap the new image using the homography,
                // so we should have something similar to the original image:
                warpPerspective(
                        image_1_->image, warped_image_->image, H, size,
                        cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

                // Print the homography found:
                ROS_INFO_STREAM("Homography = " << H);
            }
            else
            {
                cv::drawMatches(
                        image_0_->image, keypoints_0_,
                        image_1_->image, keypoints_1, matches,
                        draw_image);

                image_1_->image.copyTo(warped_image_->image);

                ROS_WARN_STREAM("No homography transformation found!");
            }

            // Publish warped image (using homography):
            warped_image_->header = image_1_->header;
            pub_.publish(warped_image_->toImageMsg());

            // Show images:
            cv::imshow("correspondences", draw_image);
            cv::imshow("homography", warped_image_->image);
            cv::waitKey(3);
        }
    }

    void match(
            const cv::Mat& descriptors_0, const cv::Mat& descriptors_1,
            std::vector<cv::DMatch>& matches)
    {
        switch (filter_)
        {
        case CROSS_CHECK_FILTER:
            crossCheckMatching(descriptors_0, descriptors_1, matches, 1);
            break;
        default:
            simpleMatching(descriptors_0, descriptors_1, matches);
        }
    }

    void homography(
            const std::vector<cv::KeyPoint>& keypoints_0,
            const std::vector<cv::KeyPoint>& keypoints_1,
            const std::vector<cv::DMatch>& matches,
            cv::Mat& H)
    {
        const size_t N = matches.size();
        std::vector<int> queryIdxs(N), trainIdxs(N);
        for (size_t i = 0; i < N; ++i)
        {
            queryIdxs[i] = matches[i].queryIdx;
            trainIdxs[i] = matches[i].trainIdx;
        }

        if (threshold_ >= 0)
        {
            std::vector<cv::Point2f> points1;
            cv::KeyPoint::convert(keypoints_0, points1, queryIdxs);

            std::vector<cv::Point2f> points2;
            cv::KeyPoint::convert(keypoints_1, points2, trainIdxs);

            H = cv::findHomography(
                    cv::Mat(points1), cv::Mat(points2),
                    cv::RANSAC, threshold_);
        }
    }

    static int getMatcherFilterType(const std::string& filter)
    {
        if (filter == "NoneFilter")
        {
            return NONE_FILTER;
        }
        else if (filter == "CrossCheckFilter")
        {
            return CROSS_CHECK_FILTER;
        }
        else
        {
            ROS_ERROR_STREAM("Invalid filter " << filter);
            return -1;
        }
    }

    void simpleMatching(
            const cv::Mat& descriptors_0, const cv::Mat& descriptors_1,
            std::vector<cv::DMatch>& matches)
    {
        matcher_->match(descriptors_0, descriptors_1, matches);
    }

    void crossCheckMatching(
            const cv::Mat& descriptors_0, const cv::Mat& descriptors_1,
            std::vector<cv::DMatch>& matches, int knn = 1)
    {
        matches.clear();
        std::vector<std::vector<cv::DMatch> > matches12, matches21;
        matcher_->knnMatch(descriptors_0, descriptors_1, matches12, knn);
        matcher_->knnMatch(descriptors_1, descriptors_0, matches21, knn);
        for (size_t m = 0; m < matches12.size(); ++m)
        {
            bool findCrossCheck = false;
            for(size_t fk = 0; fk < matches12[m].size(); ++fk)
            {
                cv::DMatch forward = matches12[m][fk];
                for (size_t bk = 0; bk < matches21[forward.trainIdx].size(); ++bk)
                {
                    cv::DMatch backward = matches21[forward.trainIdx][bk];
                    if (backward.trainIdx == forward.queryIdx)
                    {
                        matches.push_back(forward);
                        findCrossCheck = true;
                        break;
                    }
                }
                if (findCrossCheck) break;
            }
        }
    }
};


int
main(int argc, char** argv)
{
    ros::init(argc, argv, "homography");

    cv::initModule_nonfree();

    Homography h;

    while (ros::ok())
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}
