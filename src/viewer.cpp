#include "pcl_ros/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "dynamic_reconfigure/server.h"
#include "fabric_grasping/parametersConfig.h"

pcl::visualization::PCLVisualizer::Ptr viewer_ptr (new pcl::visualization::PCLVisualizer ("pclEditNormals Viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
int int_displayed_normals;
double double_length_normals;

void callback(fabric_grasping::parametersConfig &config, u_int32_t level)
{
    ROS_INFO("Parameters changed!");
    int_displayed_normals = config.int_displayed_normals;
    double_length_normals = config.double_length_normals;
}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
    cloud_ptr = input_cloud;
}

void normalsCallback(const pcl::PointCloud<pcl::Normal>::Ptr input_normals)
{
    if (cloud_ptr->size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorhandler (cloud_ptr, 0, 255, 0);
    
        viewer_ptr->removeAllPointClouds();
        viewer_ptr->addPointCloud<pcl::PointXYZ>(cloud_ptr, colorhandler, "cloud");
        viewer_ptr->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_ptr, input_normals, int_displayed_normals, double_length_normals, "normals");
        viewer_ptr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting viewer node...");
    
    ros::init(argc, argv, "viewer");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<fabric_grasping::parametersConfig> server;
    dynamic_reconfigure::Server<fabric_grasping::parametersConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Subscriber subscriber_pointcloud = n.subscribe("pointcloud_edited", 10, cloudCallback);
    ros::Subscriber subscriber_pointcloud_normals = n.subscribe("pointcloud_edited_normals", 10, normalsCallback);
    ros::Rate loop_rate(1);

    viewer_ptr->setBackgroundColor (0, 0, 0);
    viewer_ptr->addCoordinateSystem (0.2);
    viewer_ptr->initCameraParameters ();

    while (ros::ok()) {
        ros::spinOnce();
        viewer_ptr->spinOnce (100);
        loop_rate.sleep();
    }

    return 0;
}