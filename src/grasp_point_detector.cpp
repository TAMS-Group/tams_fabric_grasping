#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "pcl/point_types.h"
#include "tf2_eigen/tf2_eigen.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/passthrough.h"
#include "chrono"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/centroid.h"
#include "pcl/filters/crop_box.h"
#include "dynamic_reconfigure/server.h"
#include "fabric_grasping/parametersConfig.h"
#include "actionlib/server/simple_action_server.h"
#include "fabric_grasping/GetGraspPointAction.h"
#include "math.h"

class GraspPointActionServer
{
protected:

    ros::NodeHandle n;
    actionlib::SimpleActionServer<fabric_grasping::GetGraspPointAction> as_;    
    std::string action_name_;

    fabric_grasping::GetGraspPointFeedback feedback_;
    fabric_grasping::GetGraspPointResult result_;

    ros::Publisher publisher_pointcloud;
    ros::Publisher publisher_pointcloud_normals;
    ros::Publisher publisher_pointcloud_debug;
    ros::Publisher publisher_pointcloud_orientation_scan;
    ros::Publisher publisher_vis_marker;
    ros::Publisher publisher_pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_ptr {new pcl::PointCloud<pcl::PointXYZ>};

    // Dynamic reconfigure
    int int_gp_method;
    int int_setKSearch;
    double double_setMinRadius;
    double double_setMaxRadius;
    double double_setNormalDistanceWeight;
    double double_setDistanceThreshold;
    std::string string_frame;
    bool bool_crop_box;

    tf2_ros::Buffer tf_buffer;
    
public:

    // Dynamic reconfigure callback
    inline void callbackDynamicReconfigure(fabric_grasping::parametersConfig &config, u_int32_t level)
    {        
        int_setKSearch = config.int_setKSearch;
        double_setNormalDistanceWeight = config.double_setNormalDistanceWeight;
        double_setDistanceThreshold = config.double_setDistanceThreshold;
        string_frame = config.string_frame;
        bool_crop_box = config.bool_crop_box;
        int_gp_method = config.gp_method;
        double_setMinRadius = config.double_setMinRadius;
        double_setMaxRadius = config.double_setMaxRadius;
        ROS_INFO("Parameters changed!");
    }
    
    // This function checks a point for NaN value. If Point value is NaN the function returns 1, otherwise 0.
    inline bool checkPointNaN(pcl::PointXYZ& input)
    {
        if (input.x == input.x)
        {
            return false;
        }
        return true;
    }

    inline double degreesToRadians(double degrees) {
        return -(degrees * (M_PI / 180));
    }

    // Calculates quaternion from RANSAC model cofficients.
    inline Eigen::Quaternionf obtainCylinderOrientationFromModel(pcl::ModelCoefficients& coefficients)
    {
        Eigen::Vector3f axis_vector(coefficients.values.at(3), coefficients.values.at(4), coefficients.values.at(5));
        Eigen::Vector3f up_vector(0.0, 0.0, -1.0);
        Eigen::Vector3f right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();

        Eigen::Quaternionf q(Eigen::AngleAxisf(-1.0 * std::acos(axis_vector.dot(up_vector)), right_vector));
        q.normalize();

        return q;
    }

    inline Eigen::Matrix3f editMatrix(Eigen::Matrix3f input_matrix)
    {
        Eigen::Matrix3f matrix_rotation;
        float lf = 1 / sqrt(std::pow(input_matrix.coeff(0,2),2) + std::pow(input_matrix.coeff(1,2),2));

        matrix_rotation <<  input_matrix.coeff(1,2) * lf,    input_matrix.coeff(0,2) * lf,   0,
                            -input_matrix.coeff(0,2) * lf,    input_matrix.coeff(1,2) * lf,   0,
                            0,                          0,                              1;

        return matrix_rotation;
    }

    inline Eigen::Matrix3f fixMatrixY(Eigen::Matrix3f input_matrix)
    {
        Eigen::Matrix3f matrix_fixed_y;

        if (input_matrix.coeff(1,1) < 0)
        {
            matrix_fixed_y <<   -input_matrix.coeff(0,0), -input_matrix.coeff(0,1)  , input_matrix.coeff(0,2),
                                -input_matrix.coeff(1,0), -input_matrix.coeff(1,1)  , input_matrix.coeff(1,2),
                                input_matrix.coeff(2,0)     , input_matrix.coeff(2,1)       , input_matrix.coeff(2,2);
            return matrix_fixed_y;
        }
        else
        {
            return input_matrix;
        }
    }

    inline Eigen::Matrix3f rotateMatrixZ(Eigen::Matrix3f input_matrix, double degrees)
    {
        Eigen::Matrix3f rotation_matrix;
        double radians = degreesToRadians(degrees);

        rotation_matrix <<  cosf(radians), -sinf(radians), 0,
                            sinf(radians),  cosf(radians), 0,
                            0,                          0, 1;

        return rotation_matrix * input_matrix;
    }
    
    // This function returns the point with the highest z value from a pointcloud.
    inline pcl::PointXYZ getPointMaxZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        pcl::PointXYZ point_z_max;
        pcl::PointXYZ point_temp;

        if (input->isOrganized())
        {
            int input_row = input->height;
            int input_colum = input->width;

            for (int r = 0; r < input_row; r++)
            {
                for (int c = 0; c < input_colum; c++)
                {            
                    point_temp = input->at(c, r);

                    if (!checkPointNaN(point_temp))
                    {                
                        if (point_z_max.z < point_temp.z)
                        {
                            point_z_max = point_temp;
                        }
                    }
                }
            }
        }
        else
        {
            int input_length = input->size();

            for (int i = 0; i < input_length; i++)
            {            
                point_temp = input->at(i);

                if (!checkPointNaN(point_temp))
                {                
                    if (point_z_max.z < point_temp.z)
                    {
                        point_z_max = point_temp;
                    }
                }
            }
        }

        return point_z_max;
    }
    inline pcl::PointXYZ getPointMaxZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointIndices::Ptr& indices_input)
    {
        pcl::PointXYZ point_z_max;
        pcl::PointXYZ point_temp;

        int indices_input_length = indices_input->indices.size();

        for (int i = 0; i < indices_input_length; i++)
        {            
            point_temp = input->at(indices_input->indices.at(i));

            if (!checkPointNaN(point_temp))
            {                
                if (point_z_max.z < point_temp.z)
                {
                    point_z_max = point_temp;
                }
            }
        }
        return point_z_max;
    }

    // Returns point with lowest z value from a pointcloud.
    inline pcl::PointXYZ getPointMinZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        pcl::PointXYZ point_z_min;
        pcl::PointXYZ point_temp;

        if (input->isOrganized())
        {
            int input_row = input->height;
            int input_colum = input->width;

            for (int r = 0; r < input_row; r++)
            {
                for (int c = 0; c < input_colum; c++)
                {            
                    point_temp = input->at(c, r);

                    if (!checkPointNaN(point_temp) or (point_temp.z != 0.0))
                    {                
                        if (point_z_min.z > point_temp.z)
                        {
                            point_z_min = point_temp;
                        }
                    }
                }
            }
        }
        else
        {
            int input_length = input->size();

            for (int i = 0; i < input_length; i++)
            {            
                point_temp = input->at(i);

                if (!checkPointNaN(point_temp) or (point_temp.z != 0.0))
                {                
                    if (point_z_min.z > point_temp.z)
                    {
                        point_z_min = point_temp;
                    }
                }
            }
        }

        return point_z_min;
    }

    // This function returns the centroid point of a given pointcloud and optionally point indices.
    inline pcl::PointXYZ getCentroidPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud)
    {
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        pcl::PointXYZ point_centroid;

        for (int i = 0; i < input_pointcloud->size(); i++)
        {
            if (!checkPointNaN(input_pointcloud->at(i)))
            {
                centroid.add(input_pointcloud->at(i));
            }
        }
        
        centroid.get(point_centroid);

        return point_centroid;
    }
    inline pcl::PointXYZ getCentroidPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, pcl::PointIndices::Ptr& input_indices)
    {
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        pcl::PointXYZ point_centroid;

        for (int i = 0; i < input_indices->indices.size(); i++)
        {
            if (!checkPointNaN(input_pointcloud->at(i)))
            {
                centroid.add(input_pointcloud->at(input_indices->indices.at(i)));
            }
        }
        
        centroid.get(point_centroid);

        return point_centroid;
    }
    
    // Adds highest negative z value of the pointcloud, to each point of a pointcloud, so minimum z value will be 0.0.
    inline void moveUp(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        pcl::PointXYZ point_z_min = getPointMinZ(input);

        if (input->isOrganized())
        {
            int input_row = input->height;
            int input_colum = input->width;

            for (int r = 0; r < input_row; r++)
            {
                for (int c = 0; c < input_colum; c++)
                {
                    if (!checkPointNaN(input->at(c,r)))
                    {                
                        input->at(c,r).z = input->at(c,r).z + fabsf(point_z_min.z);
                    }
                }
            }
        }
        else
        {
            int input_length = input->size();

            for (int i = 0; i < input_length; i++)
            {
                if (!checkPointNaN(input->at(i)))
                {
                    input->at(i).z = input->at(i).z + fabsf(point_z_min.z);
                }
            }
        }
    }

    // Mirrors the pointcloud in z direction.
    inline void mirrorPointcloudZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        int input_length = input->size();
        pcl::PointXYZ point_temp;

        for (int i = 0; i < input_length; i++)
        {
            point_temp = input->at(i);
            point_temp.z = point_temp.z * -1;
            input->push_back(point_temp);
        }
    }

    // Rotates the given pointcloud.
    inline void rectifyTilt(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        transform.translation() << input->at(100, 100).x, input->at(100, 100).y, input->at(100, 100).z;
        transform.rotate(Eigen::AngleAxisf((-187*M_PI) / 180, Eigen::Vector3f::UnitX()));
        transform.rotate(Eigen::AngleAxisf((0*M_PI) / 180, Eigen::Vector3f::UnitY()));
        transform.rotate(Eigen::AngleAxisf((0*M_PI) / 180, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud(*input, *input, transform);
    }

    // This function uses RANSAC to remove all points representing the groundplane form a pointcloud.
    inline void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
    {
        // Ransac
        pcl::ModelCoefficients::Ptr coefficients_ptr (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices);

        // create RandomSampleConsensus object and compute the appropriated model
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.004);
        seg.setInputCloud(input);
        seg.segment(*inliers_ptr, *coefficients_ptr);
        
        pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
        extr_inliers_filter.setInputCloud(input);
        extr_inliers_filter.setIndices(inliers_ptr);
        extr_inliers_filter.setNegative(true);
        extr_inliers_filter.filter(*input);


        /*
        pcl::PassThrough<pcl::PointXYZ> pass_filter;
        pass_filter.setInputCloud(input);
        pass_filter.setFilterFieldName("z");
        pass_filter.setFilterLimits(getPointMaxZ(input, inliers_ptr).z, 1.0);
        pass_filter.setNegative(false);
        pass_filter.filter(*input);
        */
    }

    inline void addCropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
    {
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(input);
        crop_box_filter.setNegative(false);
        crop_box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        crop_box_filter.filter(*input);
    }

    inline pcl::PointCloud<pcl::PointXYZ>::Ptr getCropBoxPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, float x_pos, float y_pos, float x_length, float y_length, double rotation_z, bool center)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr {new pcl::PointCloud<pcl::PointXYZ>};
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        Eigen::Vector3f vec_translation(x_pos, y_pos, 0.0);
        Eigen::Vector3f vec_rotation(0.0, 0.0, degreesToRadians(rotation_z));

        crop_box_filter.setInputCloud(input);
        crop_box_filter.setNegative(false);
        if (center)
        {
            crop_box_filter.setMin(Eigen::Vector4f(-(x_length / 2), -(y_length / 2), -10.0, 1.0));
            crop_box_filter.setMax(Eigen::Vector4f(x_length / 2, y_length / 2 , 10.0, 1.0));
        }
        else
        {
            crop_box_filter.setMin(Eigen::Vector4f(0.0, -(y_length / 2), -10.0, 1.0));
            crop_box_filter.setMax(Eigen::Vector4f(x_length, y_length / 2, 10.0, 1.0));
        }
        
        crop_box_filter.setRotation(vec_rotation);
        crop_box_filter.setTranslation(vec_translation);
        crop_box_filter.filter(*pointcloud_ptr);

        return pointcloud_ptr;
    }

    inline Eigen::Quaternionf getPointOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, float input_box_size, pcl::PointXYZ input_center_point)
    {
        Eigen::Matrix3f matrix;
        Eigen::Quaternionf quaternion;
        pcl::PointXYZ point_max_avg_z_box (0.0, 0.0, 0.0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_max_avg_z_ptr (new pcl::PointCloud<pcl::PointXYZ>);

        float rotation_temp;
        
        for (int i = 0; i <= 359; i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_temp_ptr (getCropBoxPointcloud(input_pointcloud, input_center_point.x, input_center_point.y, input_box_size, input_box_size / 4, i, false));
            pcl::PointXYZ point_avg_temp = getCentroidPoint(pointcloud_temp_ptr);
            publisher_pointcloud_orientation_scan.publish(pointcloud_temp_ptr);
            if (point_avg_temp.z > point_max_avg_z_box.z)
            {
                point_max_avg_z_box = point_avg_temp;
                pointcloud_max_avg_z_ptr = pointcloud_temp_ptr;
                rotation_temp = i;
            }
        }

        publisher_pointcloud_orientation_scan.publish(pointcloud_max_avg_z_ptr);

        matrix <<   0, 1, 0,
                    -1, 0, 0,
                    0, 0, 1;

        return Eigen::Quaternionf(fixMatrixY(rotateMatrixZ(matrix, rotation_temp)));
    }

    // This function uses RANSAC to fit a cylinder in to the given pointcloud.
    inline void findCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::ModelCoefficients::Ptr& output_coefficients, pcl::PointIndices::Ptr& output_point_indicies)
    {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        //Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);
        ne.setNumberOfThreads(8);
        ne.setKSearch(int_setKSearch);
        ne.setInputCloud(input);
        ne.setSearchMethod(tree);
        ne.compute(*cloud_normals_ptr);

        // publish pointcloud to "pclEditNormals" topic
        publisher_pointcloud_normals.publish(*cloud_normals_ptr);

        // create RandomSampleConsensus object and compute the appropriated model
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg (true);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (double_setNormalDistanceWeight);
        seg.setMaxIterations (30000);
        seg.setDistanceThreshold(double_setDistanceThreshold);
        seg.setRadiusLimits(double_setMinRadius, double_setMaxRadius);
        //seg.setRadiusLimits(0.0002, 0.0008);
        //seg.setRadiusLimits(0.02, 0.04);
        //seg.setAxis(axis);
        //seg.setEpsAngle(10.0f * (M_PI/180.0f));
        seg.setInputCloud(input);
        seg.setInputNormals(cloud_normals_ptr);
        seg.setNumberOfThreads(8);
        ROS_INFO_STREAM("Calculate cylinder...");
        seg.segment(*output_point_indicies, *output_coefficients);

        ROS_INFO_STREAM("Cylinder coefficients (x, y, z, ax, ay, az): ");
        for (int i = 0; i < 7; i++)
        {
            ROS_INFO_STREAM(i+1 << " = " << output_coefficients->values.at(i));
        }
        
        ROS_INFO_STREAM("Cylinder inliers: " << output_point_indicies->indices.size());
    }

    // This function sets a sphere marker to the given point.
    inline void setSphereMarker(pcl::PointXYZ input_point, std::string input_namespace, std::string input_frame, float input_color_r, float input_color_g, float input_color_b, float input_color_a)
    {
        visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

        marker_ptr->header.frame_id = input_frame;
        marker_ptr->header.stamp = ros::Time::now();
        marker_ptr->type = visualization_msgs::Marker::SPHERE;
        marker_ptr->ns = input_namespace;
        marker_ptr->id = 0;
        marker_ptr->action = visualization_msgs::Marker::ADD;

        // Set the position and orientation of the marker
        marker_ptr->pose.position.x = input_point.x;
        marker_ptr->pose.position.y = input_point.y;
        marker_ptr->pose.position.z = input_point.z;
        marker_ptr->pose.orientation.x = 0.0;
        marker_ptr->pose.orientation.y = 0.0;
        marker_ptr->pose.orientation.z = 0.0;
        marker_ptr->pose.orientation.w = 1.0;
    
        // Set the scale of the marker
        marker_ptr->scale.x = 0.005;
        marker_ptr->scale.y = 0.005;
        marker_ptr->scale.z = 0.005;
    
        // Set the color of the marker
        marker_ptr->color.r = input_color_r;
        marker_ptr->color.g = input_color_g;
        marker_ptr->color.b = input_color_b;
        marker_ptr->color.a = input_color_a;

        marker_ptr->lifetime = ros::Duration();

        publisher_vis_marker.publish(marker_ptr);
    }

    // This function removes the sphere marker.
    inline void removeSphereMarker(std::string input_namespace, std::string input_frame)
    {
        visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

        marker_ptr->header.frame_id = input_frame;
        marker_ptr->header.stamp = ros::Time::now();
        marker_ptr->type = visualization_msgs::Marker::SPHERE;
        marker_ptr->ns = input_namespace;
        marker_ptr->id = 0;
        marker_ptr->action = visualization_msgs::Marker::DELETEALL;

        publisher_vis_marker.publish(marker_ptr);
    }

    // This function sets a cylinder marker to the given coefficients.
    inline void setCylinderMarker(pcl::ModelCoefficients::Ptr& input_coefficients, std::string input_frame)
    {    
        visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

        Eigen::Quaternionf quaternion_from_coefficients;

        quaternion_from_coefficients = obtainCylinderOrientationFromModel(*input_coefficients);

        marker_ptr->header.frame_id = input_frame;
        marker_ptr->header.stamp = ros::Time::now();
        marker_ptr->type = visualization_msgs::Marker::CYLINDER;
        marker_ptr->ns = "cylinder";
        marker_ptr->id = 0;
        marker_ptr->action = visualization_msgs::Marker::ADD;

        // Set the position and orientation of the marker
        marker_ptr->pose.position.x = input_coefficients->values.at(0);
        marker_ptr->pose.position.y = input_coefficients->values.at(1);
        marker_ptr->pose.position.z = input_coefficients->values.at(2);
        marker_ptr->pose.orientation.x = quaternion_from_coefficients.x();
        marker_ptr->pose.orientation.y = quaternion_from_coefficients.y();
        marker_ptr->pose.orientation.z = quaternion_from_coefficients.z();
        marker_ptr->pose.orientation.w = quaternion_from_coefficients.w();
    
        // Set the scale of the marker
        marker_ptr->scale.x = input_coefficients->values.at(6)*2;
        marker_ptr->scale.y = input_coefficients->values.at(6)*2;
        marker_ptr->scale.z = 20.0;
    
        // Set the color of the marker
        marker_ptr->color.r = 1.0f;
        marker_ptr->color.g = 1.0f;
        marker_ptr->color.b = 1.0f;
        marker_ptr->color.a = 0.6;

        marker_ptr->lifetime = ros::Duration();

        publisher_vis_marker.publish(marker_ptr);
    }

    // This function removes the existing cylinder marker.
    inline void removeCylinderMarker(std::string input_frame)
    {
        visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

        marker_ptr->header.frame_id = input_frame;
        marker_ptr->header.stamp = ros::Time::now();
        marker_ptr->type = visualization_msgs::Marker::CYLINDER;
        marker_ptr->ns = "cylinder";
        marker_ptr->id = 0;
        marker_ptr->action = visualization_msgs::Marker::DELETEALL;

        publisher_vis_marker.publish(marker_ptr);
    }

    // This function sets a points marker to the given point indices.
    inline void setPointListMarker(pcl::PointIndices::Ptr& input_point_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, std::string input_namespace, std::string input_frame, float input_color_r, float input_color_g, float input_color_b, float input_color_a)
    {    
        visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

        marker_ptr->header.frame_id = input_frame;
        marker_ptr->header.stamp = ros::Time::now();
        marker_ptr->type = visualization_msgs::Marker::POINTS;
        marker_ptr->ns = input_namespace;
        marker_ptr->id = 0;
        marker_ptr->action = visualization_msgs::Marker::ADD;

        int input_point_indices_size = input_point_indices->indices.size();

        for (int i = 0; i < input_point_indices_size; i++)
        {
            geometry_msgs::Point p;

            p.x = input_pointcloud->at(input_point_indices->indices.at(i)).x;
            p.y = input_pointcloud->at(input_point_indices->indices.at(i)).y;
            p.z = input_pointcloud->at(input_point_indices->indices.at(i)).z;
            //ROS_INFO_STREAM("Inlier: " << i+1 << " - " << "x: " << p.x << " ,y: " << p.y << " ,z: " << p.z);
            marker_ptr->points.push_back(p);
        }
    
        // Set the scale of the marker
        marker_ptr->scale.x = 0.003;
        marker_ptr->scale.y = 0.003;
        marker_ptr->scale.z = 0.003;
    
        // Set the color of the marker
        marker_ptr->color.r = input_color_r;
        marker_ptr->color.g = input_color_g;
        marker_ptr->color.b = input_color_b;
        marker_ptr->color.a = input_color_a;

        marker_ptr->lifetime = ros::Duration();

        publisher_vis_marker.publish(marker_ptr);
    }

    // This function removes the existing points marker.
    inline void removePointListMarker(std::string input_namespace, std::string input_frame)
    {    
        visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

        marker_ptr->header.frame_id = input_frame;
        marker_ptr->header.stamp = ros::Time::now();
        marker_ptr->type = visualization_msgs::Marker::POINTS;
        marker_ptr->ns = input_namespace;
        marker_ptr->id = 0;
        marker_ptr->action = visualization_msgs::Marker::DELETEALL;

        publisher_vis_marker.publish(marker_ptr);
    }

    // This function removes every marker set before.
    inline void removeAllMarkers()
    {
        removeCylinderMarker(string_frame);
        removeSphereMarker("z_max_all", string_frame);
        removeSphereMarker("z_max_cylinder", string_frame);
        removeSphereMarker("centroid_all", string_frame);
        removeSphereMarker("centroid_cylinder_inliers", string_frame);
        removePointListMarker("cylinder_inliers", string_frame);
        removePointListMarker("cylinder_neighbors", string_frame);
        removePointListMarker("max_z_neighbors", string_frame);
    }

    GraspPointActionServer(std::string name) :
        as_(n, name, boost::bind(&GraspPointActionServer::callbackAction, this, _1), false), action_name_(name)
    {
        dynamic_reconfigure::Server<fabric_grasping::parametersConfig> server;
        dynamic_reconfigure::Server<fabric_grasping::parametersConfig>::CallbackType f;

        f = boost::bind(&GraspPointActionServer::callbackDynamicReconfigure, this, _1, _2);
        server.setCallback(f);

        publisher_pointcloud = n.advertise<sensor_msgs::PointCloud2>("pointcloud_edited", 1);
        publisher_pointcloud_normals = n.advertise<sensor_msgs::PointCloud2>("pointcloud_edited_normals", 1);
        publisher_pointcloud_debug = n.advertise<sensor_msgs::PointCloud2>("pointcloud_debug", 1);
        publisher_pointcloud_orientation_scan = n.advertise<sensor_msgs::PointCloud2>("pointcloud_orientation_scan", 1);
        publisher_vis_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        publisher_pose = n.advertise<geometry_msgs::PoseStamped>("grasping_pose", 1);
        
        tf2_ros::TransformListener tfListener(tf_buffer);

        as_.start();
        ros::spin();
    }

    ~GraspPointActionServer(void)
    {
    }

    void callbackAction(const fabric_grasping::GetGraspPointGoalConstPtr &goal)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_pc;
        pcl::PointIndices::Ptr cylinder_inliers_ptr (new pcl::PointIndices);
        pcl::PointIndices::Ptr cylinder_max_z_neighbors_ptr (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr cylinder_coefficients_ptr (new pcl::ModelCoefficients);
        pcl::PointXYZ point_max_z_cylinder;
        pcl::PointXYZ point_centroid_cylinder;
        pcl::PointXYZ point_max_z_all;
        pcl::PointXYZ point_centroid_all;
        geometry_msgs::PoseStamped pose_result;
        geometry_msgs::PoseStamped pose_result_copy;
        Eigen::Quaternionf quaternion_cylinder;
        Eigen::Quaternionf quaternion_point_z_all;
        Eigen::Quaternionf quaternion_point_centroid_all;
        sensor_msgs::PointCloud2ConstPtr sen_msg_pc2_ptr;
        sensor_msgs::PointCloud2 sen_msg_pc2_tf;
        auto begin = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

        sen_msg_pc2_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/depth/points", ros::Duration(2));
        
        if(sen_msg_pc2_ptr == NULL)
        {
            ROS_INFO("No point clound message received");
            return;
        }
        else
        {
            ROS_INFO("Point clound message received");
            ros::Duration(1.0).sleep();
            pcl_ros::transformPointCloud(string_frame, *sen_msg_pc2_ptr, sen_msg_pc2_tf, tf_buffer);
            if (sen_msg_pc2_tf.data.size() < 1)
            {
                as_.setAborted();
                return;
            }
            pcl::fromROSMsg(sen_msg_pc2_tf, *pcl_pc_ptr);

            //DEBUG publish pointcloud to "pointcloud_debug" topic
            publisher_pointcloud_debug.publish(pcl_pc_ptr);
        }

        removeAllMarkers();

        ROS_INFO_STREAM("Seq. Nr.: " << pcl_pc_ptr->header.seq);
        

        // fix tilted pointcloudros_info_stream
        if (string_frame == "depth_camera_link")
        {
            rectifyTilt(pcl_pc_ptr);

            moveUp(pcl_pc_ptr);
        }
        // remove every point from the pointcloud representing the groundplane
        removeGroundPlane(pcl_pc_ptr);

        if (bool_crop_box)
        {
            addCropBox(pcl_pc_ptr, 0.25, -0.50, 0.0, 0.60, 0.10, 0.25);
        }

        // publish pointcloud to "pclEdit" topic
        publisher_pointcloud.publish(*pcl_pc_ptr);

        if (!pcl_pc_ptr->empty())
        {
            // publish sphere marker at overall z max, color: purple
            point_max_z_all = getPointMaxZ(pcl_pc_ptr);
            setSphereMarker(point_max_z_all, "z_max_all", string_frame, 0.5, 0.0, 0.5, 1.0);

            // publish points marker at centroid overall points, color: green
            point_centroid_all = getCentroidPoint(pcl_pc_ptr);
            setSphereMarker(point_centroid_all, "centroid_all", string_frame, 0.0, 1.0, 0.0, 1.0);
            
            switch (int_gp_method)
            {
            case 0: // Max z all
                // start timer
                begin = std::chrono::high_resolution_clock::now();

                point_max_z_all = getPointMaxZ(pcl_pc_ptr);
                quaternion_point_z_all = getPointOrientation(pcl_pc_ptr, 0.02, point_max_z_all);
                pose_result.pose.position.x = point_max_z_all.x;
                pose_result.pose.position.y = point_max_z_all.y;
                pose_result.pose.position.z = std::max(point_max_z_all.z + 0.16 - 0.01, 0.16 + 0.01);
                pose_result.pose.orientation.w = quaternion_point_z_all.w();
                pose_result.pose.orientation.x = quaternion_point_z_all.x();
                pose_result.pose.orientation.y = quaternion_point_z_all.y();
                pose_result.pose.orientation.z = quaternion_point_z_all.z();
                pcl_conversions::fromPCL(pcl_pc_ptr->header, pose_result.header);

                // stop timer and get result
                end = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
                printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
                ROS_INFO_STREAM("Max. Z: " << point_max_z_all.z);

                pose_result_copy = pose_result;
                pose_result_copy.pose.position.z = std::max(point_max_z_all.z, 0.02f);

                publisher_pose.publish(pose_result_copy);
                result_.grasping_pose = pose_result;
                result_.grasping_width = 1.0;
                as_.setSucceeded(result_);
                ROS_INFO_STREAM("#### Done! ####");
                break;
            case 1: // Max z all fixed orientation
                // start timer
                begin = std::chrono::high_resolution_clock::now();

                point_max_z_all = getPointMaxZ(pcl_pc_ptr);
                //quaternion_point_z_all = getPointOrientation(pcl_pc_ptr, 0.02, point_max_z_all);
                pose_result.pose.position.x = point_max_z_all.x;
                pose_result.pose.position.y = point_max_z_all.y;
                pose_result.pose.position.z = std::max(point_max_z_all.z + 0.16 - 0.01, 0.16 + 0.01);
                pose_result.pose.orientation.w = -1;
                pose_result.pose.orientation.x = 0;
                pose_result.pose.orientation.y = 0;
                pose_result.pose.orientation.z = 0;
                pcl_conversions::fromPCL(pcl_pc_ptr->header, pose_result.header);

                // stop timer and get result
                end = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
                printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
                ROS_INFO_STREAM("Max. Z: " << point_max_z_all.z);

                pose_result_copy = pose_result;
                pose_result_copy.pose.position.z = std::max(point_max_z_all.z, 0.02f);

                publisher_pose.publish(pose_result_copy);
                result_.grasping_pose = pose_result;
                result_.grasping_width = 1.0;
                as_.setSucceeded(result_);
                ROS_INFO_STREAM("#### Done! ####");
                break;
            case 2: // Centroid all
                // start timer
                begin = std::chrono::high_resolution_clock::now();

                point_centroid_all = getCentroidPoint(pcl_pc_ptr);
                quaternion_point_centroid_all = getPointOrientation(pcl_pc_ptr, 0.02, point_centroid_all);
                pose_result.pose.position.x = point_centroid_all.x;
                pose_result.pose.position.y = point_centroid_all.y;
                pose_result.pose.position.z = std::max(point_centroid_all.z + 0.16 - 0.01, 0.16 + 0.01);
                pose_result.pose.orientation.w = quaternion_point_centroid_all.w();
                pose_result.pose.orientation.x = quaternion_point_centroid_all.x();
                pose_result.pose.orientation.y = quaternion_point_centroid_all.y();
                pose_result.pose.orientation.z = quaternion_point_centroid_all.z();
                pcl_conversions::fromPCL(pcl_pc_ptr->header, pose_result.header);

                // stop timer and get result
                end = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
                printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
                ROS_INFO_STREAM("Max. Z: " << point_max_z_all.z);
                ROS_INFO_STREAM("Centroid. Z: " << point_centroid_all.z);

                pose_result_copy = pose_result;
                pose_result_copy.pose.position.z = std::max(point_centroid_all.z, 0.02f);

                publisher_pose.publish(pose_result_copy);
                result_.grasping_pose = pose_result;
                result_.grasping_width = 1.0;
                as_.setSucceeded(result_);
                ROS_INFO_STREAM("#### Done! ####");
                break;
            case 3: // Cylinder max z
                // start timer
                begin = std::chrono::high_resolution_clock::now();
                
                // find cylinder and publish markers
                findCylinder(pcl_pc_ptr, cylinder_coefficients_ptr, cylinder_inliers_ptr);
                
                if (cylinder_inliers_ptr->indices.size() > 0)
                {
                    // publish cylinder marker
                    setCylinderMarker(cylinder_coefficients_ptr,  string_frame);

                    // publish points marker at cylinder inliers, color: blue
                    setPointListMarker(cylinder_inliers_ptr, pcl_pc_ptr, "cylinder_inliers", string_frame, 0.0, 0.0, 1.0, 1.0);

                    // publish sphere marker at cylinder inliers z max, color: blue
                    point_max_z_cylinder = getPointMaxZ(pcl_pc_ptr, cylinder_inliers_ptr);
                    setSphereMarker(point_max_z_cylinder, "z_max_cylinder", string_frame, 0.0, 0.0, 1.0, 1.0);

                    // publish sphere marker at cylinder inliers centroid color: red
                    point_centroid_cylinder = getCentroidPoint(pcl_pc_ptr, cylinder_inliers_ptr);
                    setSphereMarker(point_centroid_cylinder, "centroid_cylinder_inliers", string_frame, 1.0, 0.0, 0.0, 1.0);
                    quaternion_cylinder = obtainCylinderOrientationFromModel(*cylinder_coefficients_ptr);

                    Eigen::Quaternionf quaternion_cylinder_fixed(fixMatrixY(editMatrix(quaternion_cylinder.toRotationMatrix())));

                    pose_result.pose.position.x = point_max_z_cylinder.x;
                    pose_result.pose.position.y = point_max_z_cylinder.y;
                    pose_result.pose.position.z = std::max(point_max_z_cylinder.z + 0.16 - 0.01, 0.16 + 0.01);
                    pose_result.pose.orientation.w = quaternion_cylinder_fixed.w();
                    pose_result.pose.orientation.x = quaternion_cylinder_fixed.x();
                    pose_result.pose.orientation.y = quaternion_cylinder_fixed.y();
                    pose_result.pose.orientation.z = quaternion_cylinder_fixed.z();
                    pcl_conversions::fromPCL(pcl_pc_ptr->header, pose_result.header);

                    // stop timer and get result
                    end = std::chrono::high_resolution_clock::now();
                    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
                    printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
                    ROS_INFO_STREAM("Max. Z: " << point_max_z_all.z);
                    ROS_INFO_STREAM("Max. Z Cylinder: " << point_max_z_cylinder.z);

                    pose_result_copy = pose_result;
                    pose_result_copy.pose.position.z = std::max(point_max_z_cylinder.z, 0.02f);

                    publisher_pose.publish(pose_result_copy);
                    result_.grasping_pose = pose_result;
                    result_.grasping_width = cylinder_coefficients_ptr->values.at(6);
                    as_.setSucceeded(result_);
                    ROS_INFO_STREAM("#### Done! ####");
                    break;
                }
                ROS_INFO("No Cylinder found!");
                as_.setAborted(result_);
                break;
            case 4: // Cylinder centroid
                // start timer
                begin = std::chrono::high_resolution_clock::now();
                
                // find cylinder and publish markers
                findCylinder(pcl_pc_ptr, cylinder_coefficients_ptr, cylinder_inliers_ptr);
                
                if (cylinder_inliers_ptr->indices.size() > 0)
                {
                    // publish cylinder marker
                    setCylinderMarker(cylinder_coefficients_ptr,  string_frame);

                    // publish points marker at cylinder inliers, color: blue
                    setPointListMarker(cylinder_inliers_ptr, pcl_pc_ptr, "cylinder_inliers", string_frame, 0.0, 0.0, 1.0, 1.0);

                    // publish sphere marker at cylinder inliers z max, color: blue
                    point_max_z_cylinder = getPointMaxZ(pcl_pc_ptr, cylinder_inliers_ptr);
                    setSphereMarker(point_max_z_cylinder, "z_max_cylinder", string_frame, 0.0, 0.0, 1.0, 1.0);

                    // publish sphere marker at cylinder inliers centroid color: red
                    point_centroid_cylinder = getCentroidPoint(pcl_pc_ptr, cylinder_inliers_ptr);
                    setSphereMarker(point_centroid_cylinder, "centroid_cylinder_inliers", string_frame, 1.0, 0.0, 0.0, 1.0);
                    quaternion_cylinder = obtainCylinderOrientationFromModel(*cylinder_coefficients_ptr);

                    Eigen::Quaternionf quaternion_cylinder_fixed(fixMatrixY(editMatrix(quaternion_cylinder.toRotationMatrix())));

                    pose_result.pose.position.x = point_centroid_cylinder.x;
                    pose_result.pose.position.y = point_centroid_cylinder.y;
                    pose_result.pose.position.z = std::max(point_centroid_cylinder.z + 0.16 - 0.01, 0.16 + 0.01);
                    pose_result.pose.orientation.w = quaternion_cylinder_fixed.w();
                    pose_result.pose.orientation.x = quaternion_cylinder_fixed.x();
                    pose_result.pose.orientation.y = quaternion_cylinder_fixed.y();
                    pose_result.pose.orientation.z = quaternion_cylinder_fixed.z();
                    pcl_conversions::fromPCL(pcl_pc_ptr->header, pose_result.header);
                    
                    // stop timer and get result
                    end = std::chrono::high_resolution_clock::now();
                    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
                    printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
                    ROS_INFO_STREAM("Max. Z: " << point_max_z_all.z);
                    ROS_INFO_STREAM("Centroid Z Cylinder: " << point_centroid_cylinder.z);
                    
                    pose_result_copy = pose_result;
                    pose_result_copy.pose.position.z = std::max(point_centroid_cylinder.z, 0.02f);

                    publisher_pose.publish(pose_result_copy);
                    result_.grasping_pose = pose_result;
                    result_.grasping_width = cylinder_coefficients_ptr->values.at(6);
                    as_.setSucceeded(result_);
                    ROS_INFO_STREAM("#### Done! ####");
                    break;
                }
                ROS_INFO("No Cylinder found!");
                as_.setAborted(result_);
                break;
            default:
                break;
            }
        }
        else
        {
            ROS_INFO("PointCloud is empty! Aborted");
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting listener node...");
    
    ros::init(argc, argv, "listener");
    
    GraspPointActionServer get_grasping_point("get_grasping_point");

    return 0;
}