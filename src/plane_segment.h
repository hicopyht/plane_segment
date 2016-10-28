#ifndef PLANE_SEGMENT_H
#define PLANE_SEGMENT_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
//
#include <pcl/console/time.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <plane_segment/PlaneSegmentConfig.h>
#include <plane_segment/LineBasedSegmentConfig.h>
#include <plane_segment/RansacSegmentConfig.h>
#include <plane_segment/OrganizedSegmentConfig.h>
#include <plane_segment/RegionGrowSegmentConfig.h>
#include <plane_from_line/plane_from_line_segment.h>

#include "plane_viewer.h"
#include "organized_plane_segment.h"
#include "utils.h"

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2,
                                                        sensor_msgs::CameraInfo> CloudSyncPolicy;

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> NoCloudSyncPolicy;


class PlaneSegment
{
public:
    enum { VGA = 0, QVGA = 1, QQVGA = 2};
    enum { NONE_METHOD = 0, LINE_BADED = 1, RANSAC = 2, ORGANSIZED = 3, REGION_GROW = 4, ALL_EXCEPT_RG = 5, LB_OMPS = 6, ALL_METHOD = 7};
public:
    PlaneSegment();

    inline void setLoopMessage( bool loop_one_message) { loop_one_message_ = loop_one_message; }

    void processCloud( PointCloudTypePtr &input );

    void setlineBasedPlaneSegmentParameters();

    void lineBasedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes);

    void lineBasedPlaneSegment(PointCloudTypePtr &input,
                               std::vector<PlaneFromLineSegment::LineType>& lines,
                               std::vector<PlaneFromLineSegment::NormalType>& normals,
                               std::vector<PlaneType> &planes,
                               vector<float>& runtimes);

    void ransacPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes);

    void organizedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes);

    void regionGrowPlaneSegment(PointCloudTypePtr &input,
                                std::vector<PlaneType> &planes,
                                PointCloudTypePtr &colored_cloud);

    void computePlane(PointCloudTypePtr &input, std::vector<int> &inliers, PlaneType &plane);

    void flipPlaneTowardsViewpoint (const PointType &point,
                                    float vp_x, float vp_y, float vp_z,
                                    float &nx, float &ny, float &nz);

    bool planeFromPoint(PointCloudTypePtr &input, int index, PlaneType &plane);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( const sensor_msgs::ImageConstPtr& visual_img_msg,
                                                                const sensor_msgs::ImageConstPtr& depth_img_msg,
                                                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg,
                                                                PlaneFromLineSegment::CAMERA_PARAMETERS &camera_parameters,
                                                                pcl::PointIndicesPtr &indices);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                              const PlaneFromLineSegment::CAMERA_PARAMETERS& camera,
                                                              pcl::PointIndicesPtr &indices);


    void downsampleOrganizedCloud( const PointCloudTypePtr &input, PointCloudTypePtr &output,
                                   PlaneFromLineSegment::CAMERA_PARAMETERS &out_camera, int size_type);

    inline void setCameraParameter(PlaneFromLineSegment::CAMERA_PARAMETERS &camera )
    {
        camera_parameters_ = camera;
    }

protected:
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                         const sensor_msgs::ImageConstPtr& depth_img_msg,
                         const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                        const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void planeSegmentReconfigCallback( plane_segment::PlaneSegmentConfig &config, uint32_t level);

    void lineBasedSegmentReconfigCallback( plane_segment::LineBasedSegmentConfig &config, uint32_t level);

    void ransacSegmentReconfigCallback( plane_segment::RansacSegmentConfig &config, uint32_t level);

    void organizedSegmentReconfigCallback( plane_segment::OrganizedSegmentConfig &config, uint32_t level);

    void regionGrowSegmentReconfigCallback( plane_segment::RegionGrowSegmentConfig &config, uint32_t level);


    void getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                            PlaneFromLineSegment::CAMERA_PARAMETERS &camera);


    void printPlaneCoefficients(const std::vector<PlaneType> &planes, const std::string &prefix);

    void pubScanPoint(vector<ScanPoint> &scan_points);

    void evaluateRuntimes(float &line, float &normal, float &plane, float &total, float &ransac, float &rgs, float &omps, float &proposed);

protected:
    inline bool isValidPoint(const PointType &p)
    {
        return (prttcp_->isValid(p) && p.z > 0);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::CallbackQueue my_callback_queue_;
    ros::AsyncSpinner* async_spinner_;
    //
    dynamic_reconfigure::Server<plane_segment::PlaneSegmentConfig> plane_segment_config_server_;
    dynamic_reconfigure::Server<plane_segment::PlaneSegmentConfig>::CallbackType plane_segment_config_callback_;
    dynamic_reconfigure::Server<plane_segment::LineBasedSegmentConfig> line_based_segment_config_server_;
    dynamic_reconfigure::Server<plane_segment::LineBasedSegmentConfig>::CallbackType line_based_segment_config_callback_;
    dynamic_reconfigure::Server<plane_segment::RansacSegmentConfig> ransac_segment_config_server_;
    dynamic_reconfigure::Server<plane_segment::RansacSegmentConfig>::CallbackType ransac_segment_config_callback_;
    dynamic_reconfigure::Server<plane_segment::OrganizedSegmentConfig> organized_segment_config_server_;
    dynamic_reconfigure::Server<plane_segment::OrganizedSegmentConfig>::CallbackType organized_segment_config_callback_;
    dynamic_reconfigure::Server<plane_segment::RegionGrowSegmentConfig> region_grow_segment_config_server_;
    dynamic_reconfigure::Server<plane_segment::RegionGrowSegmentConfig>::CallbackType region_grow_segment_config_callback_;

    // subscribers
    int subscriber_queue_size_;
    std::string topic_image_visual_;
    std::string topic_image_depth_;
    std::string topic_camera_info_;
    std::string topic_point_cloud_;
    //
    message_filters::Subscriber<sensor_msgs::Image> *visual_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    //
    message_filters::Synchronizer<CloudSyncPolicy>* cloud_sync_;
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    //
    tf::TransformListener tf_listener_;
    ros::Publisher scan_point_publisher_;
    //
    PointRepresentationConstPtr prttcp_;
    //
    PlaneViewer *viewer_;

    // Plane segment
    int cloud_size_type_;
    int cloud_size_type_config_;
    int plane_segment_method_;
    bool print_plane_coefficients_;
    bool evaluate_run_time_;
    bool loop_one_message_;
    int loop_message_;
    double voxel_grid_downsample_size_;

    //
    bool is_update_line_based_parameters_;
    // LineBased segment
    bool use_horizontal_line_;
    bool use_verticle_line_;
    unsigned y_skip_;
    unsigned x_skip_;
    float line_point_min_distance_;
    bool use_depth_noise_model_;
    float scan_rho_constant_error_;
    float scan_rho_distance_error_;
    float scan_rho_quadratic_error_;
    unsigned slide_window_size_;
    unsigned line_min_inliers_;
    float line_fitting_threshold_;
    int normals_per_line_;
    bool normal_use_depth_dependent_smoothing_;
    float normal_max_depth_change_factor_;
    unsigned normal_smoothing_size_;
    float normal_min_inliers_percentage_;
    float normal_maximum_curvature_;
    //
    bool remove_duplicate_candidate_;
    float duplicate_candidate_normal_thresh_;
    float duplicate_candidate_distance_thresh_;
    //
    int plane_segment_criterion_;
    float k_curvature_;
    float k_inlier_;
    unsigned min_inliers_;
    float max_curvature_;
    float distance_threshold_;
    float neighbor_threshold_;
    bool optimize_coefficients_;
    bool project_points_;
    bool extract_boundary_;

    // Plane segment based line segment
    PlaneFromLineSegment::CAMERA_PARAMETERS camera_parameters_;
    PlaneFromLineSegment plane_from_line_segment_;
    //
    bool is_extract_single_plane_;
    int single_plane_row_;
    int single_plane_col_;

    // Organized Muit Plane segment parameters
    OrganizedPlaneSegment organized_plane_segment_;

    // RANSAC segment parameters
    bool ransac_use_downsample_;
    int ransac_method_type_;
    double ransac_points_left_persentage_;
    double ransac_distance_threshold_;
    int ransac_max_iterations_;
    int ransac_min_points_size_;

    // RegionGrow segment parameters
    pcl::PointIndicesPtr region_grow_cloud_indices_;
    bool rg_use_downsample_;
    int rg_normal_k_search_;
    int rg_min_cluster_size_;
    int rg_max_cluster_size_;
    int rg_number_of_neighbours_;
    double rg_smoothness_thresh_;
    double rg_curvature_thresh_;

};

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

#endif // PLANE_SEGMENT_H
