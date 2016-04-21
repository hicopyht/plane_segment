#include "plane_segment.h"

using namespace std;

PlaneSegment::PlaneSegment() :
    private_nh_("~")
  , plane_segment_config_server_( ros::NodeHandle( "PlaneSegment" ) )
  , line_based_segment_config_server_( ros::NodeHandle( "LineBasedSegment" ) )
  , ransac_segment_config_server_( ros::NodeHandle( "RansacSegment" ) )
  , organized_segment_config_server_( ros::NodeHandle( "OrganizedSegment" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , pcl_viewer_( new pcl::visualization::PCLVisualizer("3D Viewer"))
  , viewer_v1_(1)
  , viewer_v2_(2)
  , viewer_v3_(3)
  , viewer_v4_(4)
  , rng(12345)
  , prttcp_ (new pcl::DefaultPointRepresentation<PointType>)
  , camera_parameters_()
  , plane_from_line_segment_()
  , organized_plane_segment_()
  , single_plane_row_( 200 )
  , single_plane_col_( 200 )
{
    nh_.setCallbackQueue(&my_callback_queue_);

    // reconfigure
    bool use_reconfigure;
    private_nh_.param<bool>("use_reconfigure", use_reconfigure, true);
    if(use_reconfigure)
    {
        plane_segment_config_callback_ = boost::bind(&PlaneSegment::planeSegmentReconfigCallback, this, _1, _2);
        plane_segment_config_server_.setCallback(plane_segment_config_callback_);
        line_based_segment_config_callback_ = boost::bind(&PlaneSegment::lineBasedSegmentReconfigCallback, this, _1, _2);
        line_based_segment_config_server_.setCallback(line_based_segment_config_callback_);
        ransac_segment_config_callback_ = boost::bind(&PlaneSegment::ransacSegmentReconfigCallback, this, _1, _2);
        ransac_segment_config_server_.setCallback(ransac_segment_config_callback_);
        organized_segment_config_callback_ = boost::bind(&PlaneSegment::organizedSegmentReconfigCallback, this, _1, _2);
        organized_segment_config_server_.setCallback(organized_segment_config_callback_);

    }

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/rgb/camera_info");
    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

    pcl_viewer_->createViewPort(0, 0, 0.5, 0.5, viewer_v1_);
    pcl_viewer_->addText("LinesAndNormals", 100, 3, "v1_text", viewer_v1_);
    pcl_viewer_->createViewPort(0.5, 0, 1.0, 0.5, viewer_v2_);
    pcl_viewer_->addText("LineBasedPlanes", 100, 3, "v2_text", viewer_v2_);
    pcl_viewer_->createViewPort(0, 0.5, 0.5, 1.0, viewer_v3_);
    pcl_viewer_->addText("RansacPlanes", 100, 3, "v3_text", viewer_v3_);
    pcl_viewer_->createViewPort(0.5, 0.5, 1.0, 1.0, viewer_v4_);
    pcl_viewer_->addText("OrganizedPlanes", 100, 3, "v4_text", viewer_v4_);
    pcl_viewer_->addCoordinateSystem(0.000001);
    pcl_viewer_->initCameraParameters();
    pcl_viewer_->setCameraPosition(0.0, 0.0, -0.4, 0, 0, 0.6, 0, -1, 0);
    pcl_viewer_->setShowFPS(true);

    // config subscribers
    if(!topic_point_cloud_.empty()) // pointcloud2
    {
        // use visual image, depth image, pointcloud2
        visual_sub_ = new image_sub_type(nh_, topic_image_visual_, subscriber_queue_size_);
        cloud_sub_ = new pc_sub_type (nh_, topic_point_cloud_, subscriber_queue_size_);
        cinfo_sub_ = new cinfo_sub_type(nh_, topic_camera_info_, subscriber_queue_size_);
        cloud_sync_ = new message_filters::Synchronizer<CloudSyncPolicy>(CloudSyncPolicy(subscriber_queue_size_),  *visual_sub_, *cloud_sub_, *cinfo_sub_),
        cloud_sync_->registerCallback(boost::bind(&PlaneSegment::cloudCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << topic_image_visual_ << ", " << topic_point_cloud_ << " and " << topic_camera_info_ << ".");
    }
    else if(!topic_camera_info_.empty())
    {
        //No cloud, use visual image, depth image, camera_info
        visual_sub_ = new image_sub_type(nh_, topic_image_visual_, subscriber_queue_size_);
        depth_sub_ = new image_sub_type (nh_, topic_image_depth_, subscriber_queue_size_);
        cinfo_sub_ = new cinfo_sub_type(nh_, topic_camera_info_, subscriber_queue_size_);
        no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(subscriber_queue_size_),  *visual_sub_, *depth_sub_, *cinfo_sub_),
        no_cloud_sync_->registerCallback(boost::bind(&PlaneSegment::noCloudCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << topic_image_visual_ << ", " << topic_image_depth_ << " and " << topic_camera_info_ << ".");
    }
    else
    {
        ROS_ERROR("Can not decide subscriber type");
        exit(1);
    }

    async_spinner_ =  new ros::AsyncSpinner(4, &my_callback_queue_);
    async_spinner_->start();

}


void PlaneSegment::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::ImageConstPtr& depth_img_msg,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("no cloud msg: %d\n", visual_img_msg->header.seq);
    getCameraParameter( cam_info_msg, camera_parameters_);
    PointCloudTypePtr cloud_in ( new PointCloudType );
    // Get Mat Image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    // Get PointCloud
    cloud_in = image2PointCloud( visual_image, depth_image, camera_parameters_);

    if(!loop_one_message_)
        processCloud( cloud_in );
    else
        while(loop_one_message_ && ros::ok())
        {
            ros::Duration(0.2).sleep();
            processCloud( cloud_in );
        }
}

void PlaneSegment::cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("cloud msg: %d\n", visual_img_msg->header.seq);

    getCameraParameter( cam_info_msg, camera_parameters_);

    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::fromROSMsg( *point_cloud, *cloud_in);

    if(!loop_one_message_)
        processCloud( cloud_in );
    else
        while(loop_one_message_ && ros::ok())
        {
            ros::Duration(0.2).sleep();
            processCloud( cloud_in );
        }
}

void PlaneSegment::processCloud( PointCloudTypePtr &input )
{
    std::vector<PlaneFromLineSegment::LineType> lines;
    std::vector<PlaneFromLineSegment::NormalType> normals;
    std::vector<PlaneType> line_based_planes;
    std::vector<PlaneType> ransac_planes;
    std::vector<PlaneType> organized_planes;
    std::vector<PlaneType> region_grow_planes;

    double start_time = pcl::getTime();
    pcl::console::TicToc time;
    time.tic();
    float line_base_dura = 0, ransac_dura = 0;
    float organized_dura = 0, region_grow_dura = 0;
    float single_plane_dura = 0, display_dura = 0;
    float total_dura = 0;

    // do segmentation
    if(plane_segment_method_ == LINE_BADED)
    {
        lineBasedPlaneSegment( input, line_based_planes );
        line_base_dura = time.toc();
        time.tic();
    }
    else if(plane_segment_method_ == RANSAC)
    {
        ransacPlaneSegment( input, ransac_planes );
        ransac_dura = time.toc();
        time.tic();
    }
    else if(plane_segment_method_ == ORGANSIZED)
    {
        organizedPlaneSegment( input, organized_planes );
        organized_dura = time.toc();
        time.tic();
    }
    else if(plane_segment_method_ == REGION_GROW)
    {
        regionGrowPlaneSegment( input, region_grow_planes );
        region_grow_dura = time.toc();
        time.tic();
    }
    else if(plane_segment_method_ == ALL_METHOD)
    {
        //
        lineBasedPlaneSegment( input, line_based_planes );
        line_base_dura = time.toc();
        time.tic();

        ransacPlaneSegment( input, ransac_planes );
        ransac_dura = time.toc();
        time.tic();

        organizedPlaneSegment( input, organized_planes );
        organized_dura = time.toc();
        time.tic();

        regionGrowPlaneSegment( input, region_grow_planes );
        region_grow_dura = time.toc();
        time.tic();
    }
    //
    std::vector<PlaneType> single_plane;
    if(is_extract_single_plane_)
    {
        PlaneType sp;
        int idx = single_plane_row_ * camera_parameters_.width + single_plane_col_;
        cout << " - single idx: " << idx << endl;
        if( planeFromPoint( input, idx, sp ) )
        {
            single_plane.push_back( sp );
        }
    }
    single_plane_dura = time.toc();
    time.tic();
    //

    // display
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();
    displayLinesAndNormals( input, lines, normals, viewer_v1_);
    displayPlanes( input, single_plane, "single_plane", viewer_v1_ );
    displayPlanes( input, line_based_planes, "line_based_plane", viewer_v2_);
    displayPlanes( input, ransac_planes, "ransac_plane", viewer_v3_);
    displayPlanes( input, organized_planes, "organized_plane", viewer_v4_);
//    displayPlanes( input_, region_grow_planes, viewer_v1_);
    pcl_viewer_->spinOnce(1);

    display_dura = time.toc();
    total_dura = pcl::getTime() - start_time;

    cout << GREEN << " Time: LineB: " << line_base_dura << ", RANSAC: " << ransac_dura
         << ", Organized: " << organized_dura << ", RegionGrow: " << region_grow_dura
         << ", Single: " << single_plane_dura << RESET << endl;
    ROS_INFO("Total time: %f, display %f \n", total_dura, display_dura);
    cout << "----------------------------------- END -------------------------------------" << endl;
}

void PlaneSegment::lineBasedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    PointCloudTypePtr cloud_in (new PointCloudType);
    pcl::copyPointCloud( *input, *cloud_in);

    if (!plane_from_line_segment_.isInitialized())
    {
        plane_from_line_segment_.setCameraParameters( camera_parameters_ );
        cout << "Initialize line base segment." << endl;
    }

    //
    if( is_update_line_based_parameters_ )
    {
        plane_from_line_segment_.setUseHorizontalLines( use_horizontal_line_ );
        plane_from_line_segment_.setUseVerticleLines( use_verticle_line_ );
        plane_from_line_segment_.setYskip( y_skip_ );
        plane_from_line_segment_.setXSkip( x_skip_ );
        plane_from_line_segment_.setLinePointMinDistance( line_point_min_distance_ );
        plane_from_line_segment_.setRhoConstantError( scan_rho_constant_error_ );
        plane_from_line_segment_.setRhoDistanceError( scan_rho_distance_error_ );
        plane_from_line_segment_.setSlideWindowSize( slide_window_size_ );
        plane_from_line_segment_.setLineMinInliers( line_min_inliers_ );
        plane_from_line_segment_.setLineFittingThreshold( line_fitting_threshold_ );
        plane_from_line_segment_.setNormalUseDepthSmoothing( normal_use_depth_dependent_smoothing_ );
        plane_from_line_segment_.setNormalDepthChangeFactor( normal_max_depth_change_factor_ );
        plane_from_line_segment_.setNormalSmoothingSize( normal_smoothing_size_ );
        plane_from_line_segment_.setNormalMinInliers( normal_min_inliers_ );
        plane_from_line_segment_.setNormalMaximumCurvature( normal_maximum_curvature_ );
        plane_from_line_segment_.setMinInliers( min_inliers_ );
        plane_from_line_segment_.setDistanceThreshold( distance_threshold_ );
        plane_from_line_segment_.setNeighborThreshold( neighbor_threshold_ );
        plane_from_line_segment_.setOptimizeCoefficients( optimize_coefficients_ );
        plane_from_line_segment_.setProjectPoints( project_points_ );
        is_update_line_based_parameters_ = false;
    }

    //
    std::vector<PlaneFromLineSegment::NormalType> line_based_planes;
    plane_from_line_segment_.setInputCloud( cloud_in );
    plane_from_line_segment_.segment( line_based_planes );

    // Refine
    for( int i = 0; i < line_based_planes.size(); i++)
    {
        PlaneFromLineSegment::NormalType &normal = line_based_planes[i];
        PlaneType plane;
        plane.centroid = normal.centroid;
        plane.coefficients = normal.coefficients;
        plane.inlier = normal.inliers;
        //
        planes.push_back( plane );
    }
}

void PlaneSegment::ransacPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    /*
    Method Type:
    SAC_RANSAC  = 0;
    SAC_LMEDS   = 1;
    SAC_MSAC    = 2;
    SAC_RRANSAC = 3; // caution
    SAC_RMSAC   = 4; // caution
    SAC_MLESAC  = 5; // caution
    SAC_PROSAC  = 6; // caution
    */

    // SAC_RANSAC plane model segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointType> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType ( pcl::SACMODEL_PLANE );
    seg.setMethodType ( ransac_method_type_ );  // Method Type
    seg.setMaxIterations( ransac_max_iterations_ );
    seg.setDistanceThreshold ( ransac_distance_threshold_ );

    // Create the filtering object
    pcl::ExtractIndices<PointType> eif;

    PointCloudTypePtr cloud_in (new PointCloudType);
    pcl::copyPointCloud( *input, *cloud_in);
    PointCloudTypePtr cloud_f (new PointCloudType);
    unsigned long npoints = cloud_in->points.size();
    // while 20% of the original cloud is still there
    while(cloud_in->points.size() > ransac_points_left_persentage_*npoints)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_in);
        seg.segment (*inliers, *coefficients);

        if(inliers->indices.size() < ransac_min_points_size_)
            break;

        // store plane
        PlaneType plane;
        plane.inlier = inliers->indices;
        plane.coefficients[0] = coefficients->values[0];
        plane.coefficients[1] = coefficients->values[1];
        plane.coefficients[2] = coefficients->values[2];
        plane.coefficients[3] = coefficients->values[3];
        planes.push_back( plane );

        // Create the filtering object
        // Extract the inliers
        eif.setInputCloud( cloud_in );
        eif.setIndices( inliers );
        eif.setNegative( true );
        eif.filter( *cloud_f );
        cloud_in->swap( *cloud_f );
    }
}

void PlaneSegment::organizedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    // copy input cloud
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::copyPointCloud( *input, *cloud_in);
    // segment planes
    OrganizedPlaneSegmentResult result;
    organized_plane_segment_.segment( cloud_in, result);
    // store planes
    for(int i = 0; i < result.model_coeffs.size(); i++)
    {
        pcl::ModelCoefficients &coef = result.model_coeffs[i];
        pcl::PointIndices &indices = result.inlier_indices[i];
        pcl::PlanarRegion<PointType> &pr = result.regions[i];
        PlaneType plane;
        Eigen::Vector3f centroid = pr.getCentroid();
        plane.centroid.x = centroid[0];
        plane.centroid.y = centroid[1];
        plane.centroid.z = centroid[2];
        plane.inlier = indices.indices;
        plane.coefficients[0] = coef.values[0];
        plane.coefficients[1] = coef.values[1];
        plane.coefficients[2] = coef.values[2];
        plane.coefficients[3] = coef.values[3];
        planes.push_back( plane );
    }
}

void PlaneSegment::regionGrowPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    // copy input cloud
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::copyPointCloud( *input, *cloud_in);
}

bool PlaneSegment::planeFromPoint(PointCloudTypePtr &input, int index, PlaneType &plane)
{
    if (!plane_from_line_segment_.isInitialized())
    {
        plane_from_line_segment_.setCameraParameters( camera_parameters_ );
        cout << "Initialize line base segment." << endl;
    }

    //
    if( is_update_line_based_parameters_ )
    {
        plane_from_line_segment_.setUseHorizontalLines( use_horizontal_line_ );
        plane_from_line_segment_.setUseVerticleLines( use_verticle_line_ );
        plane_from_line_segment_.setYskip( y_skip_ );
        plane_from_line_segment_.setXSkip( x_skip_ );
        plane_from_line_segment_.setLinePointMinDistance( line_point_min_distance_ );
        plane_from_line_segment_.setRhoConstantError( scan_rho_constant_error_ );
        plane_from_line_segment_.setRhoDistanceError( scan_rho_distance_error_ );
        plane_from_line_segment_.setSlideWindowSize( slide_window_size_ );
        plane_from_line_segment_.setLineMinInliers( line_min_inliers_ );
        plane_from_line_segment_.setLineFittingThreshold( line_fitting_threshold_ );
        plane_from_line_segment_.setNormalUseDepthSmoothing( normal_use_depth_dependent_smoothing_ );
        plane_from_line_segment_.setNormalDepthChangeFactor( normal_max_depth_change_factor_ );
        plane_from_line_segment_.setNormalSmoothingSize( normal_smoothing_size_ );
        plane_from_line_segment_.setNormalMinInliers( normal_min_inliers_ );
        plane_from_line_segment_.setNormalMaximumCurvature( normal_maximum_curvature_ );
        plane_from_line_segment_.setMinInliers( min_inliers_ );
        plane_from_line_segment_.setDistanceThreshold( distance_threshold_ );
        plane_from_line_segment_.setNeighborThreshold( neighbor_threshold_ );
        plane_from_line_segment_.setOptimizeCoefficients( optimize_coefficients_ );
        plane_from_line_segment_.setProjectPoints( project_points_ );
        is_update_line_based_parameters_ = false;
    }

    //
    PlaneFromLineSegment::NormalType line_based_plane;
    if( !isValidPoint( input->points[index] ))
    {
        cout << YELLOW << "Invalid point for single plane segment." << RESET << endl;
        return false;
    }
    //
    PointCloudTypePtr cloud_in (new PointCloudType);
    pcl::copyPointCloud( *input, *cloud_in);
    plane_from_line_segment_.setInputCloud( cloud_in );
    if( !plane_from_line_segment_.initCompute() )
    {
        ROS_WARN("Cann't init compute for single plane segment.");
        return false;
    }
    plane_from_line_segment_.segmentSinglePlane( index, line_based_plane);
    plane.centroid = line_based_plane.centroid;
    plane.coefficients = line_based_plane.coefficients;
    plane.inlier = line_based_plane.inliers;
    plane_from_line_segment_.deinitCompute();
    return line_based_plane.valid;
}

void PlaneSegment::planeSegmentReconfigCallback(plane_segment::PlaneSegmentConfig &config, uint32_t level)
{
    plane_segment_method_ = config.segment_method;
    display_input_cloud_ = config.display_input_cloud;
    display_line_cloud_ = config.display_line_cloud;
    display_plane_ = config.display_plane;
    loop_one_message_ = config.loop_one_message;

    cout << GREEN <<"Common Segment Config." << RESET << endl;
}

void PlaneSegment::lineBasedSegmentReconfigCallback(plane_segment::LineBasedSegmentConfig &config, uint32_t level)
{
    use_horizontal_line_ = config.use_horizontal_line;
    use_verticle_line_ = config.use_verticle_line;
    y_skip_ = config.y_skip;
    x_skip_ = config.x_skip;
    line_point_min_distance_ = config.line_point_min_distance;
    scan_rho_constant_error_ = config.scan_rho_constant_error;
    scan_rho_distance_error_ = config.scan_rho_distance_error;
    slide_window_size_ = config.slide_window_size;
    line_min_inliers_ = config.line_min_inliers;
    line_fitting_threshold_ = config.line_fitting_threshold;
    normal_use_depth_dependent_smoothing_ = config.normal_use_depth_dependent_smoothing;
    normal_max_depth_change_factor_ = config.normal_max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    normal_min_inliers_ = config.normal_min_inliers;
    normal_maximum_curvature_ = config.normal_maximum_curvature;
    min_inliers_ = config.min_inliers;
    distance_threshold_ = config.distance_threshold;
    neighbor_threshold_ = config.neighbor_threshold;
    optimize_coefficients_ = config.optimize_coefficients;
    project_points_ = config.project_points;
    //
    is_extract_single_plane_ = config.is_extract_single_plane;
    single_plane_row_ = config.single_plane_row;
    single_plane_col_ = config.single_plane_col;

    cout << GREEN <<"Line Based Segment Config." << RESET << endl;

    is_update_line_based_parameters_ = true;
}

void PlaneSegment::ransacSegmentReconfigCallback(plane_segment::RansacSegmentConfig &config, uint32_t level)
{
    ransac_method_type_ = config.ransac_method_type;
    ransac_max_iterations_ = config.ransac_max_iterations;
    ransac_distance_threshold_ = config.ransac_distance_threshold;
    ransac_min_points_size_ = config.ransac_min_points_size;
    ransac_points_left_persentage_ = config.ransac_points_left_persentage;

    cout << GREEN <<"RANSAC Segment Config." << RESET << endl;
}

void PlaneSegment::organizedSegmentReconfigCallback(plane_segment::OrganizedSegmentConfig &config, uint32_t level)
{
    //
    organized_plane_segment_.ne_method_ = config.organized_ne_method;
    organized_plane_segment_.ne_max_depth_change_factor_ = config.organized_ne_max_depth_change_factor;
    organized_plane_segment_.ne_normal_smoothing_size_ = config.organized_ne_normal_smoothing_size;
    //
    organized_plane_segment_.angular_threshold_ = config.organized_angular_threshold;
    organized_plane_segment_.distance_threshold_ = config.organized_distance_threshold;
    organized_plane_segment_.min_inliers_ = config.organized_min_inliers;
    organized_plane_segment_.project_bounding_points_ = config.organized_project_bounding_points;

    cout << GREEN <<"Organized Segment Config." << RESET << endl;
}


void PlaneSegment::displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport)
{
    if(display_plane_)
    {
        for(int j = 0; j < planes.size(); j++)
        {
            stringstream ss;
            ss << "_" << j;
            pclViewerPlane( input, planes[j], prefix + ss.str(), viewport );
        }
    }
}

void PlaneSegment::displayLinesAndNormals( const PointCloudTypePtr &input,
                                            std::vector<PlaneFromLineSegment::LineType> &lines,
                                            std::vector<PlaneFromLineSegment::NormalType> &normals,
                                            int viewport)
{
    if(display_input_cloud_)
    {
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgba_color(frame_current.point_cloud, 255, 255, 255);
        pcl_viewer_->addPointCloud( input, "rgba_cloud" );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud");
    }

    if(display_line_cloud_)
    {
        for(int j = 0; j < lines.size(); j++)
        {
            stringstream ss;
            ss << "line_" << j;
            pclViewerLineRegion( input, lines[j], ss.str(), viewport );
        }
    }

    if(display_normal_)
    {
        for(int j = 0; j < normals.size(); j++)
        {
            stringstream ss;
            ss << "normal_" << j;
            pclViewerNormal( input, normals[j], ss.str(), viewport );
        }
    }
}

void PlaneSegment::pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint)
{
    PointCloudTypePtr cloud (new PointCloudType );

    for(int i = 0; i < line.inliers.size(); i++)
    {
        cloud->points.push_back( input->points[line.inliers[i]] );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0));
    pcl_viewer_->addPointCloud(cloud, color, id, viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, viewpoint);

}

void PlaneSegment::pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint)
{
    PlaneType plane;
    plane.centroid = normal.centroid;
    plane.coefficients = normal.coefficients;
    plane.inlier = normal.inliers;

    pclViewerPlane( input, plane, id, viewpoint);
}

void PlaneSegment::pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewpoint)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    // add a line
    PointType p1, p2;
    p1 = plane.centroid;
    // check centroid
    if( p1.z == 0)
    {
        Eigen::Vector4f cen;
        pcl::compute3DCentroid( *input, plane.inlier, cen);
        p1.x = cen[0];
        p1.y = cen[1];
        p1.z = cen[2];
    }

    p2.x = p1.x + plane.coefficients[0]*0.2;
    p2.y = p1.y + plane.coefficients[1]*0.2;
    p2.z = p1.z + plane.coefficients[2]*0.2;
    pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewpoint);
    // add a sphere
//    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewpoint);
    // add inlier
    PointCloudTypePtr cloud (new PointCloudType );

    for(int i = 0; i < plane.inlier.size(); i++)
    {
        cloud->points.push_back( input->points[plane.inlier[i]] );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
    pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier", viewpoint);
}

void PlaneSegment::getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                                      PlaneFromLineSegment::CAMERA_PARAMETERS &camera)
{
    /* Intrinsic camera matrix for the raw (distorted) images.
         [fx  0 cx]
     K = [ 0 fy cy]
         [ 0  0  1] */
    camera.cx = cam_info_msg->K[2];
    camera.cy = cam_info_msg->K[5];
    camera.fx = cam_info_msg->K[0];
    camera.fy = cam_info_msg->K[4];
    camera.scale = 1.0;
    // Additionally, organized cloud width and height.
    camera.width = cam_info_msg->width;
    camera.height = cam_info_msg->height;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PlaneSegment::image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                                        const PlaneFromLineSegment::CAMERA_PARAMETERS& camera )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );
    cloud->is_dense = false;
    cloud->width = depth_img.cols;
    cloud->height = depth_img.rows;
    cloud->points.resize(cloud->width * cloud->height);

    const double fx = 1.0 / camera.fx;
    const double fy = 1.0 / camera.fy;
//    const double min_depth = range_min_depth_;
    const double min_depth = 0.1;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt_iter = cloud->begin();
    int depth_idx = 0;
    int color_idx = 0;
    int color_skip_idx = 3;
    for (int v = 0; v < depth_img.rows; v++)
    {
        for (int u = 0; u < depth_img.cols; u++)
        {
            if(pt_iter == cloud->end())
            {
                break;
            }
            pcl::PointXYZRGBA &pt = *pt_iter;
            float Z = depth_img.at<float>(depth_idx);
            // Check for invalid measurements
            if (Z <= min_depth) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - camera.cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - camera.cy) * 1.0 * fy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
            }
            else // Fill in XYZ
            {
                pt.x = (u - camera.cx) * Z * fx;
                pt.y = (v - camera.cy) * Z * fy;
                pt.z = Z;
            }

            RGBValue color;

            color.Blue = rgb_img.at<uint8_t>(color_idx);
            color.Green = rgb_img.at<uint8_t>(color_idx+1);
            color.Red = rgb_img.at<uint8_t>(color_idx+2);
            color.Alpha = 1.0;
            pt.rgb = color.float_value;
            //
            pt_iter ++;
            depth_idx ++;
            color_idx += color_skip_idx;
        }
    }

    return cloud;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segment_node");
    PlaneSegment ps;
    ros::spin();
}
