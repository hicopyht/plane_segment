#include "plane_segment.h"

using namespace std;

PlaneSegment::PlaneSegment() :
    private_nh_("~")
  , plane_segment_config_server_( ros::NodeHandle( "PlaneSegment" ) )
  , line_based_segment_config_server_( ros::NodeHandle( "LineBasedSegment" ) )
  , ransac_segment_config_server_( ros::NodeHandle( "RansacSegment" ) )
  , organized_segment_config_server_( ros::NodeHandle( "OrganizedSegment" ) )
  , region_grow_segment_config_server_( ros::NodeHandle( "RegionGrowSegment" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , prttcp_ (new pcl::DefaultPointRepresentation<PointType>)
  , camera_parameters_()
  , plane_from_line_segment_()
  , organized_plane_segment_()
  , single_plane_row_( 200 )
  , single_plane_col_( 200 )
  , region_grow_cloud_indices_( new pcl::PointIndices )
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
        region_grow_segment_config_callback_ = boost::bind(&PlaneSegment::regionGrowSegmentReconfigCallback, this, _1, _2);
        region_grow_segment_config_server_.setCallback(region_grow_segment_config_callback_);
    }

    viewer_ = new PlaneViewer( nh_ );

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/rgb/camera_info");
    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");


    camera_parameters_.cx = 319.5;
    camera_parameters_.cy = 239.5;
    camera_parameters_.fx = 525.0;
    camera_parameters_.fy = 525.0;
    camera_parameters_.scale = 1.0;
    camera_parameters_.width = 640;
    camera_parameters_.height = 480;


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

    scan_point_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud2", 10);
}


void PlaneSegment::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::ImageConstPtr& depth_img_msg,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("no cloud msg: %d\n", visual_img_msg->header.seq);
    getCameraParameter( cam_info_msg, camera_parameters_);
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::PointIndicesPtr indices ( new pcl::PointIndices );
    // Get Mat Image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    // Get PointCloud
    cloud_in = image2PointCloud( visual_image, depth_image, camera_parameters_, indices);

    *region_grow_cloud_indices_ = *indices;

    if( loop_message_ && visual_img_msg->header.seq >= loop_message_ )
    {
        while( loop_message_ && ros::ok())
        {
            ros::Duration(1.0).sleep();
            processCloud( cloud_in );
        }
    }

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
    pcl::PointIndicesPtr indices ( new pcl::PointIndices );

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
    PointCloudTypePtr colored_cloud ( new PointCloudType );
    std::vector<PlaneType> region_grow_planes;

    PointCloudTypePtr cloud_in( new PointCloudType );
    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( input, cloud_in, camera_parameters_, QVGA);
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( input, cloud_in, camera_parameters_, QQVGA);
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        cloud_in = input; // copy pointer
    }

    ros::Time start_time = ros::Time::now();
    ros::Time time = ros::Time::now();
//    double start_time = pcl::getTime();
//    pcl::console::TicToc time;
//    time.tic();
    float line_base_dura = 0, ransac_dura = 0;
    float organized_dura = 0, region_grow_dura = 0;
    float single_plane_dura = 0, display_dura = 0;
    float total_dura = 0;
    vector<float> runtimes;

    // do segmentation
    if(plane_segment_method_ == LINE_BADED)
    {
//        lineBasedPlaneSegment( input, line_based_planes );
        lineBasedPlaneSegment( cloud_in, lines, normals, line_based_planes, runtimes );
//        line_base_dura = time.toc();
//        time.tic();
        line_base_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if(plane_segment_method_ == RANSAC)
    {
        ransacPlaneSegment( cloud_in, ransac_planes );
//        ransac_dura = time.toc();
//        time.tic();
        ransac_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if(plane_segment_method_ == ORGANSIZED)
    {
        organizedPlaneSegment( cloud_in, organized_planes );
//        organized_dura = time.toc();
//        time.tic();
        organized_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if(plane_segment_method_ == REGION_GROW)
    {
        regionGrowPlaneSegment( cloud_in, region_grow_planes, colored_cloud );
//        region_grow_dura = time.toc();
//        time.tic();
        region_grow_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if(plane_segment_method_ == ALL_EXCEPT_RG)
    {
        //
//        lineBasedPlaneSegment( input, line_based_planes );
        lineBasedPlaneSegment( cloud_in, lines, normals, line_based_planes, runtimes );
//        line_base_dura = time.toc();
//        time.tic();
        line_base_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();

        ransacPlaneSegment( cloud_in, ransac_planes );
//        ransac_dura = time.toc();
//        time.tic();
        ransac_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();

        organizedPlaneSegment( cloud_in, organized_planes );
//        organized_dura = time.toc();
//        time.tic();
        organized_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if( plane_segment_method_ == LB_OMPS)
    {
        //
//        lineBasedPlaneSegment( input, line_based_planes );
        lineBasedPlaneSegment( cloud_in, lines, normals, line_based_planes, runtimes );
//        line_base_dura = time.toc();
//        time.tic();
        line_base_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();

        organizedPlaneSegment( cloud_in, organized_planes );
//        organized_dura = time.toc();
//        time.tic();
        organized_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if(plane_segment_method_ == ALL_METHOD)
    {
        //
//        lineBasedPlaneSegment( input, line_based_planes );
        lineBasedPlaneSegment( cloud_in, lines, normals, line_based_planes, runtimes );
//        line_base_dura = time.toc();
//        time.tic();
        line_base_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();

        ransacPlaneSegment( cloud_in, ransac_planes );
//        ransac_dura = time.toc();
//        time.tic();
        ransac_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();

        organizedPlaneSegment( cloud_in, organized_planes );
//        organized_dura = time.toc();
//        time.tic();
        organized_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();

        regionGrowPlaneSegment( cloud_in, region_grow_planes, colored_cloud );
//        region_grow_dura = time.toc();
//        time.tic();
        region_grow_dura = (ros::Time::now() - time).toSec()*1000;
        time = ros::Time::now();
    }
    else if(plane_segment_method_ == NONE_METHOD)   // just view pointcloud
    {
//        pcl_viewer_->removeAllPointClouds();
//        pcl_viewer_->removeAllShapes();
//        if(display_input_cloud_)
//        {
//            pcl_viewer_->addPointCloud( cloud_in, "rgba_cloud", viewer_v1_);
////            pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud", viewer_v1_);
//        }
//        pcl_viewer_->spinOnce(1);
        return;
    }
    //
    std::vector<PlaneType> single_plane;
    if(is_extract_single_plane_)
    {
        PlaneType sp;
        int idx = single_plane_row_ * camera_parameters_.width + single_plane_col_;
        cout << " - single idx: " << idx << endl;
        if( planeFromPoint( cloud_in, idx, sp ) )
        {
            single_plane.push_back( sp );
        }
    }
//    single_plane_dura = time.toc();
//    time.tic();
    single_plane_dura = (ros::Time::now() - time).toSec()*1000;
    time = ros::Time::now();
    //

    // display
    viewer_->clearViewer();
    viewer_->clearViewerSingle();
    viewer_->displayInputCloud( cloud_in );
    viewer_->displayLinesAndNormals( cloud_in, lines, normals, viewer_->vp1());
    viewer_->displayLinesAndNormalsSingle( cloud_in, lines, normals );
    vector<pcl::PointIndices> vindices;
    vector< vector <ScanPoint> > scans;
    plane_from_line_segment_.getScanlineCloud( cloud_in, vindices, scans );
    viewer_->displayScanlines( cloud_in, vindices, scans, viewer_->vp3() );
    if( scans.size() > 0)
        pubScanPoint( scans[scans.size()/2] );
//    displayPlanes( input, single_plane, "single_plane", viewer_->vp1() );
    viewer_->displayPlanes( cloud_in, line_based_planes, "line_based_plane", viewer_->vp2() );
    viewer_->displayPlanes( cloud_in, ransac_planes, "ransac_plane", viewer_->vp3() );
    viewer_->displayPlanes( cloud_in, organized_planes, "organized_plane", viewer_->vp4() );
    viewer_->displayPlanes( cloud_in, region_grow_planes, "region_growing_plane", viewer_->vp1() );
    //
    viewer_->display2DText("Lines And Normals", 240, 3, "v1_text", viewer_->vp1() );
    viewer_->display2DText("Proposed Planes", 240, 3, "v2_text", viewer_->vp2() );
    viewer_->display2DText("Scene", 240, 3, "v3_text", viewer_->vp3() );
    viewer_->display2DText("OMPS Planes", 240, 3, "v4_text", viewer_->vp4() );
    viewer_->spinViewer(10);
    viewer_->spinViewerSingle(10);

//    display_dura = time.toc();
//    total_dura = pcl::getTime() - start_time;
    display_dura = (ros::Time::now() - time).toSec() * 1000;
    time = ros::Time::now();
    total_dura = (ros::Time::now() - start_time).toSec()*1000;

//    std::cout.precision(2);
    cout << GREEN << " Time: LineB: " << line_base_dura << ", RANSAC: " << ransac_dura
         << ", Organized: " << organized_dura << ", RGrow: " << region_grow_dura
         << ", Single: " << single_plane_dura << RESET << endl;
    cout << YELLOW << " Planes: LB: " << line_based_planes.size()
            << ", RANSAC: " << ransac_planes.size() << ", Organized: " << organized_planes.size()
               << ", RGrow: " << region_grow_planes.size() << RESET << endl;
    ROS_INFO("Total time: %f, display %f \n", total_dura, display_dura);

    evaluateRuntimes(runtimes[0], runtimes[1], runtimes[2], runtimes[3], ransac_dura, region_grow_dura, organized_dura, line_base_dura);

    if( print_plane_coefficients_ )
        printPlaneCoefficients( line_based_planes, "Line Based Planes:");

    cout << "----------------------------------- END -------------------------------------" << endl;
}

void PlaneSegment::setlineBasedPlaneSegmentParameters()
{
    //
    plane_from_line_segment_.setUseHorizontalLines( use_horizontal_line_ );
    plane_from_line_segment_.setUseVerticleLines( use_verticle_line_ );
    plane_from_line_segment_.setYskip( y_skip_ );
    plane_from_line_segment_.setXSkip( x_skip_ );
    plane_from_line_segment_.setLinePointMinDistance( line_point_min_distance_ );
    plane_from_line_segment_.setUseDepthNoiseModel( use_depth_noise_model_ );
    plane_from_line_segment_.setRhoConstantError( scan_rho_constant_error_ );
    plane_from_line_segment_.setRhoDistanceError( scan_rho_distance_error_ );
    plane_from_line_segment_.setRhoQuadraticError( scan_rho_quadratic_error_ );
    plane_from_line_segment_.setSlideWindowSize( slide_window_size_ );
    plane_from_line_segment_.setLineMinInliers( line_min_inliers_ );
    plane_from_line_segment_.setLineFittingThreshold( line_fitting_threshold_ );
    //
    plane_from_line_segment_.setNormalsPerLine( normals_per_line_ );
    plane_from_line_segment_.setNormalUseDepthSmoothing( normal_use_depth_dependent_smoothing_ );
    plane_from_line_segment_.setNormalDepthChangeFactor( normal_max_depth_change_factor_ );
    plane_from_line_segment_.setNormalSmoothingSize( normal_smoothing_size_ );
    plane_from_line_segment_.setNormalMinInliersPercentage( normal_min_inliers_percentage_ );
    plane_from_line_segment_.setNormalMaximumCurvature( normal_maximum_curvature_ );
    //
    plane_from_line_segment_.setRemoveDuplicateCandidate( remove_duplicate_candidate_ );
    plane_from_line_segment_.setDuplicateCandidateThreshold( duplicate_candidate_normal_thresh_,
                                                             duplicate_candidate_distance_thresh_ );
    //
    plane_from_line_segment_.setPlaneSegmentCriterion( plane_segment_criterion_ );
    plane_from_line_segment_.setCriterionBothParameters( k_curvature_, k_inlier_ );
    plane_from_line_segment_.setMinInliers( min_inliers_ );
    plane_from_line_segment_.setMaxCurvature( max_curvature_ );
    plane_from_line_segment_.setDistanceThreshold( distance_threshold_ );
    plane_from_line_segment_.setNeighborThreshold( neighbor_threshold_ );
    plane_from_line_segment_.setOptimizeCoefficients( optimize_coefficients_ );
    plane_from_line_segment_.setProjectPoints( project_points_ );
    plane_from_line_segment_.setExtractBoundary( extract_boundary_ );
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
        setlineBasedPlaneSegmentParameters();
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
        plane.boundary_inlier = normal.boundary_inlier;
        plane.hull_inlier = normal.hull_inlier;
        projectPoints( input, plane.inlier, plane.coefficients, *(plane.cloud) );
        projectPoints( input, plane.boundary_inlier, plane.coefficients, *(plane.cloud_boundary) );
        projectPoints( input, plane.hull_inlier, plane.coefficients, *(plane.cloud_hull) );
        //
        planes.push_back( plane );
    }
}


void PlaneSegment::lineBasedPlaneSegment(PointCloudTypePtr &input,
                                         std::vector<PlaneFromLineSegment::LineType>& lines,
                                         std::vector<PlaneFromLineSegment::NormalType>& normals,
                                         std::vector<PlaneType> &planes,
                                         vector<float>& runtimes)
{
    static int last_size_type = cloud_size_type_;

    PointCloudTypePtr cloud_in (new PointCloudType);
    pcl::copyPointCloud( *input, *cloud_in);

    //
    if( is_update_line_based_parameters_ )
    {
        setlineBasedPlaneSegmentParameters();
        is_update_line_based_parameters_ = false;
    }

    if (!plane_from_line_segment_.isInitialized() || cloud_size_type_ != last_size_type)
    {
        last_size_type = cloud_size_type_;
        plane_from_line_segment_.setCameraParameters( camera_parameters_ );
        cout << "Initialize line base segment." << endl;
    }

    //
    std::vector<PlaneFromLineSegment::NormalType> line_based_planes;
    plane_from_line_segment_.setInputCloud( cloud_in );
    plane_from_line_segment_.segment( lines, normals, line_based_planes, runtimes);
    cout << YELLOW << "L: " << lines.size() << ", N: " << normals.size() << ", P: " << line_based_planes.size() << RESET << endl;

    // Refine
    for( int i = 0; i < line_based_planes.size(); i++)
    {
        PlaneFromLineSegment::NormalType &normal = line_based_planes[i];
        PlaneType plane;
        plane.centroid = normal.centroid;
        plane.coefficients = normal.coefficients;
        plane.inlier = normal.inliers;
        plane.boundary_inlier = normal.boundary_inlier;
        plane.hull_inlier = normal.hull_inlier;
        projectPoints( input, plane.inlier, plane.coefficients, *(plane.cloud) );
        projectPoints( input, plane.boundary_inlier, plane.coefficients, *(plane.cloud_boundary) );
        projectPoints( input, plane.hull_inlier, plane.coefficients, *(plane.cloud_hull) );
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

    // cloud
    PointCloudTypePtr cloud_in (new PointCloudType);
    if( ransac_use_downsample_ )
        voxelGridDownsample( input, cloud_in, voxel_grid_downsample_size_);  // downsample cloud
    else
        pcl::copyPointCloud( *input, *cloud_in);    // copy input cloud
    //
    // Create the filtering object
    pcl::ExtractIndices<PointType> eif;
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

void PlaneSegment::regionGrowPlaneSegment(PointCloudTypePtr &input,
                                          std::vector<PlaneType> &planes,
                                          PointCloudTypePtr &colored_cloud)
{
    // copy input cloud
    PointCloudTypePtr cloud_in ( new PointCloudType );
    if( rg_use_downsample_ )
        voxelGridDownsample( input, cloud_in, voxel_grid_downsample_size_ );
    else
        pcl::copyPointCloud( *input, *cloud_in);
    // build indices
    pcl::PointIndicesPtr indices( new pcl::PointIndices );
    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        if( isValidPoint(cloud_in->points[i]) )
            indices->indices.push_back( i );
    }

    pcl::search::Search<PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod ( tree );
    normal_estimator.setInputCloud ( cloud_in );
    normal_estimator.setKSearch ( rg_normal_k_search_ );
    normal_estimator.compute ( *normals );

//    cout << "extract normal..." << endl;

//    pcl::IndicesPtr indices (new std::vector <int>);
//    pcl::PassThrough<PointType> pass;
//    pass.setInputCloud ( cloud_in );
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 10.0);
//    pass.filter ( *indices );

    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    reg.setMinClusterSize ( rg_min_cluster_size_ );
    reg.setMaxClusterSize ( rg_max_cluster_size_ );
    reg.setSearchMethod ( tree );
    reg.setNumberOfNeighbours ( rg_number_of_neighbours_ );
    reg.setInputCloud ( cloud_in );
    reg.setIndices ( indices );
    reg.setInputNormals ( normals );
    reg.setSmoothnessThreshold ( rg_smoothness_thresh_ / 180.0 * M_PI );
    reg.setCurvatureThreshold ( rg_curvature_thresh_ );

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "\ncluster:" << clusters.size() << std::endl;

    // compute plane parameters
    for(int i = 0; i < clusters.size(); i++)
    {
        PlaneType plane;
        pcl::PointIndices &indices = clusters[i];
        plane.inlier = indices.indices;
        computePlane( input, plane.inlier, plane);
        planes.push_back( plane );
    }

    colored_cloud = reg.getColoredCloudRGBA();
}

void PlaneSegment::computePlane(PointCloudTypePtr &input, std::vector<int> &inliers, PlaneType &plane)
{
    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    float curvature;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    pcl::computeMeanAndCovarianceMatrix (*input, inliers, covariance_matrix, xyz_centroid);
    //
    plane.centroid.x = xyz_centroid[0];
    plane.centroid.y = xyz_centroid[1];
    plane.centroid.z = xyz_centroid[2];

    // Get the plane normal and surface curvature
    pcl::solvePlaneParameters (covariance_matrix, xyz_centroid, plane.coefficients, curvature);
    flipNormalTowardsViewpoint(plane.centroid, 0, 0, 0, plane.coefficients[0], plane.coefficients[1], plane.coefficients[2]);
    plane.coefficients[3] = 0;
    plane.coefficients[3] = -1 * plane.coefficients.dot (plane.centroid.getVector4fMap());
}

void PlaneSegment::flipPlaneTowardsViewpoint (const PointType &point,
                            float vp_x, float vp_y, float vp_z,
                            float &nx, float &ny, float &nz)
{
    // See if we need to flip any plane normals
    vp_x -= point.x;
    vp_y -= point.y;
    vp_z -= point.z;

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

    // Flip the plane normal
    if (cos_theta < 0)
    {
        nx *= -1;
        ny *= -1;
        nz *= -1;
    }
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
        setlineBasedPlaneSegmentParameters();
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
    cloud_size_type_config_ = config.cloud_size_type;
    plane_segment_method_ = config.segment_method;
    voxel_grid_downsample_size_ = config.voxel_grid_downsample_size;
    print_plane_coefficients_ = config.print_plane_coefficients;
    evaluate_run_time_ = config.evaluate_run_time;
    loop_one_message_ = config.loop_one_message;
    loop_message_ = config.loop_message;

    cout << GREEN <<"Common Segment Config." << RESET << endl;
}

void PlaneSegment::lineBasedSegmentReconfigCallback(plane_segment::LineBasedSegmentConfig &config, uint32_t level)
{
    //
    use_horizontal_line_ = config.use_horizontal_line;
    use_verticle_line_ = config.use_verticle_line;
    y_skip_ = config.y_skip;
    x_skip_ = config.x_skip;
    line_point_min_distance_ = config.line_point_min_distance;
    use_depth_noise_model_ = config.use_depth_noise_model;
    scan_rho_constant_error_ = config.scan_rho_constant_error;
    scan_rho_distance_error_ = config.scan_rho_distance_error;
    scan_rho_quadratic_error_ = config.scan_rho_quadratic_error;
    slide_window_size_ = config.slide_window_size;
    line_min_inliers_ = config.line_min_inliers;
    line_fitting_threshold_ = config.line_fitting_threshold;
    //
    normals_per_line_ = config.normals_per_line;
    normal_use_depth_dependent_smoothing_ = config.normal_use_depth_dependent_smoothing;
    normal_max_depth_change_factor_ = config.normal_max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    normal_min_inliers_percentage_ = config.normal_min_inliers_percentage;
    normal_maximum_curvature_ = config.normal_maximum_curvature;
    //
    remove_duplicate_candidate_ = config.remove_duplicate_candidate;
    duplicate_candidate_normal_thresh_ = config.duplicate_candidate_normal_thresh;
    duplicate_candidate_distance_thresh_ = config.duplicate_candidate_distance_thresh;
    //
    plane_segment_criterion_ = config.plane_segment_criterion;
    k_curvature_ = config.k_curvature;
    k_inlier_ = config.k_inlier;
    min_inliers_ = config.min_inliers;
    max_curvature_ = config.max_curvature;
    distance_threshold_ = config.distance_threshold;
    neighbor_threshold_ = config.neighbor_threshold;
    optimize_coefficients_ = config.optimize_coefficients;
    project_points_ = config.project_points;
    extract_boundary_ = config.extract_boundary;
    //
    is_extract_single_plane_ = config.is_extract_single_plane;
    single_plane_row_ = config.single_plane_row;
    single_plane_col_ = config.single_plane_col;

    cout << GREEN <<"Line Based Segment Config." << RESET << endl;

    is_update_line_based_parameters_ = true;
}

void PlaneSegment::ransacSegmentReconfigCallback(plane_segment::RansacSegmentConfig &config, uint32_t level)
{
    ransac_use_downsample_ = config.ransac_use_downsample;
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

void PlaneSegment::regionGrowSegmentReconfigCallback(plane_segment::RegionGrowSegmentConfig &config, uint32_t level)
{
    rg_use_downsample_ = config.rg_use_downsample;
    rg_normal_k_search_ = config.rg_normal_k_search;
    rg_min_cluster_size_ = config.rg_min_cluster_size;
    rg_max_cluster_size_ = config.rg_max_cluster_size;
    rg_number_of_neighbours_ = config.rg_number_of_neighbours;
    rg_smoothness_thresh_ = config.rg_smoothness_thresh;
    rg_curvature_thresh_ = config.rg_curvature_thresh;

    cout << GREEN <<"RegionGrow Segment Config." << RESET << endl;
}



void PlaneSegment::printPlaneCoefficients(const std::vector<PlaneType> &planes, const std::string &prefix)
{
    cout << MAGENTA << prefix << RESET << endl;
    for(int i = 0; i < planes.size(); i++)
    {
        const PlaneType &plane = planes[i];
        cout << CYAN << " - " << plane.coefficients[0] << ", " << plane.coefficients[1] << ", "
             << plane.coefficients[2] << ", " << plane.coefficients[3] << RESET << endl;
    }
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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PlaneSegment::image2PointCloud( const sensor_msgs::ImageConstPtr& visual_img_msg,
                                                                        const sensor_msgs::ImageConstPtr& depth_img_msg,
                                                                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg,
                                                                        PlaneFromLineSegment::CAMERA_PARAMETERS &camera_parameters,
                                                                        pcl::PointIndicesPtr &indices)
{
    getCameraParameter( cam_info_msg, camera_parameters);

    // Get Mat Image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    // Get PointCloud
    return image2PointCloud( visual_image, depth_image, camera_parameters, indices);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PlaneSegment::image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                                        const PlaneFromLineSegment::CAMERA_PARAMETERS& camera,
                                                                        pcl::PointIndicesPtr &indices)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );
    cloud->is_dense = false;
    cloud->width = depth_img.cols;
    cloud->height = depth_img.rows;
    cloud->points.resize(cloud->width * cloud->height);
    //
    indices->indices.clear();

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
//                pt.z = std::numeric_limits<float>::quiet_NaN();
                pt.z = 0;
            }
            else // Fill in XYZ
            {
                pt.x = (u - camera.cx) * Z * fx;
                pt.y = (v - camera.cy) * Z * fy;
                pt.z = Z;
            }

            if( isValidPoint( pt ) )
            {
                indices->indices.push_back( depth_idx );
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


void PlaneSegment::downsampleOrganizedCloud(const PointCloudTypePtr &input, PointCloudTypePtr &output,
                                            PlaneFromLineSegment::CAMERA_PARAMETERS &out_camera, int size_type)
{
    int skip = pow(2, size_type);
    int width = input->width / skip;
    int height = input->height / skip;
    output->width = width;
    output->height = height;
    output->is_dense = false;
    output->points.resize( width * height);
    for( size_t i = 0, y = 0; i < height; i++, y+=skip)
    {
        for( size_t j = 0, x = 0; j < width; j++, x+=skip)
        {
            output->points[i*width + j] = input->points[input->width*y + x];
        }
    }

    out_camera.width = width;
    out_camera.height = height;
    out_camera.cx = width / 2 - 0.5;
    out_camera.cy = height / 2 - 0.5;
    out_camera.fx = 525.0 / skip;
    out_camera.fy = 525.0 / skip;
    out_camera.scale = 1.0;
}

void PlaneSegment::pubScanPoint( vector<ScanPoint> &scan_points)
{
    PointCloudXYZPtr cloud ( new PointCloudXYZ );
    //
    for( int i = 0; i < scan_points.size(); i++)
    {
        ScanPoint& sp = scan_points[i];
        pcl::PointXYZ p;
        p.x = sp.x;
        p.y = sp.y;
        p.z = 0.2;
        cloud->points.push_back( p );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2);
    cloud2.header.frame_id = "/base_link";
    cloud2.header.stamp = ros::Time::now();
    scan_point_publisher_.publish( cloud2 );
}

void PlaneSegment::evaluateRuntimes(float &line, float &normal, float &plane, float &total,
                                    float &ransac, float &rgs, float &omps, float &proposed)
{
    static vector<float> line_durations;
    static vector<float> normal_durations;
    static vector<float> plane_durations;
    static vector<float> total_durations;
    static vector<float> ransac_durations;
    static vector<float> rgs_durations;
    static vector<float> omps_durations;
    static vector<float> proposed_durations;
    static int count = 0;
    const int num = 20;

    if(evaluate_run_time_)
    {
        if(count < num)
        {
            line_durations.push_back( line );
            normal_durations.push_back( normal );
            plane_durations.push_back( plane );
            total_durations.push_back( total );
            ransac_durations.push_back( ransac );
            rgs_durations.push_back( rgs );
            omps_durations.push_back( omps );
            proposed_durations.push_back( proposed );
            count ++;
            if( count == num )
            {
                std::sort(line_durations.begin(), line_durations.end());
                std::sort(normal_durations.begin(), normal_durations.end());
                std::sort(plane_durations.begin(), plane_durations.end());
                std::sort(total_durations.begin(), total_durations.end());
                std::sort(ransac_durations.begin(), ransac_durations.end());
                std::sort(rgs_durations.begin(), rgs_durations.end());
                std::sort(omps_durations.begin(), omps_durations.end());
                std::sort(proposed_durations.begin(), proposed_durations.end());
            }
            cout << CYAN << " Runtime data: " << count << " of " << num << RESET << endl;
        }
        else
        {
//            std::cout.precision(2);
            cout << YELLOW << "Run time: " << endl;
            cout << " Approach  \t min   \t max \t middle \tstddev " << GREEN << endl;
            cout << " - Line Ex:  " << line_durations[0] << "\t " << line_durations[num-1] << "\t "
                    << (line_durations[0] + line_durations[num-1])/2.0 << "\t "
                    << (line_durations[num-1] - line_durations[0])/2.0 << endl;
            cout << " - Normal Ex:" << normal_durations[0] << "\t " << normal_durations[num-1] << "\t "
                    << (normal_durations[0] + normal_durations[num-1])/2.0 << "\t "
                    << (normal_durations[num-1] - normal_durations[0])/2.0 << endl;
            cout << " - Plane Ex: " << plane_durations[0] << "\t " << plane_durations[num-1] << "\t "
                    << (plane_durations[0] + plane_durations[num-1])/2.0 << "\t "
                    << (plane_durations[num-1] - plane_durations[0])/2.0 << endl;
            cout << " - Total Ex: " << total_durations[0] << "\t " << total_durations[num-1] << "\t "
                    << (total_durations[0] + total_durations[num-1])/2.0 << "\t "
                    << (total_durations[num-1] - total_durations[0])/2.0 << endl;
            cout << " - RANSAC:   " << ransac_durations[0] << "\t " << ransac_durations[num-1] << "\t "
                    << (ransac_durations[0] + ransac_durations[num-1])/2.0 << "\t "
                    << (ransac_durations[num-1] - ransac_durations[0])/2.0 << endl;
            cout << " - RGS:      " << rgs_durations[0] << "\t " << rgs_durations[num-1] << "\t "
                    << (rgs_durations[0] + rgs_durations[num-1])/2.0 << "\t "
                    << (rgs_durations[num-1] - rgs_durations[0])/2.0 << endl;
            cout << " - OMPS:     " << omps_durations[0] << "\t " << omps_durations[num-1] << "\t "
                    << (omps_durations[0] + omps_durations[num-1])/2.0 << "\t "
                    << (omps_durations[num-1] - omps_durations[0])/2.0 << endl;
            cout << " - Proposed: " << proposed_durations[0] << "\t " << proposed_durations[num-1] << "\t "
                    << (proposed_durations[0] + proposed_durations[num-1])/2.0 << "\t "
                    << (proposed_durations[num-1] - proposed_durations[0])/2.0 << endl;

            cout << RESET << endl;

            cout << " Node auto shutdown." << endl;
            ros::shutdown();
        }
    }
    else
    {
        count = 0;
        line_durations.clear();
        normal_durations.clear();
        plane_durations.clear();
        total_durations.clear();
        ransac_durations.clear();
        rgs_durations.clear();
        omps_durations.clear();
        proposed_durations.clear();
    }
}

