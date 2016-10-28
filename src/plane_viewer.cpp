#include "plane_viewer.h"

PlaneViewer::PlaneViewer(  ros::NodeHandle &nh  )
    : private_nh_(nh)
    , viewer_config_server_( ros::NodeHandle( "Viewer" ) )
    , single_viewer_( new pcl::visualization::PCLVisualizer("single Viewer") )
    , pcl_viewer_( new pcl::visualization::PCLVisualizer("3D Viewer"))
    , viewer_v1_(1)
    , viewer_v2_(2)
    , viewer_v3_(3)
    , viewer_v4_(4)
    , rng(12345)
{
    viewer_config_callback_ = boost::bind(&PlaneViewer::viewerReconfigCallback, this, _1, _2);
    viewer_config_server_.setCallback( viewer_config_callback_ );

    pcl_viewer_->createViewPort(0, 0, 0.5, 0.5, viewer_v1_);
    pcl_viewer_->addText("Lines And Normals", 100, 3, "v1_text", viewer_v1_);
    pcl_viewer_->createViewPort(0.5, 0, 1.0, 0.5, viewer_v2_);
    pcl_viewer_->addText("Line Based Planes", 100, 3, "v2_text", viewer_v2_);
    pcl_viewer_->createViewPort(0, 0.5, 0.5, 1.0, viewer_v3_);
    pcl_viewer_->addText("Ransac Planes", 100, 3, "v3_text", viewer_v3_);
    pcl_viewer_->createViewPort(0.5, 0.5, 1.0, 1.0, viewer_v4_);
    pcl_viewer_->addText("Organized Planes", 100, 3, "v4_text", viewer_v4_);
    pcl_viewer_->addCoordinateSystem(0.000001);
    pcl_viewer_->initCameraParameters();
    pcl_viewer_->setCameraPosition(0.0, 0.0, -0.4, 0, 0, 0.6, 0, -1, 0);
    pcl_viewer_->setShowFPS(true);

    single_viewer_->addCoordinateSystem(0.000001);
    single_viewer_->initCameraParameters();
    single_viewer_->setCameraPosition(0.0, 0.0, -0.4, 0, 0, 0.6, 0, -1, 0);
    single_viewer_->setShowFPS(true);

}

void PlaneViewer::displayInputCloud(const PointCloudTypePtr &input)
{
    if( display_input_cloud_ )
    {
        pclViewerCloud( input, "rgbd_cloud", 0 );
        pclViewerCloudSingle( input, "rgbd_cloud" );
    }
}

void PlaneViewer::displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport)
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

void PlaneViewer::displayLinesAndNormals( const PointCloudTypePtr &input,
                                          std::vector<PlaneFromLineSegment::LineType> &lines,
                                          std::vector<PlaneFromLineSegment::NormalType> &normals,
                                          int viewport)
{
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


void PlaneViewer::displayScanlines( const PointCloudTypePtr &input,
                                    vector<pcl::PointIndices> &vindices,
                                    vector< vector <ScanPoint> > &scans,
                                    int viewport)
{
    if(display_scanline_cloud_)
    {
        //
//        vector<pcl::PointIndices> vindices;
//        vector< vector <ScanPoint> > scans;
        // get scanline pointcloud and projected 2d scan
//        plane_from_line_segment_.getScanlineCloud( input, vindices, scans);

        // display scanline pointcloud
        for( int i = 0; i < vindices.size(); i++)
        {
            // add inlier
            pcl::PointIndices &indices = vindices[i];
            PointCloudTypePtr cloud = getPointCloudFromIndices(input, indices);
            pcl::visualization::PointCloudColorHandlerCustom<PointType> color(cloud, rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0));
            stringstream ss;
            ss << "scanline_" << i;
            pcl_viewer_->addPointCloud(cloud, color, ss.str(), viewport);
            pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str(), viewport);
        }

        // show 2d scan as sensor_msgs/LaserScan
        if( scans.size() > 0)
        {
            int mid = 0;
//            cout << RED << "  display scanline: " << mid << RESET << endl;
//            pubScanPoint( scans[mid] );
        }
    }
}

void PlaneViewer::display2DText( const std::string &text, int xpos, int ypos,
                                 const std::string &id, int viewport )
{
    pcl_viewer_->addText( text, xpos, ypos, id, viewport);
}

void PlaneViewer::pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint)
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

void PlaneViewer::pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    if( display_normal_arrow_ )
    {
        // add a line
        PointType p1, p2;
        p1 = normal.centroid;
        // check centroid
        if( p1.z == 0)
        {
            Eigen::Vector4f cen;
            pcl::compute3DCentroid( *input, normal.inliers, cen);
            p1.x = cen[0];
            p1.y = cen[1];
            p1.z = cen[2];
        }

        p2.x = p1.x + normal.coefficients[0]*0.20;
        p2.y = p1.y + normal.coefficients[1]*0.20;
        p2.z = p1.z + normal.coefficients[2]*0.20;

        PointType p3;
        p3.x = p1.x + normal.coefficients[0]*0.08;
        p3.y = p1.y + normal.coefficients[1]*0.08;
        p3.z = p1.z + normal.coefficients[2]*0.08;

//        // add a line
//        pcl_viewer_->addLine<PointType>( p1, p2, 0.0, 0.0, 0.0, id+"_line", viewpoint);
////        pcl_viewer_->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, r/255.0, g/255.0, b/255.0, id+"_line", viewpoint );

//        // add a cone
//        pcl::ModelCoefficients coeffs;
//        coeffs.values.clear ();
//        coeffs.values.push_back ( p3.x );
//        coeffs.values.push_back ( p3.y );
//        coeffs.values.push_back ( p3.z );
//        coeffs.values.push_back ( -normal.coefficients[0]*0.04 );
//        coeffs.values.push_back ( -normal.coefficients[1]*0.04 );
//        coeffs.values.push_back ( -normal.coefficients[2]*0.04 );
//        coeffs.values.push_back ( 1.0 );
//        pcl_viewer_->addCone (coeffs, id+"_cone", viewpoint );
//        pcl_viewer_->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, r/255.0, g/255.0, b/255.0, id+"_cone", viewpoint );

//        pcl_viewer_->addArrow(p2, p1, 0.0, 0.0, 0.0, false, id+"_arrow", viewpoint);
        pcl_viewer_->addArrow(p2, p1, r/255.0, g/255.0, b/255.0, false, id+"_arrow", viewpoint);
//        pcl_viewer_->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 2, id+"_arrow", viewpoint );
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier", viewpoint);

//        // normal cloud
//        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//        pcl::Normal nm( normal.coefficients[0], normal.coefficients[1], normal.coefficients[2]);
//        cloud_normals->push_back( nm );
//        cloud_normals->height = 1;
//        cloud_normals->width = 1;
//        cloud_normals->is_dense = false;

//        // centroid cloud
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr centroid_cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
//        centroid_cloud->push_back( normal.centroid );
//        centroid_cloud->height = 1;
//        centroid_cloud->width = 1;
//        centroid_cloud->is_dense = false;

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb(centroid_cloud, r, g, b);
//        pcl_viewer_->addPointCloud(centroid_cloud, rgb, id+"centroide cloud", viewpoint);
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"centroide cloud", viewpoint);
//        pcl_viewer_->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (centroid_cloud, cloud_normals, 1, 0.1, id+"normal_arrow", viewpoint);
////        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r/255.0, g/255.0, b/255.0, id+"normal_arrow", viewpoint);
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, id+"normal_arrow", viewpoint);
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, id+"normal_arrow", viewpoint);

        // add a sphere
//        pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewpoint);
    }
    // add inlier
    PointCloudTypePtr cloud (new PointCloudType );

    for(int i = 0; i < normal.inliers.size(); i++)
    {
        cloud->points.push_back( input->points[normal.inliers[i]] );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
    pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier", viewpoint);

}

void PlaneViewer::pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    // inlier
    if( display_plane_inlier_ && display_plane_projected_inlier_ )
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color( plane.cloud, r, g, b);
        pcl_viewer_->addPointCloud( plane.cloud, color, id+"_inlier", viewport);
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier", viewport);

        if( display_plane_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0)
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *plane.cloud, cen );
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.2;
            p2.y = p1.y + plane.coefficients[1]*0.2;
            p2.z = p1.z + plane.coefficients[2]*0.2;
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
        }
    }
    else if( display_plane_inlier_ )
    {
        PointCloudTypePtr cloud = getPointCloudFromIndices( input, plane.inlier );

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
        pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewport);
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier", viewport);

        if( display_plane_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0)
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *cloud, cen);
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.2;
            p2.y = p1.y + plane.coefficients[1]*0.2;
            p2.z = p1.z + plane.coefficients[2]*0.2;
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
        }
    }

    // boundary
    if( display_plane_boundary_ && display_plane_projected_inlier_ )
    {
        r = rng.uniform(0.0, 255.0);
        g = rng.uniform(0.0, 255.0);
        b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( plane.cloud_boundary, r, g, b);
        pcl_viewer_->addPointCloud( plane.cloud_boundary, color_boundary, id+"_boundary", viewport );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary", viewport);
    }
    else if( display_plane_boundary_ )
    {
        PointCloudTypePtr cloud_boundary = getPointCloudFromIndices( input, plane.boundary_inlier );
        r = rng.uniform(0.0, 255.0);
        g = rng.uniform(0.0, 255.0);
        b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( cloud_boundary, r, g, b);
        pcl_viewer_->addPointCloud( cloud_boundary, color_boundary, id+"_boundary", viewport );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary", viewport);

    }

    // hull
    if( display_plane_hull_ && display_plane_projected_inlier_ )
    {
        r = rng.uniform(0.0, 1.0);
        g = rng.uniform(0.0, 1.0);
        b = rng.uniform(0.0, 1.0);

        const int num = plane.cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                pcl_viewer_->addLine(plane.cloud_hull->points[i-1], plane.cloud_hull->points[i], r, g, b, ss.str(), viewport );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            pcl_viewer_->addLine(plane.cloud_hull->points[0], plane.cloud_hull->points[num-1], r, g, b, ss.str(), viewport );
        }

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_hull( plane.cloud_hull, r, g, b);
//        pcl_viewer_->addPointCloud( plane.cloud_hull, color_hull, id+"_hull", viewpoint );
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_hull", viewport );

    }
    else if( display_plane_hull_ )
    {
        PointCloudTypePtr cloud_hull = getPointCloudFromIndices( input, plane.hull_inlier );
        r = rng.uniform(0.0, 1.0);
        g = rng.uniform(0.0, 1.0);
        b = rng.uniform(0.0, 1.0);

        const int num = cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                pcl_viewer_->addLine(cloud_hull->points[i-1], cloud_hull->points[i], r, g, b, ss.str(), viewport );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            pcl_viewer_->addLine(cloud_hull->points[0], cloud_hull->points[num-1], r, g, b, ss.str(), viewport );
        }
    }
}


void PlaneViewer::displayLinesAndNormalsSingle( const PointCloudTypePtr &input,
                                            std::vector<PlaneFromLineSegment::LineType> &lines,
                                            std::vector<PlaneFromLineSegment::NormalType> &normals)
{
    if(display_line_cloud_)
    {
        for(int j = 0; j < lines.size(); j++)
        {
            stringstream ss;
            ss << "line_" << j;
            pclViewerLineRegionSingle( input, lines[j], ss.str() );
        }
    }

    if(display_normal_)
    {
        for(int j = 0; j < normals.size(); j++)
        {
            stringstream ss;
            ss << "normal_" << j;
            pclViewerNormalSingle( input, normals[j], ss.str() );
        }
    }
}

void PlaneViewer::pclViewerLineRegionSingle( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id )
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
    single_viewer_->addPointCloud(cloud, color, id);
    single_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);

}

void PlaneViewer::pclViewerNormalSingle( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id )
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    if( display_normal_arrow_ )
    {
        // add a line
        PointType p1, p2;
        p1 = normal.centroid;
        // check centroid
        if( p1.z == 0)
        {
            Eigen::Vector4f cen;
            pcl::compute3DCentroid( *input, normal.inliers, cen);
            p1.x = cen[0];
            p1.y = cen[1];
            p1.z = cen[2];
        }

        p2.x = p1.x + normal.coefficients[0]*0.10;
        p2.y = p1.y + normal.coefficients[1]*0.10;
        p2.z = p1.z + normal.coefficients[2]*0.10;


        single_viewer_->addArrow(p2, p1, 0.0, 0.0, 0.0, false, id+"_arrow");
//        single_viewer_->addArrow(p2, p1, r/255.0, g/255.0, b/255.0, false, id+"_arrow");

        // add a sphere
//        single_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewpoint);
    }
    // add inlier
    PointCloudTypePtr cloud (new PointCloudType );

    for(int i = 0; i < normal.inliers.size(); i++)
    {
        cloud->points.push_back( input->points[normal.inliers[i]] );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
    single_viewer_->addPointCloud(cloud, color, id+"_inlier");
    single_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier");

}

void PlaneViewer::viewerReconfigCallback(plane_segment::ViewerConfig &config, uint32_t level)
{
    display_input_cloud_ = config.display_input_cloud;
    display_scanline_cloud_ = config.display_scanline_cloud;
    display_line_cloud_ = config.display_line_cloud;
    display_normal_ = config.display_normal;
    display_normal_arrow_ = config.display_normal_arrow;
    display_plane_ = config.display_plane;
    display_plane_arrow_ = config.display_plane_arrow;
    display_plane_inlier_ = config.display_plane_inlier;
    display_plane_projected_inlier_ = config.display_plane_projected_inlier;
    display_plane_boundary_ = config.display_plane_boundary;
    display_plane_hull_ = config.display_plane_hull;

}

