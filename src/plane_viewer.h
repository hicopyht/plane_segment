#ifndef PLANE_VIEWER_H
#define PLANE_VIEWER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/core/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <plane_segment/ViewerConfig.h>
#include "utils.h"

class PlaneViewer
{
public:
    PlaneViewer( ros::NodeHandle &nh );

    void displayInputCloud( const PointCloudTypePtr &input );

    void displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport);

    void displayLinesAndNormals( const PointCloudTypePtr &input,
                                 std::vector<PlaneFromLineSegment::LineType> &lines,
                                 std::vector<PlaneFromLineSegment::NormalType> &normals,
                                 int viewport);

    void displayScanlines( const PointCloudTypePtr &input,
                           vector<pcl::PointIndices> &vindices,
                           vector< vector <ScanPoint> > &scans,
                           int viewport);

    void display2DText( const std::string &text, int xpos, int ypos,
                        const std::string &id, int viewport );

    void pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint);

    void pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint);

    void pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport);

    void pclViewerCloud( const PointCloudTypePtr &cloud, const std::string &id = "cloud", const int viewport = 0)
    {
        pcl_viewer_->addPointCloud( cloud, id, viewport );
    }

    //
    void displayLinesAndNormalsSingle( const PointCloudTypePtr &input,
                                       std::vector<PlaneFromLineSegment::LineType> &lines,
                                       std::vector<PlaneFromLineSegment::NormalType> &normals);

    void pclViewerNormalSingle( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id );

    void pclViewerLineRegionSingle( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id );

    void pclViewerCloudSingle( const PointCloudTypePtr &cloud, const std::string &id = "cloud")
    {
        single_viewer_->addPointCloud( cloud, id );
    }

    //
    void clearViewer() { pcl_viewer_->removeAllPointClouds(); pcl_viewer_->removeAllShapes(); }
    void spinViewer( int time = 1) { pcl_viewer_->spinOnce(time); }
    void clearViewerSingle() { single_viewer_->removeAllPointClouds(); single_viewer_->removeAllShapes(); }
    void spinViewerSingle( int time = 1) { single_viewer_->spinOnce(time); }

    //
    const int& vp1() const { return viewer_v1_; }
    const int& vp2() const { return viewer_v2_; }
    const int& vp3() const { return viewer_v3_; }
    const int& vp4() const { return viewer_v4_; }

protected:

    void viewerReconfigCallback( plane_segment::ViewerConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Reconfigure
    dynamic_reconfigure::Server<plane_segment::ViewerConfig> viewer_config_server_;
    dynamic_reconfigure::Server<plane_segment::ViewerConfig>::CallbackType viewer_config_callback_;

    // Visualizer
    pcl::visualization::PCLVisualizer* single_viewer_; // use a big single viewer, the screenshot looks nicer.
    pcl::visualization::PCLVisualizer* pcl_viewer_;
    int viewer_v1_;
    int viewer_v2_;
    int viewer_v3_;
    int viewer_v4_;
    cv::RNG rng;

    // Display parameters
    bool display_input_cloud_;
    bool display_scanline_cloud_;
    bool display_line_cloud_;
    bool display_normal_;
    bool display_normal_arrow_;
    bool display_plane_;
    bool display_plane_arrow_;
    bool display_plane_inlier_;
    bool display_plane_projected_inlier_;
    bool display_plane_boundary_;
    bool display_plane_hull_;

};

#endif // PLANE_VIEWER_H
