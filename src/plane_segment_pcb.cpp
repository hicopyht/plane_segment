#include "plane_segment.h"
#include <pcl/io/pcd_io.h>

#define USAGE "Usage: \n" \
              "  plane_segment_pcb <pcb_file>"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segment_pcb_node");
    //
    std::string filename;
    PlaneFromLineSegment::CAMERA_PARAMETERS camera_parameters;
    camera_parameters.width = 640;
    camera_parameters.height = 480;
    camera_parameters.cx = 319.5;
    camera_parameters.cy = 239.5;
    camera_parameters.fx = 525.0;
    camera_parameters.fy = 525.0;
    camera_parameters.scale = 1.0;

    //
    if( argc < 2)
    {
        puts( USAGE );
        ros::shutdown();
    }
    filename = argv[1];

    // read pointcloud
    cout << GREEN << "Load pcd file: " << filename << RESET << endl;
    PointCloudTypePtr cloud (new PointCloudType );
    if (pcl::io::loadPCDFile<PointType> (filename, *cloud) == -1)
    {
        cout << RED << "Couldn't read file test_pcd.pcd" << RESET << endl;
        ros::shutdown();
    }

    if( !cloud->isOrganized() || cloud->width != 640 || cloud->height != 480 )
    {
        cout << RED << "Pointcloud is not organized, exit." << RESET << endl;
        ros::shutdown();
    }

    PlaneSegment ps;
    ps.setCameraParameter( camera_parameters );
    // in loop mode
    ros::Rate loop_rate( 2 );
    cout << GREEN << "Process pointcloud..." << endl;
    while( ros::ok() )
    {
        // process message
        ps.processCloud( cloud );
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}
