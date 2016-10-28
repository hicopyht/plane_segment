#include "plane_segment.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>

#include <pcl/io/pcd_io.h>

#include <signal.h>
#include <termios.h>

// Terminal
bool    terminal_modified_;
termios orig_flags_;
fd_set  stdin_fdset_;
int     maxfd_;

void setupTerminal()
{
    if (terminal_modified_)
       return;

    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);
    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
    terminal_modified_ = true;
}

void restoreTerminal()
{
    if (!terminal_modified_)
        return;

    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);

    terminal_modified_ = false;
}

char readCharFromStdin() {
#ifdef __APPLE__
    fd_set testfd;
    FD_COPY(&stdin_fdset_, &testfd);
#else
    fd_set testfd = stdin_fdset_;
#endif

    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
    return EOF;

    return getc(stdin);
}

using namespace std;


#define USAGE   "Usage: \n" \
                "  plane_segment_bagfile_depth <filename> [<depth_topic>] [<rgb_topic>] [<camera_info_topic>]" \
                "Notes: "\
                "      - Press space to process one message."\
                "      - Press r to reprocess the previous message."\
                "      - Press s to save current pointcloud to pcd file."\

void savePCDFile( const PointCloudTypePtr &cloud_in)
{
    std::stringstream ss;
    ss << "pointcloud_" << cloud_in->header.seq << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), *cloud_in);
    std::cout << GREEN << "Save pcb file: " << ss.str() << RESET << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segment_bagfile_depth_node");
    std::string filename;
//    std::string depth_topic = "/camera/depth/image";
//    std::string rgb_topic = "/camera/rgb/image_color";
//    std::string camera_info_topic = "/camera/depth/camera_info";
    std::string depth_topic = "/head_kinect/depth_registered/image";
    std::string rgb_topic = "/head_kinect/rgb/image_rect_color";
    std::string camera_info_topic = "/head_kinect/depth_registered/camera_info";
    if(argc < 2)
    {
        puts(USAGE);
        std::cout << RED << "You should give the bag file name." << RESET << std::endl;
        ros::shutdown();
    }
    filename = argv[1];
    if(argc >= 3)
    {
        depth_topic = argv[2];
    }
    if(argc >= 4)
    {
        rgb_topic = argv[3];
    }
    if(argc >= 5)
    {
        camera_info_topic = argv[4];
    }

    PlaneSegment ps;

    // Open a bag file
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    cout << GREEN << " Open bag file: " << filename << RESET << endl;

    // Pointcloud topics to load
    std::vector<std::string> topics;
    topics.push_back(depth_topic);
    topics.push_back(rgb_topic);
    topics.push_back(camera_info_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    double duration = (view.getEndTime() - view.getBeginTime()).toSec();

    cout << GREEN << " Get topics: " << depth_topic << ", " << rgb_topic << ", " << camera_info_topic << RESET << endl;
    cout << GREEN << " Duration: " << duration << " seconds." << RESET << endl;
    cout << WHITE << " ------------------------------------------------- " << RESET << endl;
    cout << GREEN << " Press space to process one message." << RESET << endl;
    cout << GREEN << " Press r to reprocess the previous message. " << RESET << endl;
    cout << GREEN << " Press s to save current pointcloud as a pcd file. " << RESET << endl;

    // Setup terminal
    setupTerminal();

    // Load all messages
    ros::Rate loop_rate(20);
    bool paused = true;
    PointCloudTypePtr cloud_in( new PointCloudType );
    PlaneFromLineSegment::CAMERA_PARAMETERS camera_parameters;
    //
    sensor_msgs::ImageConstPtr visual_img_msg;
    sensor_msgs::ImageConstPtr depth_img_msg;
    sensor_msgs::CameraInfoConstPtr cam_info_msg;
    bool depth_ok = false, rgb_ok = false, info_ok = false;
    const double delta_time_threshold = 0.020;
    cout << " view size: " << view.size() << endl;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        cout << " -topic: " << m.getTopic() << endl;
        if (m.getTopic() == depth_topic || ("/" + m.getTopic() == depth_topic))
        {
            // get depth image
            depth_img_msg = m.instantiate<sensor_msgs::Image>();
            depth_ok = true;
            cout << GREEN << " D: " << depth_img_msg->header.stamp.toSec() << RESET << endl;
        }

        if (m.getTopic() == rgb_topic || ("/" + m.getTopic() == rgb_topic))
        {
            // get rgb image
            visual_img_msg = m.instantiate<sensor_msgs::Image>();
            rgb_ok = true;
            cout << GREEN << " RGB: " << visual_img_msg->header.stamp.toSec() << RESET << endl;
        }

        if (m.getTopic() == camera_info_topic || ("/" + m.getTopic() == camera_info_topic))
        {
            // get rgb image
            cam_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
            info_ok = true;
            cout << GREEN << " Info: " << cam_info_msg->header.stamp.toSec() << RESET << endl;
        }

        if( depth_ok && rgb_ok && info_ok)
        {
            // check if synchronous
            double dt = (depth_img_msg->header.stamp - visual_img_msg->header.stamp).toSec();
            cout << GREEN << " dt: " << dt << RESET << endl;
            if( dt > delta_time_threshold) {
                rgb_ok = false;
                continue;
            }
            else if( dt < -delta_time_threshold) {
                depth_ok = false;
                continue;
            }

            depth_ok = false; rgb_ok = false;

            //
            cout << YELLOW << " process sequence: " << depth_img_msg->header.seq << endl;
            cout << " view time = " << (m.getTime() - view.getBeginTime()).toSec()
                 << " / " << duration << RESET << endl;

            // get point cloud
            pcl::PointIndicesPtr indices( new pcl::PointIndices);
            cloud_in = ps.image2PointCloud( visual_img_msg, depth_img_msg, cam_info_msg, camera_parameters, indices);
            cloud_in->header.seq = depth_img_msg->header.seq;
            cout << GREEN << " Compute PC." << RESET << endl;

            ps.setCameraParameter( camera_parameters );
            ps.processCloud( cloud_in );

            // pause
            paused = true;
            while(paused && ros::ok())
            {
                char cin = readCharFromStdin();
                if(cin == ' ')
                {
                    paused = false;
                    break;
                }
                else if(cin == 's')
                {
                    savePCDFile( cloud_in );
                }
                else if(cin == 'r')
                {
                    ps.processCloud( cloud_in );
                }
                ros::spinOnce();
                loop_rate.sleep();
            }

        }

        // exit
        if(!ros::ok())
            break;

    }

    cout << GREEN << "End." << RESET << endl;
    // Restore terminal
    restoreTerminal();
    bag.close();
    ros::spin();
}


