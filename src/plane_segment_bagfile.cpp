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
                "  plane_segment_bagfile <filename> [<topic_name>]" \
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
    ros::init(argc, argv, "plane_segment_bagfile_node");
    std::string filename;
    std::string topic_name = "/camera/rgb/points";
    if(argc < 2)
    {
        puts(USAGE);
        std::cout << RED << "You should give the bag file name." << RESET << std::endl;
        ros::shutdown();
    }
    filename = argv[1];
    if(argc >= 3 && (argv[2][0] == '/'))
    {
        topic_name = argv[2];
    }

    PlaneSegment ps;
    // Open a bag file
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    cout << GREEN << " Open bag file: " << filename << RESET << endl;

    // Pointcloud topics to load
    std::vector<std::string> topics;
    topics.push_back(topic_name);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    cout << GREEN << " Get pointcloud topic: " << topic_name << RESET << endl;
    cout << GREEN << " Press space to process one message." << RESET << endl;
    cout << GREEN << " Press r to reprocess the previous message. " << RESET << endl;
    cout << GREEN << " Press s to save current pointcloud as a pcd file. " << RESET << endl;

    // Setup terminal
    setupTerminal();

    // Load all messages
    ros::Rate loop_rate(20);
    bool paused = true;
    PointCloudTypePtr cloud_in( new PointCloudType );

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        if (m.getTopic() == topic_name || ("/" + m.getTopic() == topic_name))
        {
            // pause
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
            // exit
            if(!ros::ok())
                break;
            // get message
            sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
            cout << BLUE << " - cloud topic: " << cloud->header.seq << RESET << endl;
            // to pcl message
            pcl::fromROSMsg( *cloud, *cloud_in);
            cloud_in->header.seq = cloud->header.seq;
            // process one cloud message
            ps.processCloud( cloud_in );
            // Reset symbol
            paused = true;
        }
    }

    // Restore terminal
    restoreTerminal();
    bag.close();
    ros::spin();
}

