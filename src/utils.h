#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <plane_from_line/plane_from_line_segment.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>

using namespace std;

typedef pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;

typedef pcl::PointCloud< pcl::PointXYZRGBA > PointCloudXYZRGBA;
typedef PointCloudXYZRGBA::Ptr PointCloudXYZRGBAPtr;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud< PointType > PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef PointCloudType::ConstPtr PointCloudTypeConstPtr;

typedef boost::shared_ptr<const pcl::PointRepresentation< PointType > > PointRepresentationConstPtr;


// RGB Value
typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

/*
  N*P + d = 0
  */
struct PlaneType
{
    PointType centroid;
    Eigen::Vector4f coefficients;
    std::vector<int> inlier;
    std::vector<int> boundary_inlier;
    std::vector<int> hull_inlier;
    PointCloudTypePtr cloud;
    PointCloudTypePtr cloud_boundary;
    PointCloudTypePtr cloud_hull;

    PlaneType() : cloud( new PointCloudType), cloud_boundary( new PointCloudType), cloud_hull( new PointCloudType) {}

};


void getPointCloudFromIndices( const PointCloudTypePtr &input,
                               pcl::PointIndices &indices,
                               PointCloudTypePtr &output);

void getPointCloudFromIndices( const PointCloudTypePtr &input,
                               std::vector<int> &indices,
                               PointCloudTypePtr &output);

PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                            pcl::PointIndices &indices);

PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                            std::vector<int> &indices);

void projectPoints ( const PointCloudTypePtr &input, const std::vector<int> &inlier,
                     const Eigen::Vector4d &model_coefficients, PointCloudType &projected_points );

void projectPoints ( const PointCloudTypePtr &input, const std::vector<int> &inlier,
                     const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points );

void voxelGridDownsample(const PointCloudTypePtr &input, PointCloudTypePtr &output, float leaf_size = 0.02);

#endif // UTILS_H
