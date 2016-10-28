#include "utils.h"


void getPointCloudFromIndices( const PointCloudTypePtr &input,
                               pcl::PointIndices &indices,
                               PointCloudTypePtr &output)
{
    output->clear();
    for(int i = 0; i < indices.indices.size(); i++)
    {
        output->points.push_back( input->points[ indices.indices[i] ]);
    }
    output->is_dense = false;
    output->height = 1;
    output->width = output->points.size();
}

void getPointCloudFromIndices( const PointCloudTypePtr &input,
                               std::vector<int> &indices,
                               PointCloudTypePtr &output)
{
    output->clear();
    for(int i = 0; i < indices.size(); i++)
    {
        output->points.push_back( input->points[ indices[i] ]);
    }
    output->is_dense = false;
    output->height = 1;
    output->width = output->points.size();
}

PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                            pcl::PointIndices &indices)
{
    PointCloudTypePtr output (new PointCloudType );
    getPointCloudFromIndices( input, indices, output);
    return output;
}

PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                            std::vector<int> &indices)
{
    PointCloudTypePtr output (new PointCloudType );
    getPointCloudFromIndices( input, indices, output);
    return output;
}

void projectPoints ( const PointCloudTypePtr &input, const std::vector<int> &inlier,
                     const Eigen::Vector4d &model_coefficients, PointCloudType &projected_points )
{
    Eigen::Vector4f coefficients;
    coefficients[0] = model_coefficients[0];
    coefficients[1] = model_coefficients[1];
    coefficients[2] = model_coefficients[2];
    coefficients[3] = model_coefficients[3];
    projectPoints( input, inlier, coefficients, projected_points );
}

void projectPoints ( const PointCloudTypePtr &input, const std::vector<int> &inlier,
                     const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points )
{
    projected_points.header = input->header;
    projected_points.is_dense = input->is_dense;

    Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

    // normalize the vector perpendicular to the plane...
    mc.normalize ();
    // ... and store the resulting normal as a local copy of the model coefficients
    Eigen::Vector4f tmp_mc = model_coefficients;
    tmp_mc[0] = mc[0];
    tmp_mc[1] = mc[1];
    tmp_mc[2] = mc[2];

    // Allocate enough space and copy the basics
    projected_points.points.resize (inlier.size ());
    projected_points.width    = static_cast<uint32_t> (inlier.size ());
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointType>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < inlier.size (); ++i)
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointType, PointType> (input->points[inlier[i]], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inlier.size (); ++i)
    {
        // Calculate the distance from the point to the plane
        Eigen::Vector4f p (input->points[inlier[i]].x,
                            input->points[inlier[i]].y,
                            input->points[inlier[i]].z,
                            1);
        // use normalized coefficients to calculate the scalar projection
        float distance_to_plane = tmp_mc.dot (p);

        pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
        pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
}

void voxelGridDownsample(const PointCloudTypePtr &input, PointCloudTypePtr &output, float leaf_size )
{
    // Create the filtering object
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud ( input );
    sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
    sor.filter ( *output );
}
