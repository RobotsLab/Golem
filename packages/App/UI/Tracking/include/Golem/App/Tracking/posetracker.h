#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/tracking/particle_filter.h>

class posetracker
{
public:
	typedef pcl::PointXYZRGBA RefPointType;
	typedef pcl::tracking::ParticleXYZRPY ParticleT;
	typedef pcl::PointXYZRGBA Point;
	typedef pcl::PointCloud<Point> Cloud;
	typedef Cloud::Ptr CloudPtr;
	typedef Cloud::ConstPtr CloudConstPtr;
	typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

	posetracker();
	~posetracker();


	void initTracking(unsigned int nr_threads = 0);
	void setTargetCloud(CloudPtr target, const Eigen::Affine3f& trans);
	void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size);
	void filterPassThrough(const CloudConstPtr &cloud, Cloud &result);
	Eigen::Affine3f runtracker(CloudPtr inputCloud);

private:
	CloudPtr cloud_pass_;
	CloudPtr cloud_pass_downsampled_;
	CloudPtr target_cloud;

	boost::mutex mtx_;
	boost::shared_ptr<ParticleFilter> tracker_;
	bool new_cloud_;
	double downsampling_grid_size_;
	int counter;
};

