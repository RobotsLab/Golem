//#include "stdafx.h"
#include <Golem/App/Tracking/posetracker.h>

#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <iostream>

using namespace pcl::tracking;
using namespace std;


posetracker::posetracker()
{
}


posetracker::~posetracker()
{
}

void posetracker::initTracking(unsigned int nr_threads)
{
	downsampling_grid_size_ = 0.015;

	std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
	default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;

	std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
	std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

	boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
		(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(nr_threads));

	ParticleT bin_size;
	bin_size.x = 0.1f;
	bin_size.y = 0.1f;
	bin_size.z = 0.1f;
	bin_size.roll = 0.1f;
	bin_size.pitch = 0.1f;
	bin_size.yaw = 0.1f;


	//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
	tracker->setMaximumParticleNum(1000);
	tracker->setDelta(0.99);
	tracker->setEpsilon(0.2);
	tracker->setBinSize(bin_size);

	//Set all parameters for  ParticleFilter
	tracker_ = tracker;
	tracker_->setTrans(Eigen::Affine3f::Identity());
	tracker_->setStepNoiseCovariance(default_step_covariance);
	tracker_->setInitialNoiseCovariance(initial_noise_covariance);
	tracker_->setInitialNoiseMean(default_initial_mean);
	tracker_->setIterationNum(1);
	tracker_->setParticleNum(600);
	tracker_->setResampleLikelihoodThr(0.00);
	tracker_->setUseNormal(false);


	//Setup coherence object for tracking
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
		(new ApproxNearestPairPointCloudCoherence<RefPointType>());

	boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
		= boost::shared_ptr<DistanceCoherence<RefPointType> >(new DistanceCoherence<RefPointType>());
	coherence->addPointCoherence(distance_coherence);

	boost::shared_ptr<pcl::search::Octree<RefPointType> > search(new pcl::search::Octree<RefPointType>(0.01));
	coherence->setSearchMethod(search);
	coherence->setMaximumDistance(0.01);

	tracker_->setCloudCoherence(coherence);
}

void posetracker::setTargetCloud(CloudPtr target, const Eigen::Affine3f& trans)
{
	CloudPtr transed_ref(new Cloud);
	pcl::transformPointCloud<RefPointType>(*target, *transed_ref, trans.inverse());
	CloudPtr transed_ref_downsampled(new Cloud);
	gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

	//set reference model and trans
	tracker_->setReferenceCloud(transed_ref_downsampled);
	tracker_->setTrans(trans);
}

void posetracker::gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
}


//Filter along a specified dimension
void posetracker::filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 2.0);
	pass.setKeepOrganized(false);
	pass.setInputCloud(cloud);
	pass.filter(result);
}

Eigen::Affine3f posetracker::runtracker(CloudPtr inputCloud)
{
	boost::mutex::scoped_lock lock(mtx_);
	cloud_pass_.reset(new Cloud);
	cloud_pass_downsampled_.reset(new Cloud);
	filterPassThrough(inputCloud, *cloud_pass_);
	gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);


	//Track the object
	tracker_->setInputCloud(cloud_pass_downsampled_);
	tracker_->compute();
	new_cloud_ = true;

	//============ compute Transformation matrix ============
	pcl::tracking::ParticleXYZRPY result = tracker_->getResult();
	Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
	//transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
	Eigen::Affine3f transs = tracker_->getTrans();

	return transformation;
}
