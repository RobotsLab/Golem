/** @file Cloud.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4267 4291 4244 4373 4305 4996 4334)
#endif // WIN32
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
	#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif // EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
//#include <Golem/App/Data.h>
//#include <Golem/Plugin/Renderer.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/Mat34.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Tools/Defs.h>

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/fpfh.h>
#ifdef WIN32
	#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

// Legacy point with curvature type definition
struct EIGEN_ALIGN16 PointCurv: public Cloud::Point, public pcl::PrincipalCurvatures {
	// Reset all members to the default values
	PointCurv() {
		principal_curvature_x = principal_curvature_y = principal_curvature_z = golem::numeric_const<float>::ZERO;
		pc1 = pc2 = golem::numeric_const<float>::ZERO;
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// Legacy point with curvature PCL point registration
POINT_CLOUD_REGISTER_POINT_STRUCT(
	PointCurv,
	(float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(float, curvature, curvature)
	(float, principal_curvature_x, principal_curvature_x)(float, principal_curvature_y, principal_curvature_y)(float, principal_curvature_z, principal_curvature_z)(float, pc1, pc1)(float, pc2, pc2)
)

//------------------------------------------------------------------------------

const char* golem::Cloud::Feature::TypeName[TYPE_SIZE] = {
	"default",
	"curvature",
	"narf36",
	"fpfh",
};

const size_t golem::Cloud::Feature::TypeSize[TYPE_SIZE] = {
	0,
	2,
	36,
	33,
};

golem::Cloud::Feature::Type golem::Cloud::Feature::getType(const std::string& type) {
	if (!type.compare(TypeName[(size_t)TYPE_DEFAULT]))
		return TYPE_DEFAULT;
	else if (!type.compare(TypeName[(size_t)TYPE_PRINCIPAL_CURVATURE]))
		return TYPE_PRINCIPAL_CURVATURE;
	else if (!type.compare(TypeName[(size_t)TYPE_NARF36]))
		return TYPE_NARF36;
	else if (!type.compare(TypeName[(size_t)TYPE_FPFH]))
		return TYPE_FPFH;
	else
		throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::Feature::getType(): unknown feature type %s", type.c_str());
}

const char* golem::Cloud::Appearance::ModeName[MODE_SIZE] = {
	"None",
	"Point",
	"Normal",
	"Frame",
	"Feature",
};

void golem::XMLData(const std::string &attr, Cloud::Appearance::Mode& val, golem::XMLContext* context, bool create) {
	if (create) {
		std::string mode =
			val == Cloud::Appearance::MODE_NONE ? "none" :
			val == Cloud::Appearance::MODE_POINT ? "point" :
			val == Cloud::Appearance::MODE_NORMAL ? "normal" :
			val == Cloud::Appearance::MODE_FRAME ? "featureFrame" :
			val == Cloud::Appearance::MODE_FEATURE ? "feature" : "unknown";
		XMLData(attr, mode, context, true);
	}
	else {
		std::string mode = "none";
		XMLData(attr, mode, context, false);
		if (!mode.compare("none"))
			val = Cloud::Appearance::MODE_NONE;
		else if (!mode.compare("point"))
			val = Cloud::Appearance::MODE_POINT;
		else if (!mode.compare("normal"))
			val = Cloud::Appearance::MODE_NORMAL;
		else if (!mode.compare("featureFrame"))
			val = Cloud::Appearance::MODE_FRAME;
		else if (!mode.compare("feature"))
			val = Cloud::Appearance::MODE_FEATURE;
		else
			throw MsgXMLParser(Message::LEVEL_ERROR, "XMLData(Cloud::Appearance::Mode&): unknown mode: %s", mode.c_str());;
	}
}

void golem::Cloud::Appearance::xmlData(golem::XMLContext* context, bool create) const {
	golem::XMLData("mode", const_cast<Mode&>(mode), context, create);
	golem::XMLData("mode_3d", const_cast<bool&>(mode3D), context, create);
	golem::XMLData("mode_3d_a", const_cast<golem::U32&>(mode3DA), context, create);
	golem::XMLData("point_size", const_cast<golem::Real&>(pointSize), context, create);
	golem::XMLData("frame_num", const_cast<golem::U32&>(frameNum), context, create);
	golem::XMLData(const_cast<golem::Vec3&>(frameSize), context->getContextFirst("frame_size", create), create);
	golem::XMLData("camera_frame", const_cast<bool&>(cameraFrame), context, create);
	golem::XMLData("camera_frame_size", const_cast<golem::Real&>(cameraFrameSize), context, create);
	golem::XMLData("override", const_cast<bool&>(colourOverride), context->getContextFirst("colour", create), create);
	golem::XMLData(const_cast<golem::RGBA&>(colour), context->getContextFirst("colour", create), create);
	try {
		golem::XMLData("feature_curv_pow", const_cast<golem::Real&>(featureCurvPow), context, create);
	}
	catch (const golem::MsgXMLParserAttributeNotFound&) {
	}

	try {
		golem::XMLData("size", const_cast<golem::Real&>(clusterSize), context->getContextFirst("clustering", create), create);
		golem::XMLData(const_cast<golem::RGBA&>(clusterColour), context->getContextFirst("clustering colour", create), create);
	}
	catch (const golem::MsgXMLParser&) {
		if (create)
			throw;
	}
}

//------------------------------------------------------------------------------

void golem::Cloud::Import::xmlData(golem::XMLContext* context, bool create) {
	golem::Import::xmlData(context, create);

	golem::XMLData(colour, context->getContextFirst("colour", create), create);
	try {
		Mat34Seq seq;
		golem::XMLData(seq, seq.max_size(), const_cast<golem::XMLContext*>(context), "transform");
		transform.setId();
		for (Mat34Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
			transform.multiply(transform, *i);
	}
	catch (const golem::MsgXMLParser&) {}
}

void golem::Cloud::Import::pointCloudXYZNormal(golem::Context& context, const std::string& path, PointSeq& points) const {
	bool empty = true;
	golem::Import::pointCloudXYZNormal(path, [&] (const Vec3& p, const Vec3& n) {
		if (empty) {
			points.clear();
			empty = false;
		}
		points.push_back(make<Point>(p, n, this->colour));
	});
	if (empty)
		throw Message(Message::LEVEL_ERROR, "golem::Cloud::Import::pointCloudXYZNormal(): empty point cloud");
	downsample(context, points);
	setSensorFrame(Mat34::identity(), points);
}

void golem::Cloud::Import::pointCloudObj(golem::Context& context, const std::string& path, PointSeq& points) const {
	Vec3Seq vertices;
	TriangleSeq triangles;
	golem::Import::pointCloudObj(path, vertices, triangles);
	points.clear();
	generate(context, vertices, triangles, points);
}

void golem::Cloud::Import::pointCloudPly(golem::Context& context, const std::string& path, PointSeq& points) const {
	Vec3Seq vertices;
	TriangleSeq triangles;
	golem::Import::pointCloudPly(path, vertices, triangles);
	points.clear();
	generate(context, vertices, triangles, points);
}

void golem::Cloud::Import::pointCloudPCLXYZRGBA(golem::Context& context, const std::string& path, PointSeq& points) const {
	pcl::PointCloud<pcl::PointXYZRGBA> inp;
	if (pcl::PCDReader().read(path, inp) < 0)
		throw Message(Message::LEVEL_ERROR, "golem::Cloud::Import::pointCloudPCLXYZRGBA(): pcl::PCDReader: unable to read from %s", path.c_str());
	pcl::copyPointCloud<pcl::PointXYZRGBA, Point>(inp, points);
	//transform(getSensorFrame(points), points, points);
}

void golem::Cloud::Import::generate(golem::Context& context, const Vec3Seq& vertices, const TriangleSeq& triangles, PointSeq& points) const {
	golem::Rand rand(context.getRandSeed());
	golem::Import::generate(rand, vertices, triangles, [&] (const Vec3& p, const Vec3& n) {
		points.push_back(make<Point>(p, n, colour));
	});
	setSensorFrame(Mat34::identity(), points);
}

void golem::Cloud::Import::downsample(golem::Context& context, PointSeq& points) const {
	golem::Rand rand(context.getRandSeed());
	golem::Import::downsample(rand, points);
}

//------------------------------------------------------------------------------

golem::Mat34 golem::Cloud::diff(const golem::Mat34& a, const golem::Mat34& b) {
	golem::Mat34 ab;
	ab.setInverseRT(a); // a.R*(a.R)^T = Id, det(a.R) = 1 (rotation matrix)
	ab.multiply(b, ab);
	return ab;
}

golem::Mat34 golem::Cloud::body(const golem::Mat34& a, const golem::Mat34& b) {
	golem::Mat34 ab;
	ab.setInverseRT(a); // a.R*(a.R)^T = Id, det(a.R) = 1 (rotation matrix)
	ab.multiply(ab, b);
	return ab;
}

void golem::Cloud::outRemRad(golem::Context& context, const OutRemDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);
	pcl::RadiusOutlierRemoval<Point> ror;
	ror.setInputCloud(getPtr(inp));
	ror.setRadiusSearch(desc.radius);
	ror.setMinNeighborsInRadius(desc.minNeighborsInRadius);
	ror.filter(out);
	context.debug("Cloud::outRemRad(): radius: %f, neighbours: %u, (%s) --> (%s)\n", desc.radius, desc.minNeighborsInRadius, inpStr.c_str(), toString(out).c_str());
};

void golem::Cloud::outRemStat(golem::Context& context, const OutRemDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);
	pcl::StatisticalOutlierRemoval<Point> sor;
	sor.setInputCloud(getPtr(inp));
	sor.setMeanK(desc.meanK);
	sor.setStddevMulThresh(desc.stddevMulThreshold);
	sor.filter(out);
	context.debug("Cloud::outRemStat(): meanK: %u, threshold: %f, (%s) --> (%s)\n", desc.meanK, desc.stddevMulThreshold, inpStr.c_str(), toString(out).c_str());
};

void golem::Cloud::outRem(golem::Context& context, const OutRemDesc::Seq& descSeq, const PointSeq& inp, PointSeq& out) {
	// run outlier removal
	if (&out != &inp)
		out = inp;
	for (OutRemDesc::Seq::const_iterator i = descSeq.begin(); i != descSeq.end(); ++i)
		if (i->enabledRadius)
			Cloud::outRemRad(context, *i, out, out);
		else if (i->enabledStatistical)
			Cloud::outRemStat(context, *i, out, out);
}

void golem::Cloud::downsampleWithNormals(golem::Context& context, const DownsampleDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);

	Vec3 min(REAL_MAX), max(-REAL_MAX);
	for (PointSeq::const_iterator i = inp.begin(); i != inp.end(); ++i)
		if (!isNanXYZNormal(*i)) {
			const Vec3 p = getPoint<golem::Real>(*i);
			min.min(p);
			max.max(p);
		}
	const Vec3 d(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
	const size_t dn[3] = { static_cast<size_t>(d[0] / desc.gridLeafSize) + 1, static_cast<size_t>(d[1] / desc.gridLeafSize) + 1, static_cast<size_t>(d[2] / desc.gridLeafSize) + 1 };
	
	typedef std::vector< std::pair<size_t, size_t> > Index;
	Index index;
	index.reserve(inp.size());
	for (size_t i = 0; i < inp.size(); ++i)
		if (!isNanXYZNormal(inp[i])) {
			const Vec3 p = getPoint<golem::Real>(inp[i]);
			const size_t n[3] = { static_cast<size_t>(((p[0] - min[0]) / d[0])*dn[0]), static_cast<size_t>(((p[1] - min[1]) / d[1])*dn[1]), static_cast<size_t>(((p[2] - min[2]) / d[2])*dn[2]) };
			const size_t j =  n[0] + dn[0]*n[1] + dn[0]*dn[1]*n[2];
			index.push_back(std::make_pair(j, i));
		}
	std::sort(index.begin(), index.end(), [] (const Index::value_type& l, const Index::value_type& r) -> bool { return l.first < r.first; });
	
	typedef std::vector< std::pair<Index::const_iterator, Index::const_iterator> > Range;
	Range range;
	range.reserve(index.size());
	range.push_back(std::make_pair(index.begin(), index.end()));
	for (Index::const_iterator i = index.begin(); i != index.end(); ++i)
		if (range.back().first->first != i->first) {
			range.back().second = i;
			range.push_back(std::make_pair(i, index.end()));
		}

	typedef std::vector<const Point*> PointPtrSeq;
	// use affine (linear) combination of normals to approximate its average
	auto average = [] (const PointPtrSeq& seq, Point& point) -> size_t {
		point = Point(); // xyz & normal = {0}
		float rgba[4] = { 0 };
		size_t n = 0;
		for (PointPtrSeq::const_iterator i = seq.begin(); i != seq.end(); ++i)
			if (*i) {
				Point p = **i;
				point.x += p.x;
				point.y += p.y;
				point.z += p.z;
				point.normal_x += p.normal_x;
				point.normal_y += p.normal_y;
				point.normal_z += p.normal_z;
				rgba[0] += p.r;
				rgba[1] += p.g;
				rgba[2] += p.b;
				rgba[3] += p.a;
				++n;
			}
		if (n <= 0)
			return 0;

		// average (centroid)
		point.x /= n;
		point.y /= n;
		point.z /= n;
		// affine combination
		const float magnitude = Math::sqrt(point.normal_x*point.normal_x + point.normal_y*point.normal_y + point.normal_z*point.normal_z);
		if (magnitude < numeric_const<float>::EPS)
			throw Message(Message::LEVEL_ERROR, "Cloud::downsampleWithNormals(): average(): invalid normals");
		const float norm = numeric_const<float>::ONE/magnitude;
		point.normal_x *= norm;
		point.normal_y *= norm;
		point.normal_z *= norm;
		// average, rounding & casting
		point.r = static_cast<uint8_t>(std::min((float)numeric_const<uint8_t>::MAX, Math::round(rgba[0]/n)));
		point.g = static_cast<uint8_t>(std::min((float)numeric_const<uint8_t>::MAX, Math::round(rgba[1]/n)));
		point.b = static_cast<uint8_t>(std::min((float)numeric_const<uint8_t>::MAX, Math::round(rgba[2]/n)));
		point.a = static_cast<uint8_t>(std::min((float)numeric_const<uint8_t>::MAX, Math::round(rgba[3]/n)));

		return n;
	};

	if (&inp != &out)
		out.resize(inp.size());

	// to find surface manifolds, perform normal clustering on grid first, then average each cluster
	// there can be no more than 2 clusters of normals with mutual (planar) angle difference > 2/3 PI (i.e. dot product smaller than -0.5, cos(2/3 PI))
	PointPtrSeq cluster[2];
	for (Range::const_iterator i = range.begin(); i != range.end(); ++i) {
		// initialise clusters
		cluster[0].clear();
		cluster[0].push_back(&inp[i->first->second]);
		cluster[1].clear();
		for (Index::const_iterator j = i->first; ++j != i->second;)
			cluster[1].push_back(&inp[j->second]);

		// normal clustering, iteratively compute distances off all members from cluster[0] to members of cluster[1]
		for (size_t clusterPtr = 0; clusterPtr < cluster[0].size(); ++clusterPtr) {
			const Point* p = cluster[0][clusterPtr];
			for (PointPtrSeq::iterator j = cluster[1].begin(); j != cluster[1].end(); ++j)
				if (*j) {
					const float product = p->normal_x*(*j)->normal_x + p->normal_y*(*j)->normal_y + p->normal_z*(*j)->normal_z;
					// check if it belongs to the first cluster
					if (product > -numeric_const<float>::HALF) {
						// swap points between clusters
						cluster[0].push_back(*j);
						*j = nullptr;
					}
				}
		}

		// average clusters
		Index::const_iterator j = i->first;
		Point point;
		(void)average(cluster[0], point); // there is always at least one cluster
		out[j++->second] = point;
		if (j != i->second) {
			if (average(cluster[1], point) > 0) // if there is a non empty second cluster
				out[j++->second] = point;
			// invalidate remaining points
			while (j != i->second)
				setNanXYZ(out[j++->second]);
		}
	}

	// remove nans
	nanRem(context, out, isNanXYZNormal<Point>);
		
	context.debug("Cloud::downsampleWithNormals(): (%s) --> (%s)\n", inpStr.c_str(), toString(out).c_str());
};

void golem::Cloud::downsampleVoxelGrid(golem::Context& context, const DownsampleDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	//context.warning("Cloud::downsampleVoxelGrid(): VoxelGrid may not interpolate correctly RGBA colour and normals\n");
	const std::string inpStr = toString(inp);
	pcl::VoxelGrid<Point> grid;
	grid.setLeafSize(desc.gridLeafSize, desc.gridLeafSize, desc.gridLeafSize);
	grid.setInputCloud(getPtr(inp));
	PointSeq tmp;
	grid.filter(tmp); // setInputCloud arg != filter arg
	out = tmp;
	std::for_each(out.begin(), out.end(), [](Point& p) {p.a = 255; }); // HACK: VoxelGrid looses colour of grid cells
	normaliseSeq(out, numeric_const<float>::EPS); // HACK: VoxelGrid introduces interpolation errors
	context.debug("Cloud::downsampleVoxelGrid(): (%s) --> (%s)\n", inpStr.c_str(), toString(out).c_str());
}

void golem::Cloud::downsample(golem::Context& context, const DownsampleDesc& desc, const PointSeq& inp, PointSeq& out) {
	if (desc.enabled) {
		if (desc.enabledWithNormals)
			Cloud::downsampleWithNormals(context, desc, inp, out);
		else if (desc.enabledVoxelGrid)
			Cloud::downsampleVoxelGrid(context, desc, inp, out);
	}
	else if (&out != &inp)
		out = inp;
};

void golem::Cloud::segmentation(golem::Context& context, const SegmentationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out) {
	assertValid(prev, isNanXYZ<Point>);
	assertValid(next, isNanXYZ<Point>);
	const std::string prevStr = toString(prev);
	const std::string nextStr = toString(next);
	pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
	pcl::SegmentDifferences<Point> seg;
	seg.setDistanceThreshold(desc.distanceThreshold);
	seg.setInputCloud(getPtr(next));
	seg.setTargetCloud(getPtr(prev));
	seg.setSearchMethod(tree);
	seg.segment(out);
	context.debug("Cloud::segmentation(): (%s), (%s) --> (%s)\n", prevStr.c_str(), nextStr.c_str(), toString(out).c_str());
}

void golem::Cloud::clustering(golem::Context& context, const ClusteringDesc& desc, const PointSeq& cloud, IntSeq& indices, IntSeq& clusters) {
	pcl::PointCloud<pcl::PointXYZ> cloudxyz;
	pcl::copyPointCloud(cloud, cloudxyz);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(getPtr(cloudxyz));

	typedef std::vector<pcl::PointIndices> PCLIndices;
	PCLIndices pclIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance(desc.tolerance);
	ece.setMinClusterSize(desc.minSize);
	ece.setMaxClusterSize(desc.maxSize);
	ece.setSearchMethod(tree);
	ece.setInputCloud(getPtr(cloudxyz));
	ece.extract(pclIndices);

	indices.clear();
	clusters.clear();
	for (PCLIndices::const_iterator cluster = pclIndices.begin(); cluster != pclIndices.end(); ++cluster) {
		clusters.push_back((int)indices.size());
		for (IntSeq::const_iterator index = cluster->indices.begin(); index != cluster->indices.end(); ++index)
			indices.push_back(*index);
	}

	context.debug("Cloud::clustering(): (%s) --> (indices=%u, clusters=%u)\n", toString(cloud).c_str(), (int)indices.size(), (int)clusters.size());
}

void golem::Cloud::normalPCA(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);
	pcl::NormalEstimation<Point, Point> ne;
	ne.setSearchMethod(pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>()));
	ne.setRadiusSearch(desc.radiusSearch);
	ne.setInputCloud(getPtr(inp));
	const Vec3 vp = getSensorOrigin(inp);
	ne.setViewPoint((float)vp.x, (float)vp.y, (float)vp.z);
	out = inp;
	ne.compute(out); // updates normals only!
	context.debug("Cloud::normalPCA(): View point: (%f, %f, %f), (%s) --> (%s)\n", vp.x, vp.y, vp.z, inpStr.c_str(), toString(out).c_str());
}

void golem::Cloud::normalII(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);
	//pcl::MovingLeastSquares<Point, Point> mls;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> nii;
	nii.setNormalEstimationMethod(nii.AVERAGE_3D_GRADIENT);
	nii.setMaxDepthChangeFactor((float)desc.maxDepthChangeFactor);
	nii.setNormalSmoothingSize((float)desc.normalSmoothingSize);
	//nii.setSearchMethod(pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>()));
	nii.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>()));
	//nii.setInputCloud(getPtr(inp));
	pcl::PointCloud<pcl::PointXYZ> inppoints;
	pcl::copyPointCloud(inp, inppoints);
	nii.setInputCloud(getPtr(inppoints));
	const Vec3 vp = getSensorOrigin(inp);
	nii.setViewPoint((float)vp.x, (float)vp.y, (float)vp.z);
	out = inp;
	//mls.process(out); // updates normals only!
	pcl::PointCloud<pcl::PointNormal> outpoints;
	nii.compute(outpoints);
	pcl::copyPointCloud(outpoints, out);
	context.debug("Cloud::normalII(): View point: (%f, %f, %f), (%s) --> (%s)\n", vp.x, vp.y, vp.z, inpStr.c_str(), toString(out).c_str());
}

void golem::Cloud::normalMLS(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);
	//pcl::MovingLeastSquares<Point, Point> mls;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setPolynomialFit(desc.polynomialFit);
	//mls.setSearchMethod(pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>()));
	mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>()));
	mls.setSearchRadius(desc.radiusSearch);
	const golem::_Vec3<float> vp(getSensorOrigin(inp));
	// transform to a viewpoint, find non-Nan size
	pcl::PointCloud<pcl::PointXYZ> inppoints;
	pcl::copyPointCloud(inp, inppoints);
	size_t inpSize = 0;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator i = inppoints.begin(); i != inppoints.end(); ++i)
		if (!isNanXYZ(*i)) {
			++inpSize;
			i->x -= vp.x; i->y -= vp.y; i->z -= vp.z;
		}
	//inppoints.reserve(inp.size());
	//for (PointSeq::const_iterator i = inp.begin(); i != inp.end(); ++i)
	//	if (!isNanXYZ(*i))
	//		inppoints.push_back(pcl::PointXYZ(i->x - vp.x, i->y - vp.y, i->z - vp.z));
	//const size_t inpSize = inppoints.size();
	// process
	mls.setInputCloud(getPtr(inppoints));
	//mls.process(out); // updates normals only!
	pcl::PointCloud<pcl::PointNormal> outpoints;
	mls.process(outpoints);
	// transform from a viewpoint
	for (pcl::PointCloud<pcl::PointNormal>::iterator i = outpoints.begin(); i != outpoints.end(); ++i) {
		if (golem::Math::isPositive(i->x*i->normal_x + i->y*i->normal_y + i->z*i->normal_z)) {
			i->normal_x = -i->normal_x; i->normal_y = -i->normal_y; i->normal_z = -i->normal_z; // HACK: MLS bug - flip normals
		}
		i->x += vp.x; i->y += vp.y; i->z += vp.z;
	}
	// make sure there is 1:1 match between non Nan input cloud and output cloud
	if (outpoints.size() != inpSize)
		throw Message(Message::LEVEL_ERROR, "Cloud::normalMLS(): cloud non-Nan input size %u different than non-Nan output size %u", inpSize, outpoints.size());
	// raw copy including Nan points
	if (&out != &inp) out = inp;
	// re-map non-Nan points and transform from a viewpoint
	size_t j = 0;
	for (PointSeq::iterator i = out.begin(); i != out.end() && j < outpoints.size(); ++i)
		if (!isNanXYZ(*i)) {
			pcl::copyPoint(outpoints[j++], (pcl::PointNormal&)*i);
		}
	context.debug("Cloud::normalMLS(): View point: (%f, %f, %f), (%s) --> (%s)\n", vp.x, vp.y, vp.z, inpStr.c_str(), toString(out).c_str());
}

void golem::Cloud::normal(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out) {
	// run normal estimation
	if (!isNormalisedSeq(inp, desc.normalEps)) {
		if (desc.enabledPCA)
			Cloud::normalPCA(context, desc, inp, out);
		else if (desc.enabledII)
			Cloud::normalII(context, desc, inp, out);
		else if (desc.enabledMLS)
			Cloud::normalMLS(context, desc, inp, out);
		// make sure is normalised
		normaliseSeq(out, desc.normalEps);
	}
	else if (&out != &inp)
		out = inp;
}

void golem::Cloud::registrationIcpnl(golem::Context& context, const RegistrationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out, golem::Mat34& trn) {
	assertValid(prev, isNanXYZ<Point>);
	assertValid(next, isNanXYZ<Point>);
	const std::string prevStr = toString(prev);
	const std::string nextStr = toString(next);
	pcl::IterativeClosestPointNonLinear<Point, Point> icpnl;
	icpnl.setMaxCorrespondenceDistance(desc.maxCorrespondenceDistance);
	icpnl.setTransformationEpsilon(desc.transformationEpsilon);
	icpnl.setMaximumIterations(desc.maximumIterations);
#if PCL_VERSION >= 100700
	icpnl.setInputSource(getPtr(next));
#else
	icpnl.setInputCloud(getPtr(next));
#endif
	icpnl.setInputTarget(getPtr(prev));
	icpnl.align(out, toEigen(trn)); // initial guess is the previous transformation
	trn = fromEigen(icpnl.getFinalTransformation());
	context.debug("Cloud::registrationIcpnl(): Has converged: %s, Fitness score: %f, (%s), (%s) --> (%s)\n", icpnl.hasConverged() ? "YES" : "NO", icpnl.getFitnessScore(), prevStr.c_str(), nextStr.c_str(), toString(out).c_str());
}

void golem::Cloud::registrationIcp(golem::Context& context, const RegistrationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out, golem::Mat34& trn) {
	assertValid(prev, isNanXYZ<Point>);
	assertValid(next, isNanXYZ<Point>);
	const std::string prevStr = toString(prev);
	const std::string nextStr = toString(next);
	pcl::IterativeClosestPoint<Point, Point> icp;
	icp.setMaxCorrespondenceDistance(desc.maxCorrespondenceDistance);
	icp.setRANSACOutlierRejectionThreshold(desc.RANSACOutlierRejectionThreshold);
	icp.setTransformationEpsilon(desc.transformationEpsilon);
	icp.setMaximumIterations(desc.maximumIterations);
#if PCL_VERSION >= 100700
	icp.setInputSource(getPtr(next));
#else
	icp.setInputCloud(getPtr(next));
#endif
	icp.setInputTarget(getPtr(prev));
	icp.align(out, toEigen(trn)); // initial guess is the previous transformation
	trn = fromEigen(icp.getFinalTransformation());
	context.debug("Cloud::registrationIcp(): Has converged: %s, Fitness score: %f, (%s), (%s) --> (%s)\n", icp.hasConverged() ? "YES" : "NO", icp.getFitnessScore(), prevStr.c_str(), nextStr.c_str(), toString(out).c_str());
};

void golem::Cloud::registration(golem::Context& context, const RegistrationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out, golem::Mat34& trn) {
	// run cloud pair alignment
	if (desc.enabled) {
		if (desc.enabledIcp)
			Cloud::registrationIcp(context, desc, prev, next, out, trn);
		else if (desc.enabledIcpnl)
			Cloud::registrationIcpnl(context, desc, prev, next, out, trn);
	}
	else
		pcl::transformPointCloudWithNormals(next, out, toEigen(trn));
}

void golem::Cloud::regionGrowing(golem::Context& context, const RegionGrowingDesc& desc, const PointSeq& inp, IntSeq& indices, IntSeq& clusters) {
	// retrieve points xyz
	pcl::PointCloud<pcl::PointXYZ> cloudxyz;
	pcl::copyPointCloud(inp, cloudxyz);

	// retrieve points with normals
	pcl::PointCloud<pcl::Normal> cloudNormals;
	pcl::copyPointCloud(inp, cloudNormals);

	// create a kd-tree on xyz
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(getPtr(cloudxyz));

	// istance of the region growing class
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	// set parameters
	reg.setMinClusterSize(desc.minClusterSize);
	reg.setMaxClusterSize(desc.maxClusterSize);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(desc.neighbours);
	reg.setInputCloud(getPtr(cloudxyz));
	reg.setInputNormals(getPtr(cloudNormals));
	reg.setSmoothnessThreshold(desc.smoothThreshold);
	reg.setCurvatureThreshold(desc.curvatureThreshold);

	// extract regions
	typedef std::vector<pcl::PointIndices> PCLIndices;
	PCLIndices pclIndices;
	reg.extract(pclIndices);

	indices.clear();
	clusters.clear();
	for (PCLIndices::const_iterator cluster = pclIndices.begin(); cluster != pclIndices.end(); ++cluster) {
		clusters.push_back((int)indices.size());
		for (IntSeq::const_iterator index = cluster->indices.begin(); index != cluster->indices.end(); ++index)
			indices.push_back(*index);
	}

	context.debug("Cloud::region(): (%s) --> (indices=%u, clusters=%u)\n", toString(inp).c_str(), (int)indices.size(), (int)clusters.size());
}

void golem::Cloud::curvature(golem::Context& context, const CurvatureDesc& desc, const PointSeq& inp, PointFeatureSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);
	
	pcl::PointCloud<pcl::PointXYZ> points;
	copy(inp, points, copyPointXYZ<Point, pcl::PointXYZ>);

	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, Point, pcl::PrincipalCurvatures> pce;
	pce.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>()));
	pce.setRadiusSearch(desc.radiusSearch);
	pce.setInputCloud(getPtr(points));
	pce.setInputNormals(getPtr(inp));
	
	pcl::PointCloud<pcl::PrincipalCurvatures> curvatures;
	pce.compute(curvatures);

	// make sure there is 1:1 correspondence with the input points
	if (curvatures.size() != inp.size())
		throw Message(Message::LEVEL_ERROR, "Cloud::curvature(): number of curvatures %u different than points %u", (U32)curvatures.size(), (U32)inp.size());

	// convert curvatures to features
	size_t i = 0;
	copy(inp, out, [&] (const Point& inp, PointFeature& out) {
		copyPointXYZNormalRGBA(inp, out);
		convertPrincipalCurvature(curvatures[i], out);
		++i;
	});

	context.debug("Cloud::curvature(): (%s) --> (%s)\n", inpStr.c_str(), toString(out).c_str());
}

void golem::Cloud::convertPrincipalCurvature(const pcl::PrincipalCurvatures& inp, Feature& out) {
	// principal direction
	out.direction.x = static_cast<Feature::Real>(inp.principal_curvature[0]);
	out.direction.y = static_cast<Feature::Real>(inp.principal_curvature[1]);
	out.direction.z = static_cast<Feature::Real>(inp.principal_curvature[2]);
	// descriptor
	out.descriptor_type = static_cast<Feature::Int>(Feature::TYPE_PRINCIPAL_CURVATURE);
	out.descriptor_size = static_cast<Feature::Int>(2);
	out.descriptor_data[0] = powerScale(static_cast<Feature::Real>(inp.pc1), golem::numeric_const<Feature::Real>::ONE);
	out.descriptor_data[1] = powerScale(static_cast<Feature::Real>(inp.pc2), golem::numeric_const<Feature::Real>::ONE);
	//out.descriptor_data[0] = Math::abs(static_cast<golem::F32>(inp.pc1));
	//out.descriptor_data[1] = Math::abs(static_cast<golem::F32>(inp.pc2));
}

void golem::Cloud::narf36(golem::Context& context, const Narf36Desc& desc, const PointSeq& inp, PointFeatureSeq& out) {
	assertValid(inp, isNanXYZ<Point>);
	const std::string inpStr = toString(inp);

	// convert points, transform to the sensor frame
	const Mat34 frame = getSensorFrame(inp);
	Mat34 frameInv;
	frameInv.setInverse(frame);
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	transform(frameInv, inp, pointCloud, transformPointXYZ<Real, Point, pcl::PointXYZ>);

	// Create RangeImage from point cloud
	pcl::RangeImagePlanar rangeImagePlanar;
	pcl::RangeImage rangeImage;
	const pcl::RangeImage* pRangeImage = nullptr;

	if (desc.usePlanarProjection) {
		// planar projection
		if (desc.imageWidth <= 1 || desc.imageHeight <= 1)
			throw Message(Message::LEVEL_ERROR, "Cloud::narf36(): organised image required");
		rangeImagePlanar.createFromPointCloudWithFixedSize(pointCloud, (int)desc.imageWidth, (int)desc.imageHeight, (float)(REAL_HALF*desc.imageWidth), (float)(REAL_HALF*desc.imageHeight), (float)desc.focalLength, (float)desc.focalLength, Eigen::Affine3f::Identity(), pcl::RangeImage::CAMERA_FRAME, (float)desc.noiseLevel, (float)desc.minRange);
		pRangeImage = &rangeImagePlanar;
		context.debug("Cloud::narf36(): (%s) --> range_image_planar_{size=%u}\n", inpStr.c_str(), rangeImagePlanar.size());
	}
	else {
		// spherical projection
		rangeImage.createFromPointCloud(pointCloud, (float)desc.angularResolution, (float)desc.maxAngleWidth, (float)desc.maxAngleHeight, Eigen::Affine3f::Identity(), pcl::RangeImage::CAMERA_FRAME, (float)desc.noiseLevel, (float)desc.minRange, (int)desc.borderSize);
		pRangeImage = &rangeImage;
		context.debug("Cloud::narf36(): (%s) --> range_image_{size=%u}\n", inpStr.c_str(), rangeImage.size());
	}

	// Extract NARF keypoints
	pcl::PointCloud<int> keypointIndicesCloud;
	if (desc.useKeypoints) {
		pcl::RangeImageBorderExtractor rangeImageBorderExtractor;
		pcl::NarfKeypoint narfKeypoint;
		narfKeypoint.setRangeImageBorderExtractor(&rangeImageBorderExtractor);
		narfKeypoint.setRangeImage(pRangeImage);
		narfKeypoint.getParameters().support_size = (float)desc.supportSize;
		narfKeypoint.compute(keypointIndicesCloud);
		context.debug("Cloud::narf36(): found %u keypoints for cloud (%s)\n", (U32)keypointIndicesCloud.size(), inpStr.c_str());
	}
	else {
		keypointIndicesCloud.resize(pointCloud.size());
		for (size_t i = 0; i < pointCloud.size(); ++i)
			keypointIndicesCloud[i] = (int)i;
		context.debug("Cloud::narf36(): using keypoints for entire cloud (%s)\n", inpStr.c_str());
	}

	// Extract NARF descriptors for keypoints points
	std::vector<int> keypointIndices;
	keypointIndices.resize(keypointIndicesCloud.size());
	for (size_t i = 0; i < keypointIndicesCloud.size(); ++i)
		keypointIndices[i] = keypointIndicesCloud[i];
	pcl::NarfDescriptor narfDescriptor(pRangeImage, &keypointIndices);
	narfDescriptor.getParameters().support_size = (float)desc.supportSize;
	narfDescriptor.getParameters().rotation_invariant = desc.rotationInvariant;
	typedef pcl::PointCloud<pcl::Narf36> NarfDescriptorsCloud;
	NarfDescriptorsCloud narfDescriptorsCloud;
	narfDescriptor.compute(narfDescriptorsCloud);

	const size_t size = Feature::TypeSize[Feature::TYPE_NARF36];
	typedef std::vector<golem::Quat> QuatSeq;
	QuatSeq orientations;
	orientations.reserve(narfDescriptorsCloud.size());
	PointFeatureSeq pointFeatures;

	// convert features
	for (NarfDescriptorsCloud::const_iterator i = narfDescriptorsCloud.begin(); i != narfDescriptorsCloud.end(); ++i) {
		// convert to local frame
		Mat34 featureFrame;
		//featureFrame.p.set(i->x, i->y, i->z);
		//featureFrame.R.fromEuler(Real(i->roll), Real(i->pitch), Real(i->yaw));
		Eigen::Affine3d trn;
		pcl::getTransformation(double(i->x), double(i->y), double(i->z), double(i->roll), double(i->pitch), double(i->yaw), trn);
		featureFrame = fromEigen(trn.matrix());
		// rotate local frame around X to get consistency with normals
		Mat33 rotX;
		rotX.rotX(REAL_PI);
		featureFrame.R.multiply(rotX, featureFrame.R);
		// transform to global frame
		featureFrame.multiply(frame, featureFrame);

		// find nearest point, TODO use knn-search
		Real d = numeric_const<Real>::MAX;
		size_t k = 0;
		for (size_t j = 0; j < inp.size(); ++j) {
			Vec3 p;
			p.setColumn3(&inp[j].x);
			Real dd = p.distanceSqr(featureFrame.p);
			if (d > dd) {
				d = dd;
				k = j;
			}
		}

		// normal (z) and principal direction (x)
		const Vec3 x = featureFrame.R * Vec3::axisX();
		const Vec3 z = featureFrame.R * Vec3::axisZ();

		// update point, normal and principal direction
		PointFeature pointFeature;
		copyRGBA(inp[k], pointFeature);
		convertPointXYZ(featureFrame.p, pointFeature);
		convertNormal(z, pointFeature);
		x.getColumn3(pointFeature.direction_data);

		// update descriptor
		pointFeature.descriptor_type = static_cast<Feature::Int>(Feature::TYPE_NARF36);
		pointFeature.descriptor_size = static_cast<Feature::Int>(size);
		for (size_t j = 0; j < size; ++j)
			pointFeature.descriptor_data[j] = static_cast<Feature::Real>(i->descriptor[j]);
		
		// done
		pointFeatures.push_back(pointFeature);
		orientations.push_back(Quat(featureFrame.R));
	}

	typedef golem::_VecN<Real, Feature::DESCRIPTOR_SIZE> Descriptor;
	const Real distNorm = desc.upsamplingDistFac/desc.supportSize;
	if (desc.upsampling) {
		copyHeader(inp, out);
		out.clear();
		for (PointSeq::const_iterator i = inp.begin(); i != inp.end(); ++i) {
			Mat34 featureFrame;
			featureFrame.p = getPoint<Real>(*i);
			PointFeature pointFeature; // calls PointFeature::clear()
			copyRGBA(*i, pointFeature);
			copyPointXYZ(*i, pointFeature);

			// interpolate
			Real norm = REAL_ZERO;
			Quat orientation(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO);
			Descriptor descriptor(size, REAL_ZERO);
			for (size_t j = 0; j < pointFeatures.size(); ++j) {
				const PointFeature &pf = pointFeatures[j];
				const Real dist = featureFrame.p.distance(getPoint<Real>(pf));
				if (dist > desc.supportSize)
					continue;

				const Real weight = Math::exp(-distNorm*dist);

				// descriptor
				Descriptor d;
				d.assign(pf.descriptor_data, pf.descriptor_data + size);
				descriptor.multiplyAdd(weight, d, descriptor);

				// orientation
				Quat o(orientations[j]);
				if (norm > REAL_ZERO) {
					// duality of quaternions
					Quat q = orientation;
					q.normalise();
					const Quat neg(-o.q0, -o.q1, -o.q2, -o.q3);
					o = q.dot(neg) > q.dot(o) ? neg : o;
				}
				// affine combination
				o *= weight;
				orientation += o;

				// normalisation
				norm += weight;
			}

			// update normal and principal direction
			orientation.normalise();
			featureFrame.R.fromQuat(orientation);
			const Vec3 x = featureFrame.R * Vec3::axisX();
			const Vec3 z = featureFrame.R * Vec3::axisZ();
			convertNormal(z, pointFeature);
			x.getColumn3(pointFeature.direction_data);

			// update descriptor
			pointFeature.descriptor_type = static_cast<Feature::Int>(Feature::TYPE_NARF36);
			pointFeature.descriptor_size = static_cast<Feature::Int>(size);
			descriptor.multiply(REAL_ONE/norm, descriptor);
			for (size_t j = 0; j < size; ++j)
				pointFeature.descriptor_data[j] = static_cast<Feature::Real>(descriptor[j]);

			// done
			out.push_back(pointFeature);
		}
	}
	else {
		// just copy
		out.clear();
		copyHeader(inp, out);
		for (size_t i = 0; i < pointFeatures.size(); ++i)
			out.push_back(pointFeatures[i]);
	}

	context.debug("Cloud::narf36(): found %u descriptors for cloud (%s) -> (%s)\n", (U32)narfDescriptorsCloud.size(), inpStr.c_str(), toString(out).c_str());
}

void golem::Cloud::fpfh(golem::Context& context, const FPFHDesc& desc, const PointFeatureSeq& inp, PointFeatureSeq& out) {
	assertValid(inp, isNanXYZ<PointFeature>);
	const std::string inpStr = toString(inp);

	const Mat34 frame = getSensorFrame(inp);
	Mat34 frameInv;
	frameInv.setInverse(frame);

	pcl::PointCloud<pcl::PointXYZ> points;
	//copy(inp, points, copyPointXYZ<PointFeature, pcl::PointXYZ>);
	transform(frameInv, inp, points, transformPointXYZ<Real, PointFeature, pcl::PointXYZ>);
	setSensorFrame(Mat34::identity(), points);
	pcl::PointCloud<pcl::Normal> normals;
	//copy(inp, normals, copyNormal<PointFeature, pcl::Normal>);
	transform(frameInv, inp, normals, transformNormal<Real, PointFeature, pcl::Normal>);
	setSensorFrame(Mat34::identity(), normals);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(getPtr(points));
	fpfh.setInputNormals(getPtr(normals));
	fpfh.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>()));
	fpfh.setRadiusSearch(desc.radiusSearch);

	pcl::PointCloud<pcl::FPFHSignature33> descriptors;
	fpfh.compute(descriptors);

	// make sure there is 1:1 correspondence with the input points
	if (descriptors.size() != inp.size())
		throw Message(Message::LEVEL_ERROR, "Cloud::fpfh(): number of FPFH descriptors %u different than points %u", (U32)descriptors.size(), (U32)inp.size());

	const size_t size = Feature::TypeSize[Feature::TYPE_FPFH];

	// convert curvatures to features
	size_t i = 0;
	copy(inp, out, [&] (const PointFeature& inp, PointFeature& out) {
		copyPointXYZNormalRGBAFeature(inp, out);
		// keep principal directions, overwrite descriptors
		out.descriptor_type = static_cast<Feature::Int>(Feature::TYPE_FPFH);
		out.descriptor_size = static_cast<Feature::Int>(size);
		for (size_t j = 0; j < size; ++j)
			out.descriptor_data[j] = static_cast<Feature::Real>(descriptors[i].histogram[j]);
		++i;
	});

	context.debug("Cloud::fpfh(): (%s) --> (%s)\n", inpStr.c_str(), toString(out).c_str());
}

//------------------------------------------------------------------------------

void golem::Cloud::load(golem::Context& context, const std::string& path, PointSeq& cloud) {
	// block pcl console messages
	const pcl::console::VERBOSITY_LEVEL pclLevel = pcl::console::getVerbosityLevel();
	golem::ScopeGuard guard([=] () { pcl::console::setVerbosityLevel(pclLevel); });
	pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);

	if (pcl::PCDReader().read(path, cloud) != 0)
		throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::load().pcl::PCDReader(): unable to read from %s", path.c_str());
}

void golem::Cloud::load(golem::Context& context, const std::string& path, PointFeatureSeq& cloud) {
	// block pcl console messages
	const pcl::console::VERBOSITY_LEVEL pclLevel = pcl::console::getVerbosityLevel();
	golem::ScopeGuard guard([=] () { pcl::console::setVerbosityLevel(pclLevel); });
	pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
	
	// try legacy format first
	pcl::PCLPointCloud2 header;
	if (pcl::PCDReader().readHeader(path, header) != 0)
		throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::load().pcl::PCDReader::readHeader(): unable to read from %s", path.c_str());
	bool legacy = false;
	for (const auto &field : header.fields)
		if (!field.name.compare("principal_curvature_x")) {
			legacy = true;
			break;
		}
	
	// read cloud
	if (legacy) {
		pcl::PointCloud<PointCurv> pointCurvSeq;
		if (pcl::PCDReader().read(path, pointCurvSeq) != 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::load().pcl::PCDReader::read(): unable to read from %s", path.c_str());
		// convert
		cloud.clear();
		copy(pointCurvSeq, cloud, [&] (const PointCurv& inp, PointFeature& out) {
			copyPointXYZNormalRGBA(inp, out);
			convertPrincipalCurvature(inp, out);
		});
		//context.verbose("Cloud::load(): %s: legacy point cloud conversion (%s) --> (%s)\n", path.c_str(), toString(pointCurvSeq).c_str(), toString(cloud).c_str());
	}
	else {
		if (pcl::PCDReader().read(path, cloud) != 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::load().pcl::PCDReader::read(): unable to read from %s", path.c_str());
		//context.verbose("Cloud::load(): %s: point cloud (%s)\n", path.c_str(), toString(cloud).c_str());
	}
}

void golem::Cloud::save(golem::Context& context, const std::string& path, const PointSeq& cloud) {
	try {
		pcl::PCDWriter().writeBinaryCompressed(path, cloud);
	}
	catch (std::exception& ex) {
		throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::save().pcl::pcdWrite(): unable to write to %s (%s)", path.c_str(), ex.what());
	}
}

void golem::Cloud::save(golem::Context& context, const std::string& path, const PointFeatureSeq& cloud) {
	try {
		pcl::PCDWriter().writeBinaryCompressed(path, cloud);
	}
	catch (std::exception& ex) {
		throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::save().pcl::pcdWrite(): unable to write to %s (%s)", path.c_str(), ex.what());
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(Cloud::FilterDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled", val.enabled, context, create);
	golem::XMLData("window", val.window, context, create);
	golem::XMLData("samples", val.samples, context, create);
}

void golem::XMLData(Cloud::OutRemDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled_radius", val.enabledRadius, context, create);
	golem::XMLData("enabled_statistical", val.enabledStatistical, context, create);
	golem::XMLData("radius", val.radius, context, create);
	golem::XMLData("min_neighbors_in_radius", val.minNeighborsInRadius, context, create);
	golem::XMLData("mean_k", val.meanK, context, create);
	golem::XMLData("stddev_mul_threshold", val.stddevMulThreshold, context, create);
}

void golem::XMLData(Cloud::DownsampleDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled", val.enabled, context, create);
	golem::XMLData("enabled_with_normals", val.enabledWithNormals, context, create);
	golem::XMLData("enabled_voxel_grid", val.enabledVoxelGrid, context, create);
	golem::XMLData("grid_leaf_size", val.gridLeafSize, context, create);
}

void golem::XMLData(Cloud::SegmentationDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("incremental", val.incremental, context, create);
	golem::XMLData("distance_threshold", val.distanceThreshold, context, create);
}

void golem::XMLData(Cloud::ClusteringDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled", val.enabled, context, create);
	golem::XMLData("tolerance", val.tolerance, context, create);
	golem::XMLData("min_size", val.minSize, context, create);
	golem::XMLData("max_size", val.maxSize, context, create);
}

void golem::XMLData(Cloud::NormalDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled_pca", val.enabledPCA, context, create);
	golem::XMLData("enabled_ii", val.enabledII, context, create);
	golem::XMLData("enabled_mls", val.enabledMLS, context, create);
	golem::XMLData("normal_eps", val.normalEps, context, create);
	golem::XMLData("polynomial_fit", val.polynomialFit, context, create);
	golem::XMLData("radius_search", val.radiusSearch, context, create);
	golem::XMLData("max_depth_change_factor", val.maxDepthChangeFactor, context, create);
	golem::XMLData("normal_smoothing_size", val.normalSmoothingSize, context, create);
}

void golem::XMLData(Cloud::CurvatureDesc &val, XMLContext* xmlcontext, bool create) {
	XMLData("radius_search", val.radiusSearch, xmlcontext, create);
}

void golem::XMLData(Cloud::Narf36Desc& val, golem::XMLContext* xmlcontext, bool create) {
	XMLData("planar_projection", val.usePlanarProjection, xmlcontext, create);
	XMLData("focal_length", val.focalLength, xmlcontext, create);

	XMLData("angular_resolution", val.angularResolution, xmlcontext, create);
	XMLData("max_angle_width", val.maxAngleWidth, xmlcontext, create);
	XMLData("max_angle_height", val.maxAngleHeight, xmlcontext, create);

	XMLData("noise_level", val.noiseLevel, xmlcontext, create);
	XMLData("min_range", val.minRange, xmlcontext, create);
	XMLData("border_size", val.borderSize, xmlcontext, create);

	XMLData("use_keypoints", val.useKeypoints, xmlcontext, create);
	XMLData("support_size", val.supportSize, xmlcontext, create);
	XMLData("rotation_invariant", val.rotationInvariant, xmlcontext, create);

	XMLData("upsampling", val.upsampling, xmlcontext, create);
	XMLData("upsampling_dist_fac", val.upsamplingDistFac, xmlcontext, create);
}

void golem::XMLData(Cloud::FPFHDesc &val, XMLContext* xmlcontext, bool create) {
	XMLData("radius_search", val.radiusSearch, xmlcontext, create);
}

void golem::XMLData(Cloud::RegistrationDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled", val.enabled, context, create);
	golem::XMLData("enabled_icp", val.enabledIcp, context, create);
	golem::XMLData("enabled_icpnl", val.enabledIcpnl, context, create);
	golem::XMLData("max_correspondence_distance", val.maxCorrespondenceDistance, context, create);
	golem::XMLData("ransac_outlier_rejection_threshold", val.RANSACOutlierRejectionThreshold, context, create);
	golem::XMLData("transformation_epsilon", val.transformationEpsilon, context, create);
	golem::XMLData("max_iterations", val.maximumIterations, context, create);
}

void golem::XMLData(Cloud::RegionGrowingDesc &val, golem::XMLContext* context, bool create) {
	golem::XMLData("enabled", val.enabled, context, create);
	golem::XMLData("min_cluster_size", val.minClusterSize, context, create);
	golem::XMLData("max_cluster_size", val.maxClusterSize, context, create);
	golem::XMLData("neighbours", val.neighbours, context, create);
	golem::XMLData("smoothThreshold", val.smoothThreshold, context, create);
	golem::XMLData("curvatureThreshold", val.curvatureThreshold, context, create);
}

void golem::XMLData(Cloud::Desc &val, golem::XMLContext* context, bool create) {
	golem::XMLData(val.filterDesc, context->getContextFirst("filter"), create);

	try {
		XMLData(val.outremAlignment, val.outremAlignment.max_size(), context, "outrem_alignment", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
	try {
		XMLData(val.outremSegmentation, val.outremSegmentation.max_size(), context, "outrem_segmentation", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	golem::XMLData(val.downsampleAlignment, context->getContextFirst("downsample_alignment"), create);
	golem::XMLData(val.downsampleSegmentation, context->getContextFirst("downsample_segmentation"), create);
	golem::XMLData(val.segmentation, context->getContextFirst("segmentation"), create);
	try {
		golem::XMLData(val.clustering, context->getContextFirst("clustering"), create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
	golem::XMLData(val.normal, context->getContextFirst("normal"), create);

	golem::XMLData(val.curvature, context->getContextFirst("curvature"), create);
	golem::XMLData(val.narf36, context->getContextFirst("narf36"), create);
	golem::XMLData(val.fpfh, context->getContextFirst("fpfh"), create);

	golem::XMLData(val.registrationAlignment, context->getContextFirst("registration_alignment"), create);
	golem::XMLData(val.registrationSegmentation, context->getContextFirst("registration_segmentation"), create);

	try {
		golem::XMLData(val.regionGrowing, context->getContextFirst("region_growing"), create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	golem::XMLData("thread_chunk_size", val.threadChunkSize, context, create);
	try {
		XMLData(val.regionDesc, val.regionDesc.max_size(), context->getContextFirst("region"), "bounds", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
	golem::XMLData(val.regionColourSolid, context->getContextFirst("region_colour solid"), create);
	golem::XMLData(val.regionColourWire, context->getContextFirst("region_colour wire"), create);
	try {
		golem::XMLData("final_clip", val.regionFinalClip, context->getContextFirst("region"), create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {
	}
}

