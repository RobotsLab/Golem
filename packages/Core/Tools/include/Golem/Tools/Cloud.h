/** @file Cloud.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_CLOUD_H_
#define _GOLEM_TOOLS_CLOUD_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Defs.h>
#include <Golem/Tools/Import.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Math/Queue.h>
#include <Golem/Math/VecN.h>
#include <Golem/Plugin/Renderer.h>
#include <limits>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Point cloud tools */
class Cloud {
public:
	/** Point type */
	typedef pcl::PointXYZRGBNormal Point;
	/** Point sequence */
	typedef pcl::PointCloud<Point> PointSeq;
	/** Point sequence pointer */
	typedef boost::shared_ptr<PointSeq> PointSeqPtr;
	/** Point sequence queue */
	typedef golem::queue<PointSeq> PointSeqQueue;

	/** Feature type */
	struct EIGEN_ALIGN16 Feature {
		/** Real type */
		typedef golem::F32 Real;
		/** Index type */
		typedef golem::U16 Int;
		/** Vec3 type */
		struct Vec3 {
			Real x;
			Real y;
			Real z;
		};

		/** Feature descriptor type */
		enum Type {
			/** Default/generic */
			TYPE_DEFAULT,
			/** Principal curvature */
			TYPE_PRINCIPAL_CURVATURE,
			/** NARF36 */
			TYPE_NARF36,
			/** FPFH */
			TYPE_FPFH,
			/** Size */
			TYPE_SIZE,
		};

		/** Type name */
		static const char* TypeName [TYPE_SIZE];
		/** Type size */
		static const size_t TypeSize[TYPE_SIZE];

		/** Feature descriptor maximum size */
		static const size_t DESCRIPTOR_SIZE = 36;

		/** Feature direction perpendicular to normal, together with normal specifies feature frame (required) */
		union {
			Vec3 direction;
			Real direction_data[3];
		};

		/** Feature descriptor type (optional) */
		Int descriptor_type;
		/** Feature descriptor size (required) */
		Int descriptor_size;
		/** Feature descriptor data (required) */
		Real descriptor_data[DESCRIPTOR_SIZE];

		/** Reset all members to the default values */
		Feature() {
			clear();
		}
		
		/** Resets all members to the default values */
		inline void clear() {
			std::fill(direction_data, direction_data + 3, golem::numeric_const<Real>::ZERO);
			descriptor_type = static_cast<Int>(TYPE_DEFAULT);
			descriptor_size = static_cast<Int>(0);
			std::fill(descriptor_data, descriptor_data + DESCRIPTOR_SIZE, golem::numeric_const<Real>::ZERO);
		}
		/** Sets all members to the specified values */
		inline void set(const Feature& feature) {
			std::copy(feature.direction_data, feature.direction_data + 3, direction_data);
			descriptor_type = feature.descriptor_type;
			descriptor_size = feature.descriptor_size;
			std::copy(feature.descriptor_data, feature.descriptor_data + DESCRIPTOR_SIZE, descriptor_data);
		}

		/** Extract type */
		static Type getType(const std::string& type);
	};
	/** Point with feature type definition */
	struct EIGEN_ALIGN16 PointFeature : public Point, public Feature {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	typedef pcl::PointCloud<PointFeature> PointFeatureSeq;
	/** Point with feature sequence pointer */
	typedef boost::shared_ptr<PointFeatureSeq> PointFeatureSeqPtr;

	/** Appearance */
	class Appearance {
	public:
		/** Mode */
		enum Mode {
			/** Nothing */
			MODE_NONE,
			/** Point */
			MODE_POINT,
			/** Normal */
			MODE_NORMAL,
			/** Local frame*/
			MODE_FRAME,
			/** Feature */
			MODE_FEATURE,
			/** Size */
			MODE_SIZE,
		};

		/** Mode name */
		static const char* ModeName[MODE_SIZE];

		/** Cloud mode */
		Mode mode;
		/** Cloud 3D mode */
		bool mode3D;
		/** Cloud 3D mode transparency */
		golem::U32 mode3DA;

		/** Point size */
		golem::Real pointSize;
		/** Number of frames to show */
		golem::U32 frameNum;
		/** Frame axes size multiplier */
		golem::Vec3 frameSize;

		/** Show camera frame */
		bool cameraFrame;
		/** Camera Frame size */
		golem::Real cameraFrameSize;

		/** Colour override flag */
		bool colourOverride;
		/** Colour override */
		golem::RGBA colour;

		/** Feature curvature power */
		golem::Real featureCurvPow;

		/** View point */
		mutable golem::Vec3 viewPoint;
		/** View direction */
		mutable golem::Vec3 viewDir;

		/** Cluster colour */
		golem::RGBA clusterColour;
		/** Cluster size */
		golem::Real clusterSize;

		/** Constructs description. */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			mode = MODE_POINT;
			mode3D = true;
			mode3DA = 177;
			pointSize = golem::Real(1.0);
			frameNum = 100;
			frameSize.set(golem::Real(0.01), golem::Real(0.01), golem::Real(0.02));
			cameraFrame = true;
			cameraFrameSize = golem::Real(0.05);
			colourOverride = false;
			colour = golem::RGBA::BLACK;
			viewPoint.set(golem::REAL_ZERO, golem::REAL_ZERO, golem::REAL_ONE);
			viewDir.set(golem::REAL_ZERO, golem::REAL_ZERO, -golem::REAL_ONE);
			featureCurvPow = golem::REAL_ONE;
			clusterColour = golem::RGBA::RED;
			clusterSize = golem::Real(2.0);
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(mode3DA <= (golem::U32)golem::numeric_const<golem::U8>::MAX, ac, "mode3DA: > 255", mode3DA);
			Assert::valid(pointSize > golem::REAL_ZERO, ac, "pointSize <= 0");
			Assert::valid(frameNum > 0, ac, "frameNum: 0");
			Assert::valid(frameSize.isPositive(), ac, "frameSize <= 0");
			Assert::valid(cameraFrameSize > 0, ac, "cameraFrameSize: 0");
			Assert::valid(viewPoint.isFinite(), ac, "viewPoint: invalid");
			Assert::valid(viewDir.isFinite(), ac, "viewDir: invalid");
			Assert::valid(golem::Math::isFinite(featureCurvPow), ac, "featureCurvPow: invalid");
			Assert::valid(clusterSize > golem::REAL_ZERO, ac, "clusterSize <= 0");
		}
		/** Reads/writes object from/to a given XML context */
		void xmlData(golem::XMLContext* context, bool create = false) const;

		/** Point drawing */
		template <typename _Seq> void drawPoints(const _Seq& points, golem::DebugRenderer& renderer, const golem::Mat34* trn = nullptr) const {
			renderer.setPointSize(pointSize);
			for (typename _Seq::const_iterator i = points.begin(); i != points.end(); ++i) {
				if (Cloud::isNanXYZ(*i))
					continue;
				const golem::Vec3 p = trn ? *trn * Cloud::getPoint<golem::Real>(*i) : Cloud::getPoint<golem::Real>(*i);
				const golem::RGBA rgba = colourOverride ? colour : Cloud::getColour(*i);
				if (mode == MODE_NORMAL) {
					golem::Vec3 n;
					n.multiplyAdd(frameSize.z, trn ? trn->R * Cloud::getNormal<golem::Real>(*i) : Cloud::getNormal<golem::Real>(*i), p);
					renderer.addLine(p, n, rgba);
				}
				if (mode != MODE_NONE) {
					renderer.addPoint(p, rgba);
				}
			}
			drawCameraFrame(points, renderer, trn);
		}
		/** Features drawing */
		template <typename _Seq, typename _FrameSize> void drawFeatures(const _Seq& points, golem::U32 feature, _FrameSize frameSize, golem::DebugRenderer& renderer, const golem::OpenGL* openGL = nullptr) const {
			const bool ModeFeature = mode == MODE_FEATURE;
			const bool ModeFrame = openGL && mode == MODE_FRAME;

			// draw points
			renderer.setPointSize(pointSize);
			if (!ModeFeature)
				drawPoints(points, renderer);
			else
				drawCameraFrame(points, renderer);

			if (points.empty() || !(ModeFrame || ModeFeature))
				return;

			// find maxima
			golem::Real norm = golem::REAL_EPS;
			for (typename _Seq::const_iterator i = points.begin(); i != points.end(); ++i) {
				if (Cloud::isNanXYZ(*i))
					continue;
				norm = std::max(norm, golem::Math::abs(static_cast<golem::Real>(i->descriptor_data[feature])));
			}
			norm = norm > golem::REAL_EPS ? golem::REAL_ONE / norm : golem::REAL_ONE;

			if (ModeFrame) {
				projections.clear();
				projections.reserve(points.size());
				viewPoint = openGL->viewPoint;
				viewDir = openGL->viewDir;
				// TODO find direction from (x, y) and projection matrix
			}

			for (typename _Seq::const_iterator i = points.begin(); i != points.end(); ++i) {
				if (Cloud::isNanXYZ(*i))
					continue;

				const golem::Vec3 p = Cloud::getPoint<golem::Real>(*i);

				if (ModeFeature) {
					const golem::Real a = norm * golem::Math::abs(i->descriptor_data[feature]);
					renderer.addPoint(p, golem::RGBA(colour._rgba.r, colour._rgba.g, colour._rgba.b, golem::U8(a*golem::numeric_const<golem::U8>::MAX)));
				}
				else if (ModeFrame) {
					// find line - point distances
					Projection projection;
					projection.point = static_cast<const PointFeature*>(&*i);
					golem::Vec3 df; // view point - point
					df.subtract(p, viewPoint);
					golem::Vec3 dp; // view point - point projection onto dir
					dp.multiply(df.dot(viewDir), viewDir);
					golem::Vec3 d; // df - dp
					d.subtract(df, dp);
					projection.distance = d.magnitudeSqr();
					projections.push_back(projection);
				}
			}

			if (ModeFrame) {
				// find closest point
				const size_t frameNum = std::min((size_t)this->frameNum, points.size());
				std::partial_sort(projections.begin(), projections.begin() + frameNum, projections.end(), [] (const Projection& l, const Projection& r) -> bool {return l.distance < r.distance; });
				projections.resize(frameNum);
				// add frames to opengl buffer
				for (Projection::Seq::const_iterator i = projections.begin(); i != projections.end(); ++i) {
					const golem::Mat34 frame = Cloud::getFrame<golem::Real>(*i->point);
					golem::Vec3 size;
					size.arrayMultiply(this->frameSize, frameSize(*i->point));
					if (mode3D)
						renderer.addAxes3D(frame, size, (golem::U8)mode3DA); // 3D
					else
						renderer.addAxes(frame, size); // 2D
				}
			}
		}
		
		/** Cluster drawing */
		template <typename _Real, typename _Seq, typename _Set> void drawCluster(const _Real* xyz, size_t stride, size_t size, const _Seq& points, const _Seq& clusters, const _Set& selection, golem::DebugRenderer& renderer, const golem::Mat34* trn = nullptr) const {
			renderer.setPointSize(clusterSize);

			for (typename _Set::const_iterator i = selection.begin(); i != selection.end(); ++i) {
				const size_t begin = (size_t)clusters[*i];
				const size_t end = *i + 1 < clusters.size() ? (size_t)clusters[*i + 1] : points.size();

				for (size_t j = begin; j < end; ++j) {
					const _Real* ptr = reinterpret_cast<const _Real*>(reinterpret_cast<const golem::U8*>(xyz)+points[j] * stride);
					const golem::_Vec3<_Real> _point(ptr[0], ptr[1], ptr[2]);
					if (Cloud::isNanXYZ(_point))
						continue;

					const golem::Vec3 point = trn ? *trn * golem::Vec3(_point) : golem::Vec3(_point);
					renderer.addPoint(point, clusterColour);
				}
			}
		}

		/** Cluster drawing */
		template <typename _Seq, typename _IndexSeq> void drawRegions(const _Seq& points, const _IndexSeq& indices, const _IndexSeq& clusters, golem::DebugRenderer& renderer) const {
			renderer.setPointSize(clusterSize);

			for (size_t i = 1; i != clusters.size(); ++i) {
				const golem::RGBA colour(golem::U8(rand() * 255), golem::U8(rand() * 255), golem::U8(rand() * 255), golem::U8(255));
				for (int j = clusters[i - 1]; j < clusters[i]; ++j) {
					const golem::Vec3 point = Cloud::getPoint<golem::Real>(points[indices[j]]);
					if (Cloud::isNanXYZ(point))
						continue;
					renderer.addPoint(point, colour);
				}
			}
		}

	protected:
		/** Camera frame drawing */
		template <typename _Seq> void drawCameraFrame(const _Seq& points, golem::DebugRenderer& renderer, const golem::Mat34* trn = nullptr) const {
			if (cameraFrame) {
				const golem::Mat34 frame = trn ? *trn * Cloud::getSensorFrame(points) : Cloud::getSensorFrame(points);
				mode3D ? renderer.addAxes3D(frame, golem::Vec3(cameraFrameSize), (golem::U8)mode3DA) : renderer.addAxes(frame, golem::Vec3(cameraFrameSize));
			}
		}

		/** Point - view direction projection */
		struct Projection {
			typedef std::vector<Projection> Seq;
			golem::Real distance;
			const PointFeature* point;
		};
		
		/** Projections */
		mutable Projection::Seq projections;
	};

	/** Import algortihms */
	class Import : public golem::Import {
	public:
		/** Object RGBA colour of the point */
		golem::RGBA colour;
		/** Transform */
		golem::Mat34 transform;

		/** Constructs the object */
		Import() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::Import::setToDefault();

			colour = golem::RGBA(127, 127, 127, 255); // grey
			transform.setId();
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			golem::Import::assertValid(ac);
			Assert::valid(transform.isValid(), ac, "transform: invalid");
		}
		/** Reads/writes object from/to a given XML context */
		void xmlData(golem::XMLContext* context, bool create = false);

		/** Import point cloud from XYZ and Normal text file */
		void pointCloudXYZNormal(golem::Context& context, const std::string& path, PointSeq& points) const;
		/** Import triangle mesh from obj file */
		void pointCloudObj(golem::Context& context, const std::string& path, PointSeq& points) const;
		/** Import triangle mesh from ply file */
		void pointCloudPly(golem::Context& context, const std::string& path, PointSeq& points) const;

		/** Import point cloud from XYZ and RGBA PCL file */
		void pointCloudPCLXYZRGBA(golem::Context& context, const std::string& path, PointSeq& points) const;

		/** Generate point cloud */
		void generate(golem::Context& context, const Vec3Seq& vertices, const TriangleSeq& triangles, PointSeq& points) const;
		/** Downample point cloud */
		void downsample(golem::Context& context, PointSeq& points) const;

		/** PCL point creation */
		template <typename _PCLPoint> static _PCLPoint make(const golem::Vec3& p, const golem::Vec3& n, const golem::RGBA& colour) {
			_PCLPoint point; // sets members to the default values
			p.getColumn3(&point.x);
			n.getColumn3(&point.normal_x);
			colour.get(point.r, point.b, point.g, point.a);
			return point;
		}
		/** PCL point creation */
		template <typename _PCLPoint> static _PCLPoint make(const golem::Mat34& frame, golem::Vec3 p, golem::Vec3 n, const golem::RGBA& colour) {
			frame.multiply(p, p);
			frame.R.multiply(n, n);
			return make<_PCLPoint>(p, n, colour);
		}
	};

	/** Erase given range */
	template <typename _Seq, typename _Range> static void erase(_Seq& seq, const _Range &range) {
		seq.erase(range.first, range.second);
	}

	/** Point cloud string info */
	template <typename _Seq> static std::string toString(const _Seq& seq) {
		std::stringstream str;
		str << "size: ";
		if (seq.isOrganized())
			str << seq.width << "x" << seq.height;
		else
			str << seq.size();
		return str.str();
	}
	
	/** Test val is NaN */
	template <typename _Type> static inline bool isNan(const _Type& val) {
		return !golem::Math::isFinite(val);
	}
	/** Test if XYZ vector is NaN */
	template <typename _Point> static inline bool isNanXYZ(const _Point& point) {
		return isNan(point.x) || isNan(point.y) || isNan(point.z);
	}
	/** Test if normal vector is NaN */
	template <typename _Point> static inline bool isNanNormal(const _Point& point) {
		return isNan(point.normal_x) || isNan(point.normal_y) || isNan(point.normal_z);
	}
	/** Test if XYZ Normal vector is NaN */
	template <typename _Point> static inline bool isNanXYZNormal(const _Point& point) {
		return isNanXYZ(point) || isNanNormal(point);
	}
	/** Test if point feature is NaN */
	static inline bool isNanFeature(const Feature& point) {
		if (isNanXYZ(point.direction))
			return true;
		for (const golem::F32* ptr = point.descriptor_data, *end = point.descriptor_data + point.descriptor_size; ptr < end; ++ptr)
			if (isNan(*ptr))
				return true;
		return false;
	}
	/** Test if XYZ Normal point feature vector is NaN */
	template <typename _Point> static inline bool isNanXYZNormalFeature(const _Point& point) {
		return isNanXYZNormal(point) || isNanFeature(point);
	}

	/** Set val to NaN */
	template <typename _Type> static inline _Type getNan() {
		return std::numeric_limits<_Type>::quiet_NaN();
	}
	/** Set val to NaN */
	template <typename _Type> static inline void setNan(_Type& val) {
		val = getNan<_Type>();
	}
	/** Set XYZ vector to NaN */
	template <typename _Point> static inline void setNanXYZ(_Point& point) {
		setNan(point.x);
		setNan(point.y);
		setNan(point.z);
	}

	/** Set val to NaN */
	template <typename _Dst, typename _Src> static inline _Dst getValue(const _Src& val) {
		return isNan(val) ? getNan<_Dst>() : static_cast<_Dst>(val);
	}

	/** Test if xyz vector is normalised */
	template <typename _Type> static inline bool isNormalised(const _Type& x, const _Type& y, const _Type& z, _Type eps = golem::numeric_const<_Type>::EPS) {
		return golem::Math::equals(x*x + y*y + z*z, golem::numeric_const<_Type>::ONE, eps);
	}
	/** Normalise xyz vector */
	template <typename _Type> static inline bool normalise(_Type& x, _Type& y, _Type& z, _Type eps = golem::numeric_const<_Type>::EPS) {
		_Type magnitude = golem::Math::sqrt(x*x + y*y + z*z);
		if (magnitude > eps) {
			const _Type length = golem::numeric_const<_Type>::ONE / magnitude;
			x *= length;
			y *= length;
			z *= length;
			return true;
		}
		else
			return false;
	}
	/** Test point is normalised */
	template <typename _Point, typename _Type> static inline bool isNormalised(const _Point& point, _Type eps = golem::numeric_const<_Type>::EPS) {
		return isNormalised(point.normal_x, point.normal_y, point.normal_z, eps);
	}
	/** Normalise xyz vector */
	template <typename _Point, typename _Type> static inline bool normalise(_Point& point, _Type eps = golem::numeric_const<_Type>::EPS) {
		return normalise(point.normal_x, point.normal_y, point.normal_z, eps);
	}
	/** Default point sequence test */
	template <typename _Seq, typename _Type> static inline bool isNormalisedSeq(const _Seq& points, _Type eps = golem::numeric_const<_Type>::EPS) {
		for (typename _Seq::const_iterator i = points.begin(); i != points.end(); ++i)
			if (!isNanXYZ(*i) && !isNormalised(*i, eps))
				return false;
		return true;
	}
	/** Normalise xyz vector */
	template <typename _Seq, typename _Type> static inline void normaliseSeq(_Seq& points, _Type eps = golem::numeric_const<_Type>::EPS) {
		for (typename _Seq::iterator i = points.begin(); i != points.end(); ++i)
			if (!isNanXYZ(*i) && (isNanNormal(*i) || !normalise(*i, eps)))
				setNanXYZ(*i);
	}

	/** Dense point cloud */
	template <typename _Seq> static void setDense(_Seq& points) {
		points.width = 1;
		points.height = (int)points.size();
		points.is_dense = true; // i.e. has no invalid points NaNs/Infs
	}

	/** Resize point cloud */
	template <typename _Seq> static void resize(_Seq& points, size_t size) {
		if (size <= 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::resize(): size must be greater than zero");
		points.resize(size);
		setDense(points);
	}

	/** Point cloud camera origin */
	template <typename _Seq> static inline golem::Vec3 getSensorOrigin(const _Seq& points) {
		return golem::Vec3(points.sensor_origin_.x(), points.sensor_origin_.y(), points.sensor_origin_.z());
	}
	/** Point cloud camera orientation */
	template <typename _Seq> static inline golem::Quat getSensorOrientation(const _Seq& points) {
		return golem::Quat(points.sensor_orientation_.w(), points.sensor_orientation_.x(), points.sensor_orientation_.y(), points.sensor_orientation_.z());
	}
	/** Point cloud camera frame */
	template <typename _Seq> static golem::Mat34 getSensorFrame(const _Seq& points) {
		return golem::Mat34(golem::Mat33(getSensorOrientation(points)), getSensorOrigin(points));
	}

	/** Point cloud camera origin */
	template <typename _Seq> static inline void setSensorOrigin(const golem::Vec3& p, _Seq& points) {
		points.sensor_origin_.x() = (float)p.x;
		points.sensor_origin_.y() = (float)p.y;
		points.sensor_origin_.z() = (float)p.z;
		points.sensor_origin_.w() = (float)0.0;
	}
	/** Point cloud camera orientation */
	template <typename _Seq> static inline void setSensorOrientation(const golem::Quat& q, _Seq& points) {
		points.sensor_orientation_.x() = (float)q.x;
		points.sensor_orientation_.y() = (float)q.y;
		points.sensor_orientation_.z() = (float)q.z;
		points.sensor_orientation_.w() = (float)q.w;
	}
	/** Point cloud camera frame */
	template <typename _Seq> static void setSensorFrame(const golem::Mat34& cameraFrame, _Seq& points) {
		setSensorOrigin(cameraFrame.p, points);
		setSensorOrientation(golem::Quat(cameraFrame.R), points);
	}

	/** Get Golem RGBA colour */
	template <typename _Point> static inline golem::RGBA getColour(const _Point& point) {
		return golem::RGBA(point.r, point.g, point.b, point.a);
	}
	/** Set Golem RGBA colour */
	template <typename _Point> static inline void setColour(const golem::RGBA& rgba, _Point& point) {
		point.r = (uint8_t)rgba._rgba.r;
		point.g = (uint8_t)rgba._rgba.g;
		point.b = (uint8_t)rgba._rgba.b;
		point.a = (uint8_t)rgba._rgba.a;
	}

	/** Get Golem point */
	template <typename _Real, typename _Point> static inline golem::_Vec3<_Real> getPoint(const _Point& point) {
		return golem::_Vec3<_Real>(point.x, point.y, point.z);
	}
	/** Get Golem point with Nan conversion */
	template <typename _Real, typename _Point> static inline golem::_Vec3<_Real> getPointNan(const _Point& point) {
		return isNanXYZ(point) ? golem::_Vec3<_Real>(getNan<_Real>()) : golem::_Vec3<_Real>(point.x, point.y, point.z);
	}
	/** Get point distance */
	template <typename _Real, typename _Point> static inline _Real getPointDistSqr(const _Point& a, const _Point& b) {
		return golem::Math::sqr(a.x - b.x) + golem::Math::sqr(a.y - b.y) + golem::Math::sqr(a.z - b.z);
	}
	/** Set Golem point */
	template <typename _Real, typename _Point> static inline void setPoint(const golem::_Vec3<_Real>& vec, _Point& point) {
		vec.getColumn3(&point.x);
	}

	/** Get Golem normal */
	template <typename _Real, typename _Point> static inline golem::_Vec3<_Real> getNormal(const _Point& point) {
		return golem::_Vec3<_Real>(point.normal_x, point.normal_y, point.normal_z);
	}
	/** Get Golem normal */
	template <typename _Real, typename _Point> static inline golem::_Vec3<_Real> getNormalNan(const _Point& point) {
		return isNanNormal(point) ? golem::_Vec3<_Real>(getNan<_Real>()) : golem::_Vec3<_Real>(point.normal_x, point.normal_y, point.normal_z);
	}
	/** Get normal product */
	template <typename _Real, typename _Point> static inline _Real getNormalDotProduct(const _Point& a, const _Point& b) {
		return a.normal_x*b.normal_x + a.normal_y*b.normal_y + a.normal_z*b.normal_z;
	}
	/** Set Golem normal */
	template <typename _Real, typename _Point> static inline void setNormal(const golem::_Vec3<_Real>& vec, _Point& point) {
		vec.getColumn3(&point.normal_x);
	}

	/** Compute local orientation from normal (z axis) and (principal) direction (x axis) */
	template <typename _RealDst, typename _RealSrc> inline static golem::_Mat33<_RealDst> getOrientation(const golem::_Vec3<_RealSrc>& normal, const golem::_Vec3<_RealSrc>& direction) {
		golem::_Vec3<_RealDst> x(direction), y, z(normal);
		y.cross(z, x);
		y.normalise();
		x.cross(y, z);
		x.normalise();
		golem::_Mat33<_RealDst> orientation;
		orientation.fromAxes(x, y, z);
		return orientation;
	}
	/** Compute frame from point feature */
	template <typename _Real> inline static golem::_Mat34<_Real> getFrame(const PointFeature& point) {
		return golem::_Mat34<_Real>(getOrientation<_Real>(getNormal<_Real>(point), golem::_Vec3<_Real>(point.direction.x, point.direction.y, point.direction.z)), getPoint<_Real>(point));
	}

	/** Point conversion */
	template <typename _Real, typename _Out> inline static void convertPointXYZ(const golem::_Vec3<_Real>& inp, _Out& out) {
		out.x = (float)inp.x;
		out.y = (float)inp.y;
		out.z = (float)inp.z;
		out.data[3] = golem::numeric_const<float>::ZERO;
	}
	/** Point conversion */
	template <typename _Inp, typename _Real> inline static void convertPointXYZ(const _Inp& inp, golem::_Vec3<_Real>& out) {
		out.x = (_Real)inp.x;
		out.y = (_Real)inp.y;
		out.z = (_Real)inp.z;
	}
	/** Normal conversion */
	template <typename _Real, typename _Out> inline static void convertNormal(const golem::_Vec3<_Real>& inp, _Out& out) {
		out.normal_x = (float)inp.x;
		out.normal_y = (float)inp.y;
		out.normal_z = (float)inp.z;
		out.data_n[3] = golem::numeric_const<float>::ZERO;
	}
	/** Normal conversion */
	template <typename _Inp, typename _Real> inline static void convertNormal(const _Inp& inp, golem::_Vec3<_Real>& out) {
		out.x = (_Real)inp.normal_x;
		out.y = (_Real)inp.normal_y;
		out.z = (_Real)inp.normal_z;
	}
	/** RGBA conversion */
	template <typename _Out> inline static void convertRGBA(const golem::RGBA& inp, _Out& out) {
		out.r = inp._rgba.r;
		out.g = inp._rgba.g;
		out.b = inp._rgba.b;
		out.a = inp._rgba.a;
	}
	/** RGBA conversion */
	template <typename _Inp> inline static void convertRGBA(const _Inp& inp, golem::RGBA& out) {
		out._rgba.r = (golem::U8)inp.r;
		out._rgba.g = (golem::U8)inp.g;
		out._rgba.b = (golem::U8)inp.b;
		out._rgba.a = (golem::U8)inp.a;
	}

	/** Cloud header copying */
	template <typename _Inp, typename _Out> inline static void copyHeader(const _Inp& inp, _Out& out) {
		out.is_dense = inp.is_dense;
		out.width = inp.width;
		out.height = inp.height;
		out.sensor_origin_ = inp.sensor_origin_;
		out.sensor_orientation_ = inp.sensor_orientation_;
	}

	/** Point copying */
	template <typename _Inp, typename _Out> inline static void copyPointXYZ(const _Inp& inp, _Out& out) {
		out.x = inp.x;
		out.y = inp.y;
		out.z = inp.z;
		out.data[3] = inp.data[3];
	}
	/** Normal copying */
	template <typename _Inp, typename _Out> inline static void copyNormal(const _Inp& inp, _Out& out) {
		out.normal_x = inp.normal_x;
		out.normal_y = inp.normal_y;
		out.normal_z = inp.normal_z;
		out.data_n[3] = inp.data_n[3];
	}
	/** RGBA copying */
	template <typename _Inp, typename _Out> inline static void copyRGBA(const _Inp& inp, _Out& out) {
		out.r = inp.r;
		out.g = inp.g;
		out.b = inp.b;
		out.a = inp.a;
	}
	/** Feature copying */
	template <typename _Inp, typename _Out> inline static void copyFeature(const _Inp& inp, _Out& out) {
		out.set(inp);
	}

	/** Point copying */
	template <typename _Inp, typename _Out> inline static void copyPointXYZNormal(const _Inp& inp, _Out& out) {
		copyPointXYZ(inp, out);
		copyNormal(inp, out);
	}
	/** Point copying */
	template <typename _Inp, typename _Out> inline static void copyPointXYZNormalRGBA(const _Inp& inp, _Out& out) {
		copyRGBA(inp, out);
		copyPointXYZ(inp, out);
		copyNormal(inp, out);
	}
	/** Point copying */
	template <typename _Inp, typename _Out> inline static void copyPointXYZNormalRGBAFeature(const _Inp& inp, _Out& out) {
		copyRGBA(inp, out);
		copyPointXYZ(inp, out);
		copyNormal(inp, out);
		copyFeature(inp, out);
	}

	/** Point copying */
	template <typename _Inp, typename _Out, typename _CopyPoint> static void copy(const _Inp& inp, _Out& out, _CopyPoint copyPoint) {
		// resize before copying header
		out.resize(inp.size());
		// copy header (width and height)
		copyHeader(inp, out);
		// copy points
		for (size_t i = 0; i < out.size(); ++i)
			copyPoint(inp[i], out[i]);
	}

	/** Point transform */
	template <typename _Real, typename _Inp, typename _Out> inline static void transformPointXYZ(const golem::_Mat34<_Real>& trn, const _Inp& inp, _Out& out) {
		setPoint(golem::_Vec3<_Real>(trn * getPoint<_Real>(inp)), out);
	}
	/** Normal transform */
	template <typename _Real, typename _Inp, typename _Out> inline static void transformNormal(const golem::_Mat34<_Real>& trn, const _Inp& inp, _Out& out) {
		setNormal(golem::_Vec3<_Real>(trn.R * getNormal<_Real>(inp)), out);
	}
	/** Feature transform */
	template <typename _Real, typename _Inp, typename _Out> inline static void transformFeature(const golem::_Mat34<_Real>& trn, const _Inp& inp, _Out& out) {
		out.set(inp);
		const golem::_Vec3<_Real> direction = trn.R * golem::_Vec3<_Real>(inp.direction.x, inp.direction.y, inp.direction.z);
		direction.getColumn3(out.direction_data);
	}

	/** Point transformation */
	template <typename _Real, typename _Inp, typename _Out> inline static void transformPointXYZNormal(const golem::_Mat34<_Real>& trn, const _Inp& inp, _Out& out) {
		transformPointXYZ<_Real>(trn, inp, out);
		transformNormal<_Real>(trn, inp, out);
	}
	/** Point transformation */
	template <typename _Real, typename _Inp, typename _Out> inline static void transformPointXYZNormalRGBA(const golem::_Mat34<_Real>& trn, const _Inp& inp, _Out& out) {
		copyRGBA(inp, out);
		transformPointXYZ<_Real>(trn, inp, out);
		transformNormal<_Real>(trn, inp, out);
	}
	/** Point transformation */
	template <typename _Real, typename _Inp, typename _Out> inline static void transformPointXYZNormalRGBAFeature(const golem::_Mat34<_Real>& trn, const _Inp& inp, _Out& out) {
		copyRGBA(inp, out);
		transformPointXYZ<_Real>(trn, inp, out);
		transformNormal<_Real>(trn, inp, out);
		transformFeature<_Real>(trn, inp, out);
	}

	/** Point transform */
	template <typename _Inp, typename _Out, typename _TransformPoint> static void transform(const golem::Mat34& trn, const _Inp& inp, _Out& out, _TransformPoint transformPoint) {
		// resize before copying header
		out.resize(inp.size());
		// copy header (width and height)
		copyHeader(inp, out);
		// sensor frame
		setSensorFrame(trn * getSensorFrame(inp), out);
		// points
		for (size_t i = 0; i < out.size(); ++i)
			transformPoint(trn, inp[i], out[i]);
	}
	/** Point transform */
	inline static void transform(const golem::Mat34& trn, const PointSeq& inp, PointSeq& out) {
		transform(trn, inp, out, transformPointXYZNormalRGBA<golem::Real, Point, Point>);
	}
	/** Point transform */
	inline static void transform(const golem::Mat34& trn, const PointFeatureSeq& inp, PointFeatureSeq& out) {
		transform(trn, inp, out, transformPointXYZNormalRGBAFeature<golem::Real, PointFeature, PointFeature>);
	}

	/** Filtering algortihm descriptiton */
	class FilterDesc {
	public:
		/** registration */
		bool enabled;

		/** Window size */
		golem::U32 window;
		/** Number of samples */
		golem::U32 samples;

		/** Constructs from description object */
		FilterDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = true;
			window = 10;
			samples = 9;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(window >= samples, ac, "window: < samples");
		}
	};

	/** PCL outlier removal algortihm descriptiton */
	class OutRemDesc {
	public:
		/** Sequence */
		typedef std::vector<OutRemDesc> Seq;

		/** Outlier removal (RadiusOutlierRemoval) */
		bool enabledRadius;
		/** Outlier removal (StatisticalOutlierRemoval) */
		bool enabledStatistical;
		// Outlier removal parameters
		double radius;
		int minNeighborsInRadius;
		int meanK;
		double stddevMulThreshold;

		/** Constructs from description object */
		OutRemDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabledRadius = true;
			enabledStatistical = false;
			radius = 0.005;
			minNeighborsInRadius = 20;
			meanK = 50;
			stddevMulThreshold = 1.0;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isPositive(radius), ac, "radius: <= 0");
			Assert::valid(minNeighborsInRadius > 0, ac, "minNeighborsInRadius: < 1");
			Assert::valid(meanK > 1, ac, "meanK: < 2");
			Assert::valid(golem::Math::isPositive(stddevMulThreshold), ac, "stddevMulThreshold: <= 0");
		}
	};

	/** PCL downsampling algortihm descriptiton */
	class DownsampleDesc {
	public:
		/** Downsampling */
		bool enabled;

		/** Downsampling (with normals) */
		bool enabledWithNormals;
		/** Downsampling (VoxelGrid) */
		bool enabledVoxelGrid;

		// Downsampling parameters
		float gridLeafSize;

		/** Constructs from description object */
		DownsampleDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = false;
			enabledWithNormals = true;
			enabledVoxelGrid = false;
			gridLeafSize = 0.003f;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isPositive(gridLeafSize), ac, "gridLeafSize: <= 0");
		}
	};

	/** PCL segmentation algortihm descriptiton */
	class SegmentationDesc {
	public:
		/** Incremental segmentation */
		bool incremental;
		// segmentation parameters
		double distanceThreshold;

		/** Constructs from description object */
		SegmentationDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			incremental = false;
			distanceThreshold = 0.0003;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isPositive(distanceThreshold), ac, "distanceThreshold: <= 0");
		}
	};

	/** PCL clustering algortihm descriptiton */
	class ClusteringDesc {
	public:
		/** Enabled */
		bool enabled;

		/** Cluster tolerance */
		double tolerance;
		/** Cluster min size */
		int minSize;
		/** Cluster max size */
		int maxSize;

		/** Constructs from description object */
		ClusteringDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = false;
			tolerance = 0.003;
			minSize = 50;
			maxSize = golem::numeric_const<int>::MAX;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isPositive(tolerance), ac, "tolerance: <= 0");
			Assert::valid(minSize > 0, ac, "minSize: <= 0");
			Assert::valid(maxSize > minSize, ac, "maxSize: <= minSize");
		}
	};

	/** PCL normal estimation algortihm descriptiton */
	class NormalDesc {
	public:
		/** PCA (NormalEstimation) */
		bool enabledPCA;
		/** Integral image (IntegralImageNormalEstimation) */
		bool enabledII;
		/** MLS (MovingLeastSquares) */
		bool enabledMLS;
		/** Polynomial (better/slow) vs tangent estimation (worse/faster) */
		bool polynomialFit;
		/** Normal epsilon */
		float normalEps;
		/** Maximum distance to neighbours */
		double radiusSearch;
		/** MaxDepthChangeFactor */
		double maxDepthChangeFactor;
		/** NormalSmoothingSize */
		double normalSmoothingSize;

		/** Constructs from description object */
		NormalDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabledPCA = true;
			enabledII = false;
			enabledMLS = false;
			normalEps = float(1e-5);
			polynomialFit = true;
			radiusSearch = 0.005;
			maxDepthChangeFactor = 0.02;
			normalSmoothingSize = 10.0;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(enabledPCA || enabledII || enabledMLS, ac, "enabledPCA || enabledII || enabledMLS: false");
			Assert::valid(golem::Math::isPositive(normalEps), ac, "normalEps: <= 0");
			Assert::valid(golem::Math::isPositive(radiusSearch), ac, "radiusSearch: <= 0");
			Assert::valid(golem::Math::isPositive(maxDepthChangeFactor), ac, "maxDepthChangeFactor: <= 0");
			Assert::valid(golem::Math::isPositive(normalSmoothingSize), ac, "normalSmoothingSize: <= 0");
		}
	};

	/** PCL curvature estimation algortihm descriptiton */
	class CurvatureDesc {
	public:
		/** Maximum distance to neighbours */
		double radiusSearch;
			
		/** Constructs from description object */
		CurvatureDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			radiusSearch = 0.005;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isPositive(radiusSearch), ac, "radiusSearch: <= 0");
		}
	};

	/** PCL Narf36 estimation algortihm descriptiton */
	class Narf36Desc {
	public:
		/** Use planar projection */
		bool usePlanarProjection;

		/** Image width */
		mutable golem::U32 imageWidth;
		/** Image height */
		mutable golem::U32 imageHeight;
		/** Camera focal length */
		golem::Real focalLength;

		/** The angular difference(in radians) between the individual pixels in the image */
		golem::Real angularResolution;
		/** An angle(in radians) defining the horizontal bounds of the sensor */
		golem::Real maxAngleWidth;
		/** An angle(in radians) defining the vertical bounds of the sensor */
		golem::Real maxAngleHeight;

		/** The distance in meters inside of which the z - buffer */
		golem::Real noiseLevel;
		/** The minimum visible range */
		golem::Real minRange;
		/** The border size */
		golem::I32 borderSize;

		/** Use keypoints */
		bool useKeypoints;
		/** Determines the support size of the feature, meaning the size in the world it covers */
		golem::Real supportSize;
		/** Rotation invariance */
		bool rotationInvariant;

		/** Feature upsampling */
		bool upsampling;
		/** Feature upsampling distance factor (in supportSize units) */
		golem::Real upsamplingDistFac;

		/** Constructs from description object */
		Narf36Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			usePlanarProjection = true;

			imageWidth = 1;
			imageHeight = 1;
			focalLength = golem::Real(525.0);

			angularResolution = golem::Real(0.00872664626); // 0.5 deg
			maxAngleWidth = golem::Real(6.28318530717959); // 360.0 deg
			maxAngleHeight = golem::Real(3.14159265358979); // 180.0 deg

			noiseLevel = golem::Real(0.0);
			minRange = golem::Real(0.0);
			borderSize = 1;

			useKeypoints = false;
			supportSize = golem::Real(0.2);
			rotationInvariant = true;

			upsampling = true;
			upsamplingDistFac = golem::Real(5.0);
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(focalLength > golem::REAL_EPS, ac, "focalLength: < eps");
			Assert::valid(angularResolution > golem::REAL_EPS, ac, "angularResolution: < eps");
			Assert::valid(maxAngleWidth > golem::REAL_EPS, ac, "maxAngleWidth: < eps");
			Assert::valid(maxAngleHeight > golem::REAL_EPS, ac, "maxAngleHeight: < eps");
			Assert::valid(noiseLevel >= golem::REAL_ZERO, ac, "noiseLevel: <= 0");
			Assert::valid(minRange >= golem::REAL_ZERO, ac, "minRange: <= 0");
			Assert::valid(supportSize > golem::REAL_EPS, ac, "supportSize: < eps");
			Assert::valid(upsamplingDistFac >= golem::REAL_ZERO, ac, "upsamplingDistFac: <= 0");
		}
	};

	/** PCL FPFH estimation algortihm descriptiton */
	class FPFHDesc {
	public:
		/** Maximum distance to neighbours */
		double radiusSearch;

		/** Constructs from description object */
		FPFHDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			radiusSearch = 0.02;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isPositive(radiusSearch), ac, "radiusSearch: <= 0");
		}
	};

	/** PCL registration algortihm descriptiton */
	class RegistrationDesc {
	public:
		/** registration */
		bool enabled;

		/** IterativeClosestPoint (RANSAC) */
		bool enabledIcp;
		/** IterativeClosestPointNonLinear */
		bool enabledIcpnl;
		// IPC parameters
		double maxCorrespondenceDistance;
		double RANSACOutlierRejectionThreshold;
		double transformationEpsilon;
		int maximumIterations;
		
		/** Constructs from description object */
		RegistrationDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = false;

			enabledIcp = true;
			enabledIcpnl = false;

			maxCorrespondenceDistance = 0.1;//golem::Math::sqrt(golem::numeric_const<double>::MAX);
			RANSACOutlierRejectionThreshold = 0.05;
			transformationEpsilon = 0.0;
			maximumIterations = 50;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(maximumIterations > 0, ac, "maximumIterations: < 1");
			Assert::valid(golem::Math::isPositive(maxCorrespondenceDistance), ac, "maxCorrespondenceDistance: <= 0");
			Assert::valid(!golem::Math::isNegative(RANSACOutlierRejectionThreshold), ac, "RANSACOutlierRejectionThreshold: < 0");
			Assert::valid(!golem::Math::isNegative(transformationEpsilon), ac, "transformationEpsilon: < 0");
		}
	};

	/** PCL region growing algortihm descriptiton */
	class RegionGrowingDesc {
	public:
		/** Region growing algorithm */
		bool enabled;

		/** Minimum cluster size */
		int minClusterSize;
		/** Maximum cluster size */
		int maxClusterSize;
		/** Number of Neighbour for KD-tree */
		int neighbours;
		/** Threshold smoothness parameter */
		float smoothThreshold;
		/** Curvature threshold */
		float curvatureThreshold;

		/** Constructs from description object */
		RegionGrowingDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = false;

			minClusterSize = 50;
			maxClusterSize = 1000000;
			neighbours = 30;
			smoothThreshold = 3.0 / 180.0 * (float)golem::REAL_PI;
			curvatureThreshold = 1.0;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(minClusterSize > 0, ac, "Invalid minimum cluster size");
			Assert::valid(maxClusterSize > 0, ac, "Invalid maximum cluster size: <= 0");
			Assert::valid(neighbours > 0, ac, "Invalid neighbours <= 0");
			Assert::valid(!golem::Math::isNegative(smoothThreshold), ac, "Invalid smooth threshold: < 0");
			Assert::valid(!golem::Math::isNegative(curvatureThreshold), ac, "Invalid angle threshold: < 0");
		}
	};

	/** PCL descriptiton */
	class Desc {
	public:
		/** Filtering algortihm descriptiton */
		FilterDesc filterDesc;

		/** PCL outlier removal algortihm descriptiton, alignment */
		OutRemDesc::Seq outremAlignment;
		/** PCL outlier removal algortihm descriptiton, segmentation */
		OutRemDesc::Seq outremSegmentation;

		/** PCL downsampling algortihm descriptiton */
		DownsampleDesc downsampleAlignment;
		/** PCL downsampling algortihm descriptiton */
		DownsampleDesc downsampleSegmentation;

		/** PCL segmentation algortihm descriptiton */
		SegmentationDesc segmentation;
		/** PCL clustering algortihm descriptiton */
		ClusteringDesc clustering;

		/** PCL normal estimation algortihm descriptiton */
		NormalDesc normal;

		/** PCL curvature estimation algortihm descriptiton */
		CurvatureDesc curvature;
		/** PCL Narf36 estimation algortihm descriptiton */
		Narf36Desc narf36;
		/** PCL FPFH estimation algortihm descriptiton */
		FPFHDesc fpfh;

		/** PCL registration algortihm descriptiton, alignment */
		RegistrationDesc registrationAlignment;
		/** PCL registration algortihm descriptiton, segmentation */
		RegistrationDesc registrationSegmentation;

		/** PCL region growing algorithm description */
		RegionGrowingDesc regionGrowing;

		/** Point cloud processing thread chunk size */
		golem::U32 threadChunkSize;

		/** Region in global coordinates */
		golem::Bounds::Desc::Seq regionDesc;
		/** Region colour solid */
		golem::RGBA regionColourSolid;
		/** Region colour wireframe */
		golem::RGBA regionColourWire;
		/** Region final clipping vs initial clipping */
		bool regionFinalClip;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			filterDesc.setToDefault();
			
			outremAlignment.clear();
			outremSegmentation.clear();
			
			downsampleAlignment.setToDefault();
			downsampleSegmentation.setToDefault();

			segmentation.setToDefault();
			clustering.setToDefault();

			normal.setToDefault();
			
			curvature.setToDefault();
			narf36.setToDefault();
			fpfh.setToDefault();
			
			registrationAlignment.setToDefault();
			registrationSegmentation.setToDefault();

			regionGrowing.setToDefault();

			threadChunkSize = 100;

			regionDesc.clear();
			regionColourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX/8);
			regionColourWire.set(golem::RGBA::GREEN);
			regionFinalClip = true;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			filterDesc.assertValid(Assert::Context(ac, "filterDesc."));

			for (OutRemDesc::Seq::const_iterator i = outremAlignment.begin(); i != outremAlignment.end(); ++i)
				i->assertValid(Assert::Context(ac, "outremAlignment[]."));
			for (OutRemDesc::Seq::const_iterator i = outremSegmentation.begin(); i != outremSegmentation.end(); ++i)
				i->assertValid(Assert::Context(ac, "outremSegmentation[]."));

			downsampleAlignment.assertValid(Assert::Context(ac, "downsampleAlignment."));
			downsampleSegmentation.assertValid(Assert::Context(ac, "downsampleSegmentation."));

			segmentation.assertValid(Assert::Context(ac, "segmentation."));
			clustering.assertValid(Assert::Context(ac, "clustering."));

			normal.assertValid(Assert::Context(ac, "normal."));
			
			curvature.assertValid(Assert::Context(ac, "curvature."));
			narf36.assertValid(Assert::Context(ac, "narf36."));
			fpfh.assertValid(Assert::Context(ac, "fpfh."));

			registrationAlignment.assertValid(Assert::Context(ac, "registrationAlignment."));
			registrationSegmentation.assertValid(Assert::Context(ac, "registrationSegmentation."));

			regionGrowing.assertValid(Assert::Context(ac, "regionGrowing."));

			Assert::valid(threadChunkSize > 0, ac, "threadChunkSize: < 1");

			for (golem::Bounds::Desc::Seq::const_iterator i = regionDesc.begin(); i != regionDesc.end(); ++i)
				Assert::valid((*i)->isValid(), ac, "regionDesc[]: invalid");
		}
	};

	/** Nan/Inf removal */
	template <typename _Seq, typename _IsNan>  static void assertValid(const _Seq& seq, _IsNan isNan) {
		for (size_t i = 0; i < seq.size(); ++i)
			if (!isNan(seq[i]))
				return;
		throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::assertValid(): empty point cloud");
	};
	/** Nan/Inf removal callback type */
	typedef std::function<void(size_t)> NanRemCallback;
	/** Nan/Inf removal */
	template <typename _Seq, typename _IsNan>  static void nanRem(golem::Context& context, _Seq& seq, _IsNan isNan, NanRemCallback callback = nullptr) {
		const std::string seqStr = toString(seq);
		size_t size = 0;
		for (size_t i = 0; i < seq.size(); ++i) {
			if (callback)
				callback(i);
			if (!isNan(seq[i]))
				seq[size++] = seq[i];
		}
		resize(seq, size);
		context.debug("Cloud::nanRem(): (%s) --> (%s)\n", seqStr.c_str(), toString(seq).c_str());
		assertValid(seq, isNan);
	};
	
	/** Region clipping */
	template <typename _Seq, typename _IsNan> static void regionClip(const golem::Bounds::Seq& region, _Seq& seq, _IsNan isNan) {
		// Check initial conditions
		if (region.empty() || seq.empty())
			return;
		// clip point cloud
		for (typename _Seq::iterator k = seq.begin(); k != seq.end(); ++k)
			if (!isNan(*k) && !golem::Bounds::intersect(region.begin(), region.end(), getPoint<golem::Real>(*k)))
				setNanXYZ(*k);
	}
	/** Region clipping */
	template <typename _Seq> static void regionClip(golem::Context& context, const golem::Bounds::Seq& region, size_t threadChunkSize, _Seq& seq, bool silent = false) {
		// Check initial conditions
		if (region.empty())
			throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::regionClip(): Empty region");
		if (seq.empty())
			throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::regionClip(): No points to clip");
		// clip point cloud
		size_t i = 0, inpSize = 0, outSize = 0;
		golem::CriticalSection cs;
		ParallelsTask(context.getParallels(), [&] (ParallelsTask*) {
			size_t size = 0, inv = 0;
			for (size_t j;;) {
				{
					golem::CriticalSectionWrapper csw(cs);
					if (i*threadChunkSize > seq.size()) {
						inpSize += size;
						outSize += size - inv;
						break;
					}
					j = i++;
				}
				for (typename _Seq::iterator k = seq.begin() + j*threadChunkSize, end = seq.begin() + std::min((j + 1)*threadChunkSize, seq.size()); k != end; ++k)
					if (!isNanXYZ(*k)) {
						++size;
						if (!golem::Bounds::intersect(region.begin(), region.end(), getPoint<golem::Real>(*k))) {
							++inv;
							setNanXYZ(*k);
						}
					}
			}
		});
		if (!silent)
			context.debug("Cloud::regionClip(): Non-Nan cloud size (%u) --> (%u)\n", inpSize, outSize);
	}
	/** Region clipping */
	template <typename _Seq> static void regionClip(golem::Context& context, const golem::Bounds::Desc::Seq& regionDesc, size_t threadChunkSize, _Seq& seq, const golem::Mat34* trn = nullptr) {
		// region in global coordinates
		golem::Bounds::Seq region;
		for (golem::Bounds::Desc::Seq::const_iterator i = regionDesc.begin(); i != regionDesc.end(); ++i) {
			region.push_back((*i)->create());
			if (trn) region.back()->multiplyPose(*trn, region.back()->getPose());
		}
		// clip point cloud
		regionClip(context, region, threadChunkSize, seq);
	}
	
	/** Filtering */
	template <typename _Seq> static void filter(golem::Context& context, const FilterDesc& desc, size_t threadChunkSize, const _Seq& inp, PointSeq& out) {
		// check dimensions
		if (inp.size() <= 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::filter(): no input point clouds");
		for (typename _Seq::const_iterator k = inp.begin(); k != inp.end(); ++k)
			if (inp.begin()->size() != k->size())
				throw golem::Message(golem::Message::LEVEL_ERROR, "Cloud::filter(): input point clouds sizes does not match");
		// resize
		if (out.size() != inp.begin()->size())
			out.resize(inp.begin()->size());
		out.width = inp.begin()->width;
		out.height = inp.begin()->height;
		// sensor origins
		typedef std::vector<pcl::PointXYZ> PointXYZSeq;
		PointXYZSeq originSeq;
		for (typename _Seq::const_iterator k = inp.begin(); k != inp.end(); ++k)
			originSeq.push_back(pcl::PointXYZ(k->sensor_origin_.x(), k->sensor_origin_.y(), k->sensor_origin_.z()));
		// run filter
		typedef std::vector<std::pair<size_t, float> > DepthRank;
		auto depthRankCmp = [] (const DepthRank::value_type& l, const DepthRank::value_type& r) -> bool { return l.first < r.first; };
		const size_t samples = std::min(size_t(desc.samples), inp.size());
		size_t i = 0;
		golem::CriticalSection cs;
		ParallelsTask(context.getParallels(), [&] (ParallelsTask*) {
			DepthRank depthRank;
			for (size_t l;;) {
				{
					golem::CriticalSectionWrapper csw(cs);
					if (i*threadChunkSize > out.size())
						break;
					l = i++;
				}
				for (size_t j = l*threadChunkSize, end = std::min((l + 1)*threadChunkSize, out.size()); j < end; ++j) {
					depthRank.clear();
					for (size_t k = 0; k < inp.size(); ++k) {
						const Point point = (*(inp.begin() + k))[j];
						if (isNanXYZ(point))
							continue;
						const pcl::PointXYZ origin = originSeq[k];
						const float d = golem::Math::sqr(point.x - origin.x) + golem::Math::sqr(point.y - origin.y) + golem::Math::sqr(point.z - origin.z);
						depthRank.push_back(std::make_pair(k, d));
					}
					if (depthRank.size() < desc.samples) {
						setNanXYZ(out[j]);
						continue;
					}
					const size_t element = depthRank.size()/2;
					std::nth_element(depthRank.begin(), depthRank.begin() + element + 1, depthRank.end(), depthRankCmp);
					out[j] = (*(inp.begin() + depthRank[element].first))[j];
				}
			}
		});
	}
	
	/** Outlier removal (RadiusOutlierRemoval) */
	static void outRemRad(golem::Context& context, const OutRemDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Outlier removal (StatisticalOutlierRemoval) */
	static void outRemStat(golem::Context& context, const OutRemDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Outlier removal */
	static void outRem(golem::Context& context, const OutRemDesc::Seq& descSeq, const PointSeq& inp, PointSeq& out);
	/** Downsampling on grid with surface manifold extraction */
	static void downsampleWithNormals(golem::Context& context, const DownsampleDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Downsampling (VoxelGrid) */
	static void downsampleVoxelGrid(golem::Context& context, const DownsampleDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Downsampling */
	static void downsample(golem::Context& context, const DownsampleDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Subtracts two point clouds */
	static void segmentation(golem::Context& context, const SegmentationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out);
	/** Point cloud clustering */
	static void clustering(golem::Context& context, const ClusteringDesc& desc, const PointSeq& cloud, IntSeq& indices, IntSeq& clusters);
	/** Estimates mormals of a given point cloud using PCA */
	static void normalPCA(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Estimates mormals of a given point cloud using integral images */
	static void normalII(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Estimates mormals of a given point cloud using moving least squares */
	static void normalMLS(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out);
	/** Estimates mormals of a given point cloud */
	static void normal(golem::Context& context, const NormalDesc& desc, const PointSeq& inp, PointSeq& out);
	/** IterativeClosestPointNonLinear */
	static void registrationIcpnl(golem::Context& context, const RegistrationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out, golem::Mat34& trn);
	/** IterativeClosestPoint (RANSAC) */
	static void registrationIcp(golem::Context& context, const RegistrationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out, golem::Mat34& trn);
	/** Aligns of a range of point clouds */
	static void registration(golem::Context& context, const RegistrationDesc& desc, const PointSeq& prev, const PointSeq& next, PointSeq& out, golem::Mat34& trn);
	/** Clusters a point cloud in topological regions */
	static void regionGrowing(golem::Context& context, const RegionGrowingDesc& desc, const PointSeq& inp, IntSeq& indices, IntSeq& clusters);

	/** Estimates curvature of a given point cloud */
	static void curvature(golem::Context& context, const CurvatureDesc& desc, const PointSeq& inp, PointFeatureSeq& out);
	/** Principal curvature conversion */
	static void convertPrincipalCurvature(const pcl::PrincipalCurvatures& inp, Feature& out);

	/** Estimates Narf36 of a given point cloud */
	static void narf36(golem::Context& context, const Narf36Desc& desc, const PointSeq& inp, PointFeatureSeq& out);

	/** Estimates FPFH of a given point cloud */
	static void fpfh(golem::Context& context, const FPFHDesc& desc, const PointFeatureSeq& inp, PointFeatureSeq& out);

	/** Load */
	static void load(golem::Context& context, const std::string& path, PointSeq& cloud);
	/** Load */
	static void load(golem::Context& context, const std::string& path, PointFeatureSeq& cloud);
	
	/** Save */
	static void save(golem::Context& context, const std::string& path, const PointSeq& cloud);
	/** Save */
	static void save(golem::Context& context, const std::string& path, const PointFeatureSeq& cloud);

	/** Relative transformation: ab*a = b => ab = b*a^-1 */
	static golem::Mat34 diff(const golem::Mat34& a, const golem::Mat34& b);

	/** Relative transformation in body frame of a: body(a, diff(a, b)) = a^-1*diff(a, b)*a = a^-1*b*a^-1*a = a^-1*b */
	static golem::Mat34 body(const golem::Mat34& a, const golem::Mat34& b);

	/** Aligns of a range of point clouds */
	template <typename _Ptr, typename _Ref, typename _Show> static void align(golem::Context& context, const Desc& desc, _Ptr begin, _Ptr end, PointSeq& out, _Ref ref, _Show show) {
		// nothing to align
		if (begin == end)
			return;

		// outlier removal
		outRem(context, desc.outremAlignment, ref(*begin), out);
		// find normals and curvature of the first cloud
		normal(context, desc.normal, out, out); // sets cameraFrame and label
		// remove NaNs
		//nanRem(context, out, isNanXYZNormal<Point>);

		// render
		show(*begin, out);

		// if there are at least two point clouds
		PointSeq current;
		// trn is a difference between actual and guess, initially identity
		golem::Mat34 trn = golem::Mat34::identity();
		for (_Ptr ptr = begin; ++ptr != end;) {
			// outlier removal
			outRem(context, desc.outremAlignment, ref(*ptr), current);
			// find normals and curvature
			normal(context, desc.normal, current, current);
			// remove NaNs
			//nanRem(context, current, isNanXYZNormal<Point>);

			// find transformation such that: out = trn*current, i.e. align current with out
			const golem::Mat34 guess = golem::Mat34::identity();
			trn = trn * guess; // account for a difference from previous iteration
			registration(context, desc.registrationAlignment, out, current, current, trn);
			trn = diff(trn, guess); // difference between actual and guess
			// sum up clouds
			out += current;
			
			// downsample
			downsample(context, desc.downsampleAlignment, out, out);

			// render
			show(*ptr, out);
		}

		// outlier removal
		outRem(context, desc.outremSegmentation, out, out);
		// downsample
		downsample(context, desc.downsampleSegmentation, out, out);

		// remove NaNs
		nanRem(context, out, isNanXYZNormal<Point>);
	}

	/** Mat34 conversion to Matrix4f */
	static Eigen::Matrix4f toEigen(const golem::Mat34& m) {
		Eigen::Matrix4f em;
		m.getColumn44(em.data());
		return em;
	}

	/** Mat34 conversion from Matrix4f */
	static golem::Mat34 fromEigen(const Eigen::Matrix4f& em) {
		golem::Mat34 m;
		m.setColumn44(em.data());
		return m;
	}
	/** Mat34 conversion from Matrix4d */
	static golem::Mat34 fromEigen(const Eigen::Matrix4d& em) {
		golem::Mat34 m;
		m.setColumn44(em.data());
		return m;
	}

	/** Smart pointer wrapper */
	template <typename _Type> static boost::shared_ptr<_Type> getPtr(_Type& val) {
		return boost::shared_ptr<_Type>(&val, [](_Type*) {});
	}
};

/** Reads/writes object from/to a given XML context */
void XMLData(const std::string &attr, Cloud::Appearance::Mode& val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::FilterDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::OutRemDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::DownsampleDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::SegmentationDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::ClusteringDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::NormalDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::CurvatureDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::Narf36Desc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::FPFHDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::RegistrationDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::RegionGrowingDesc &val, golem::XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Cloud::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
	golem::Cloud::PointFeature,
	(float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(float, curvature, curvature)
	(float[3], direction_data, direction_data)(golem::U16, descriptor_type, descriptor_type)(golem::U16, descriptor_size, descriptor_size)(float[golem::Cloud::Feature::DESCRIPTOR_SIZE], descriptor_data, descriptor_data)
)

//------------------------------------------------------------------------------


#endif /*_GOLEM_TOOLS_CLOUD_H_*/
