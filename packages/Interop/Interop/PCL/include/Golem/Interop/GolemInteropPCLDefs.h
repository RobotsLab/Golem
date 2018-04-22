/** @file GolemInteropPCLDefs.h
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
#ifndef _GOLEM_INTEROP_PCL_PCL_DEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PCL_PCL_DEFS_H_

#include "GolemInteropDefs.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <stdexcept>

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {
	/** FP number conversion */
	template <typename _Dst, typename _Src> _Dst convertPCLFloat(_Src src) {
		return std::isfinite(src) ? static_cast<_Dst>(src) : std::numeric_limits<_Dst>::quiet_NaN();
	}

	/** pcl::PointXYZ point conversion */
	template <typename _PCLPointXYZ> void convertPCLPointXYZ(const _PCLPointXYZ& src, interop::Point3D& dst) {
		dst.position[0] = convertPCLFloat<float_t>(src.x);
		dst.position[1] = convertPCLFloat<float_t>(src.y);
		dst.position[2] = convertPCLFloat<float_t>(src.z);
	}
	/** pcl::PointXYZ point conversion */
	template <typename _PCLPointXYZ> void convertPCLPointXYZ(const interop::Point3D& src, _PCLPointXYZ& dst) {
		dst.x = convertPCLFloat<float>(src.position[0]);
		dst.y = convertPCLFloat<float>(src.position[1]);
		dst.z = convertPCLFloat<float>(src.position[2]);
	}

	/** pcl::PointNormal point conversion */
	template <typename _PCLPointNormal> void convertPCLPointNormal(const _PCLPointNormal& src, interop::Point3D& dst) {
		dst.normal[0] = convertPCLFloat<float_t>(src.normal_x);
		dst.normal[1] = convertPCLFloat<float_t>(src.normal_y);
		dst.normal[2] = convertPCLFloat<float_t>(src.normal_z);
	}
	/** pcl::PointNormal point conversion */
	template <typename _PCLPointNormal> void convertPCLPointNormal(const interop::Point3D& src, _PCLPointNormal& dst) {
		dst.normal_x = convertPCLFloat<float>(src.normal[0]);
		dst.normal_y = convertPCLFloat<float>(src.normal[1]);
		dst.normal_z = convertPCLFloat<float>(src.normal[2]);
	}

	/** pcl::PointRGBA point conversion */
	template <typename _PCLPointRGBA> void convertPCLPointRGBA(const _PCLPointRGBA& src, interop::Point3D& dst) {
		dst.colour.r = static_cast<std::uint8_t>(src.r);
		dst.colour.g = static_cast<std::uint8_t>(src.g);
		dst.colour.b = static_cast<std::uint8_t>(src.b);
		dst.colour.a = static_cast<std::uint8_t>(src.a);
	}
	/** pcl::PointRGBA point conversion */
	template <typename _PCLPointRGBA> void convertPCLPointRGBA(const interop::Point3D& src, _PCLPointRGBA& dst) {
		dst.r = static_cast<std::uint8_t>(src.colour.r);
		dst.g = static_cast<std::uint8_t>(src.colour.g);
		dst.b = static_cast<std::uint8_t>(src.colour.b);
		dst.a = static_cast<std::uint8_t>(src.colour.a);
	}

	/** pcl::PointXYZ point conversion */
	inline void convert(const pcl::PointXYZ& src, interop::Point3D& dst) {
		convertPCLPointXYZ(src, dst);
	}
	/** pcl::PointXYZ point conversion */
	inline void convert(const interop::Point3D& src, pcl::PointXYZ& dst) {
		convertPCLPointXYZ(src, dst);
	}

	/** pcl::PointNormal point conversion */
	inline void convert(const pcl::PointNormal& src, interop::Point3D& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
	}
	/** pcl::PointNormal point conversion */
	inline void convert(const interop::Point3D& src, pcl::PointNormal& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
	}

	/** pcl::PointXYZRGBNormal point conversion */
	inline void convert(const pcl::PointXYZRGBNormal& src, interop::Point3D& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
		convertPCLPointRGBA(src, dst);
	}
	/** pcl::PointXYZRGBNormal point conversion */
	inline void convert(const interop::Point3D& src, pcl::PointXYZRGBNormal& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
		convertPCLPointRGBA(src, dst);
	}

	/** Point cloud conversion */
	template <typename _PCLPoint, typename _Point> void convert(const pcl::PointCloud<_PCLPoint>& src, std::vector<_Point>& dst) {
		dst.resize(src.size());
		copy(&src.front(), &src.front() + src.size(), dst.data());
	}
	/** Point cloud conversion */
	template <typename _PCLPoint, typename _Point> void convert(const std::vector<_Point>& src, pcl::PointCloud<_PCLPoint>& dst) {
		dst.resize(src.size());
		copy(src.data(), src.data() + src.size(), &dst.front());
	}

	/** Point cloud conversion */
	template <typename _PCLPoint, typename _Point> void convert(const pcl::PointCloud<_PCLPoint>& src, interop::Cloud<_Point>& dst) {
		// points
		convert(src, (typename std::vector<_Point>&)dst);
		// sensor frame
		dst.frame.p[0] = static_cast<float_t>(src.sensor_origin_.x());
		dst.frame.p[1] = static_cast<float_t>(src.sensor_origin_.y());
		dst.frame.p[2] = static_cast<float_t>(src.sensor_origin_.z());
		dst.frame.q.w = static_cast<float_t>(src.sensor_orientation_.w());
		dst.frame.q.x = static_cast<float_t>(src.sensor_orientation_.x());
		dst.frame.q.y = static_cast<float_t>(src.sensor_orientation_.y());
		dst.frame.q.z = static_cast<float_t>(src.sensor_orientation_.z());
		// dimensions
		dst.width = src.width;
		dst.height = src.height;
	}
	/** Point cloud conversion */
	template <typename _PCLPoint, typename _Point> void convert(const interop::Cloud<_Point>& src, pcl::PointCloud<_PCLPoint>& dst) {
		// points
		convert((const typename std::vector<_Point>&)src, dst);
		// sensor frame
		dst.sensor_origin_.x() = static_cast<float>(src.frame.p[0]);
		dst.sensor_origin_.y() = static_cast<float>(src.frame.p[1]);
		dst.sensor_origin_.z() = static_cast<float>(src.frame.p[2]);
		dst.sensor_orientation_.w() = static_cast<float>(src.frame.q.w);
		dst.sensor_orientation_.x() = static_cast<float>(src.frame.q.x);
		dst.sensor_orientation_.y() = static_cast<float>(src.frame.q.y);
		dst.sensor_orientation_.z() = static_cast<float>(src.frame.q.z);
		// dimensions (must be set after updating pcl::PointCloud)
		dst.width = src.width;
		dst.height = src.height;
		dst.is_dense = src.height < 2 && src.width > 1;
	}

	/** Import PCL point cloud from file */
	template <typename _PCLPoint, typename _Point> void loadPCLCloud(const std::string& path, Cloud<_Point>& cloud) {
		// disable PCL messages
		const pcl::console::VERBOSITY_LEVEL pclLevel = pcl::console::getVerbosityLevel();
		ScopeGuard guard([=]() { pcl::console::setVerbosityLevel(pclLevel); });
		pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
		// load point cloud
		pcl::PointCloud<_PCLPoint> points;
		if (pcl::PCDReader().read(path, points) != 0)
			throw std::runtime_error("golem::interop::loadPCLCloud(): unable to read from: " + path);
		// convert
		convert(points, cloud);
	}

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PCL_PCL_DEFS_H_