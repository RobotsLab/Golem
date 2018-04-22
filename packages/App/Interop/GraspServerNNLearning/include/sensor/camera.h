/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Header file for camera class that keeps the camera parameters and type of noise.
 * Most important functionality is the conversion of pixel coordinates to rays and 
 * the projection of rays onto the image plane. These functions are taken from the 
 * ros-package image_geometry.
 */

#ifndef KINECT_SIM_CAMERA_H_
#define KINECT_SIM_CAMERA_H_

#include <opencv2/opencv.hpp>
#include <iostream>

namespace Grasp
{
  enum NoiseType
  {
    GAUSSIAN=1,
    PERLIN,
    SIMPLEX,
    NONE
  };

  class CameraInfo
  {
  public:
	// Initialized with Carmine 1.09 parameters.
    int width = 640, height = 480;
    double z_near = 0.20, z_far = 1.4;
    double fx = 574.0, fy = 574.0; // 574 both For a vertical focal length close to 45.
    double cx_ = 320, cy_ = 240;
    double tx_ = 0.075; // -0.075 normally.
    double colourtx_ = 0.027; // -0.048 normally.
    NoiseType noise_ = GAUSSIAN;

  };

  class Camera
  {
  public:
    
    double getFx()
    {
      return info_.fx;
    }

    double getFy()
    {
      return info_.fy;
    }

    double getCx()
    {
      return info_.cx_;
    }

    double getCy()
    {
      return info_.cy_;
    }

    int getWidth()
    {
      return info_.width;
    }

    int getHeight()
    {
      return info_.height;
    }

    double getZNear()
    {
      return info_.z_near;
    }

    double getZFar()
    {
      return info_.z_far;
    }

    double getTx() 
    {
      return info_.tx_;
    }
    
    double getColourTx()
    {
      return info_.colourtx_;
    }

  Camera(const CameraInfo p_info)
    : info_(p_info) {}

      CameraInfo info_;
  };
 
} // namespace render_kinect




#endif // KINECT_SIM_CAMERA_H_
