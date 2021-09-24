//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

CONFIG_FLOAT(init_loc_noise_, "loc_noise");
CONFIG_FLOAT(init_angle_noise_, "angle_noise");
CONFIG_FLOAT(k1, "motion_model.k1");
CONFIG_FLOAT(k2, "motion_model.k2");
CONFIG_FLOAT(k3, "motion_model.k3");
CONFIG_FLOAT(k4, "motion_model.k4");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

//Rishabh
void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  if(!odom_initialized_) return;
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
  }
}

// Joey
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  if(!odom_initialized_) return;
  vector<Vector2f> predicted_cloud;
  for (Particle& p: particles_) {
    // Compute our predicted point_cloud
    GetPredictedPointCloud(p.loc, p.angle, /* */, range_min, range_max, angle_min, angle_max, &predicted_cloud);
    for (int i = 0; i < ranges.size(); i++) {
      // Compute "difference" between the predicted point and real point
    }
    // Update particle weight accordingly
  }
}

// Joey
void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  vector<Particle> new_particles;

  // Create the buckets

  float x = rng_.UniformRandom(0, 1);
  
  // Determine which bucket the random float falls into

  // Start the specific random bucket and add an offset = sum(particle weights)/num_particles

  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if(!odom_initialized_) return;
  if(move_flag_) {
    // Update
    // Resample
  }
  move_flag_ = false;
}

// Adam
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  if(!odom_initialized_) return;


  for (Particle p: particles_) {
    // Update the position/angle of the particles based on Motion Model

    // determine the distance being traveled based on prev_odom_loc_ and prev_odom_angle_
    // UpdatePosition(Particle* p, Vector2f dloc, float di)
  }
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  prev_odom_loc_ = loc;
  prev_odom_angle_ = angle;
  particles_.clear();
  for(int i = 0; i < FLAGS_num_particles; i++) {
    particles_.push_back({
      // Draw samples for location noise
      loc + Eigen::Vector2f(rng_.Gaussian(0.0, CONFIG_init_loc_noise_), rng_.Gaussian(0.0, CONFIG_init_loc_noise_)),
      // Draw samples for angle noise
      angle + rng_.Gaussian(0.0, CONFIG_init_angle_noise_),
      1.0/num_particles,
    })
  }
  odom_initialized_ = true;
}

// Adam
void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  if(!odom_initialized_) return;
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
  for (Particle p: particles_) {
    // compute weighted sum of location and angle
  }
}


}  // namespace particle_filter
