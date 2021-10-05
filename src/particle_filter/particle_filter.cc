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
    // printf("P0: %f, %f P1: %f,%f\n", 
    //        my_line.p0.x(),
    //        my_line.p0.y(),
    //        my_line.p1.x(),
    //        my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      // printf("Intersects at %f,%f\n", 
      //        intersection_point.x(),
      //        intersection_point.y());
    } else {
      // printf("No intersection\n");
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
    (void)p; // Temp to compile
    // Compute our predicted point_cloud
    // GetPredictedPointCloud(p.loc, p.angle, /* */, range_min, range_max, angle_min, angle_max, &predicted_cloud);
    for (size_t i = 0; i < ranges.size(); i++) {
      // Compute "difference" between the predicted point and real point
      (void)i; // Temp to compile
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
  (void)x; // Temp to compile
  
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
  // if(move_flag_) {
  //   // Update
  //   // Resample
  // }
  // move_flag_ = false;
}

// Adam
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  if(!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  // Calculate the delta position and angles from this odometery message
  Vector2f delta_pos = Eigen::Rotation2Df(-1*prev_odom_angle_) * (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleMod(odom_angle - prev_odom_angle_);

  UpdateParticlesNaive(delta_pos, delta_angle);

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  float uniform_weight = 1.0/FLAGS_num_particles;
  particles_.clear();
  for(int i = 0; i < FLAGS_num_particles; i++) {
    Particle particle;
    particle.loc = loc + Eigen::Vector2f(rng_.Gaussian(0.0, CONFIG_init_loc_noise_), rng_.Gaussian(0.0, CONFIG_init_loc_noise_));
    particle.angle = angle + rng_.Gaussian(0.0, CONFIG_init_angle_noise_);
    particle.weight = uniform_weight;
    particles_.push_back(particle);
  }
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
  Vector2f angle_vector(0, 0);
  float sum_of_weights = 0.0;
  for (const Particle particle: particles_) {
    // compute weighted sum of location and angle
    loc += particle.weight * particle.loc;
    sum_of_weights += particle.weight;

    // Handle the angle by creating a unit vector in the direction
    angle_vector += particle.weight * Vector2f(cos(particle.angle), sin(particle.angle));
  }

  // Normalize
  loc.x() /= sum_of_weights;
  loc.y() /= sum_of_weights;
  angle_vector.x() /= sum_of_weights;
  angle_vector.y() /= sum_of_weights;
  
  // Convert unit vector back to angle
  angle = math_util::AngleMod(atan2(angle_vector.y(), angle_vector.x()));
}

void ParticleFilter::UpdateParticlesNaive(Vector2f& delta_pos, float delta_angle){
  // Calculate the variances of the normal distributions
  float linear_variance = CONFIG_k1*delta_pos.norm() + CONFIG_k2 * fabs(delta_angle);
  float angle_variance = CONFIG_k3*delta_pos.norm() + CONFIG_k4 * fabs(delta_angle);

  for (Particle& particle: particles_) {
    // Get the delta position vector in the map frame
    Vector2f map_delta = Eigen::Rotation2Df(particle.angle) * delta_pos;

    // Update the position/angle of the particles based on Motion Model
    particle.angle += delta_angle + rng_.Gaussian(0.0, angle_variance);
    particle.loc.x() += map_delta.x() + rng_.Gaussian(0.0, linear_variance);
    particle.loc.y() += map_delta.y() + rng_.Gaussian(0.0, linear_variance);
  }
}


}  // namespace particle_filter
