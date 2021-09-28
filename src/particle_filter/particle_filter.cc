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

#define LIDAR_RANGE 30.0
const Vector2f kLaserLoc(0.2, 0);

DEFINE_double(num_particles, 50, "Number of particles");
DEFINE_bool(use_naive_motion_model, false, "Use naive (differential) motion model");

namespace particle_filter {

CONFIG_FLOAT(init_loc_noise_, "loc_noise");
CONFIG_FLOAT(init_angle_noise_, "angle_noise");
CONFIG_FLOAT(naive_k1, "naive_motion_model.k1");
CONFIG_FLOAT(naive_k2, "naive_motion_model.k2");
CONFIG_FLOAT(naive_k3, "naive_motion_model.k3");
CONFIG_FLOAT(naive_k4, "naive_motion_model.k4");
CONFIG_FLOAT(motion_k1, "motion_model.k1");
CONFIG_FLOAT(motion_k2, "motion_model.k2");
CONFIG_FLOAT(gamma, "gamma");
CONFIG_FLOAT(lidar_stddev, "lidar_stddev");
CONFIG_FLOAT(distance_observe_threshold, "distance_observe_threshold");
CONFIG_FLOAT(angle_observe_threshold, "angle_observe_threshold");
CONFIG_FLOAT(min_update_before_resample_count, "min_update_before_resample_count");
CONFIG_INT(rays, "rays");

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    n_updates(0),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

//Rishabh
void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            unsigned int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            float angle_increment,
                                            vector<Vector2f>* scan_ptr) {
  if(!odom_initialized_) return;
  vector<Vector2f>& scan = *scan_ptr;
  unsigned int max_ranges = (unsigned int) std::ceil((angle_max-angle_min)/angle_increment) + 1;
  unsigned int range_delta = max_ranges/num_ranges;
  scan.resize(max_ranges);
  for (size_t i = 0; i < max_ranges; i+=range_delta) {
    float dangle = std::min(angle_max, angle_min + i * angle_increment);
    const Vector2f lidar_base =  Eigen::Rotation2Df(angle)*kLaserLoc + loc;
    float min_distance = range_max;
        
    // Line segment from location to LIDAR Range.
    scan[i] = Vector2f(lidar_base.x()+min_distance*std::cos(angle+dangle), lidar_base.y()+min_distance*std::sin(angle+dangle)); // Set default ray to be maximum range of laser
    const line2f my_line(lidar_base.x(), lidar_base.y(), lidar_base.x()+min_distance*std::cos(angle+dangle), lidar_base.y()+min_distance*std::sin(angle+dangle)); 
    // Iterate through map lines
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];
      // Check for intersections
      Vector2f intersection_point; // Return variable
      bool intersects = map_line.Intersection(my_line, &intersection_point);
      if (intersects && (lidar_base-intersection_point).norm() < min_distance) {
        // printf("Intersects at %f,%f\n", 
        //       intersection_point.x(),
        //       intersection_point.y());
        scan[i] = intersection_point;
        min_distance =  (lidar_base-intersection_point).norm();
      } 
    }
    // printf("Min distance %f\n", min_distance);
  }
}

// Joey
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            float angle_increment,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  if(!odom_initialized_) return;
  vector<Vector2f> predicted_cloud;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, CONFIG_rays, range_min, range_max, angle_min, angle_max, angle_increment, &predicted_cloud);
  unsigned int range_delta = ranges.size()/CONFIG_rays;
  Eigen::Vector2f lidar_loc = Eigen::Rotation2Df(p_ptr->angle)*kLaserLoc + p_ptr->loc;
  for(unsigned i = 0; i < ranges.size(); i+=range_delta)
  {
    // printf("LIDAR Loc (%f, %f) particle loc (%f, %f) intersection point (%f, %f)", lidar_loc.x(), lidar_loc.y(), p_ptr->loc.x(), p_ptr->loc.y(), predicted_cloud[i].x(), predicted_cloud[i].y());
    float pred_range = (predicted_cloud[i] - lidar_loc).norm();
    float observed_range = ranges[i];
    // printf("Difference between %d observed(%f) and predicted(%f) range %f\n", i, observed_range, pred_range, observed_range-pred_range);
    p_ptr->weight += CONFIG_gamma * (-0.5 * ((observed_range-pred_range)*(observed_range-pred_range))/(CONFIG_lidar_stddev*CONFIG_lidar_stddev));  //log likelihood
  }
  // printf("Final weight %f\n", p_ptr->weight);
}

// Joey
void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  vector<Particle> new_particles;

  // Create the buckets
  // Normalize weights and create buckets
  float sum_particle_weights = 0;
  vector<float> buckets;
  float max_log_weight = particles_[0].weight;
  for(unsigned int i = 1; i < particles_.size(); ++i)
  {
    if (particles_[i].weight > max_log_weight) {
      max_log_weight = particles_[i].weight;
    }
  }
  for(Particle p : particles_)
  {    
    // printf("particle weight %f\n", p.weight - max_log_weight);
    p.weight = expf(p.weight - max_log_weight);
    sum_particle_weights += p.weight;
    buckets.push_back(sum_particle_weights);
  }
  // printf("sum particle weights %f\n", sum_particle_weights);
  // Determine which bucket the random float falls into
  // Start the specific random bucket and add an offset = sum(particle weights)/num_particles
  float x = rng_.UniformRandom(0, 1);
  float increment = sum_particle_weights/particles_.size();
  unsigned particle_index = static_cast<int>(x * (particles_.size()-1));
  float curr_weight = buckets[particle_index];
  // printf("Initial weight %f, idx %d increment %f\n", curr_weight, particle_index, increment);
  while(new_particles.size() < particles_.size())
  {
    if(curr_weight > buckets[particle_index])
    {
      ++particle_index;
      if(particle_index >= particles_.size())
      {
        particle_index%=particles_.size(); //don't index OOB
        curr_weight = curr_weight - sum_particle_weights;
      }
      continue;
    }
    curr_weight += increment;
    new_particles.push_back(particles_[particle_index]);
    // printf("Added weight %f, idx %d increment %f\n", curr_weight, particle_index, increment);
  }
  // float x = rng_.UniformRandom(0.0, sum_particle_weights);
  // unsigned int idx = 0;
  // while (x > buckets[idx]) ++idx;

  // // Start the specific random bucket and add an offset = sum(particle weights)/num_particles
  // float increment = sum_particle_weights/FLAGS_num_particles;
  // while(new_particles.size() < particles_.size())
  // {
  //   new_particles.push_back(particles_[idx]);
  //   x += increment;
  //   if (x > sum_particle_weights) {
  //     idx = 0;
  //     x = x-sum_particle_weights;
  //   }
  //   while (x > buckets[idx]) ++idx;
  // }


  for(Particle& p : new_particles)
  {
    p.weight = 1;
  }
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max,
                                  float angle_increment) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if(!odom_initialized_) return;
  if(distance_traveled > CONFIG_distance_observe_threshold || angle_traveled > CONFIG_angle_observe_threshold)
  {
    for(Particle& p : particles_)
    {
        Update(ranges, range_min, range_max, angle_min, angle_max, angle_increment, &p);
        n_updates += 1;
    }
    if (n_updates > CONFIG_min_update_before_resample_count)
    {
      Resample();
      n_updates = 0;
    }
    distance_traveled = 0;
    angle_traveled = 0;
  }
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

  distance_traveled += delta_pos.norm();
  angle_traveled += abs(delta_angle); //NOTE: potentially change this later to just be the raw value of delta angle

  if (FLAGS_use_naive_motion_model) {
    UpdateParticlesNaive(delta_pos, delta_angle);
  } else {
    UpdateParticles(delta_pos, delta_angle);
  }

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

// Rishabh
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
    // printf("init particl weight %f\n", particle.weight);
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
  float linear_variance = CONFIG_naive_k1*delta_pos.norm() + CONFIG_naive_k2 * fabs(delta_angle);
  float angle_variance = CONFIG_naive_k3*delta_pos.norm() + CONFIG_naive_k4 * fabs(delta_angle);

  for (Particle& particle: particles_) {
    // Get the delta position vector in the map frame
    Vector2f map_delta = Eigen::Rotation2Df(particle.angle) * delta_pos;

    // Update the position/angle of the particles based on Motion Model
    particle.angle += delta_angle + rng_.Gaussian(0.0, angle_variance);
    particle.loc.x() += map_delta.x() + rng_.Gaussian(0.0, linear_variance);
    particle.loc.y() += map_delta.y() + rng_.Gaussian(0.0, linear_variance);
  }
}

void ParticleFilter::UpdateParticles(Eigen::Vector2f& delta_pos, float delta_angle) {
  // Check for linear (c = 0) case
  if (fabs(delta_angle) < 1e-5) {
    UpdateParticlesNaive(delta_pos, delta_angle);
    return;
  }

  // Find the arc angle and radius
  float arc_angle = delta_angle;
  float radius = sqrt((pow(delta_pos.norm(),2))/(2 - 2*cos(fabs(arc_angle))));

  // Get the variances for each direction
  float tangential_variance = CONFIG_motion_k1 * fabs(arc_angle*radius);
  float radial_variance = CONFIG_motion_k2 * fabs(1/radius);

  // TODO think about how to handle this angle variance
  float angle_variance = CONFIG_naive_k3*delta_pos.norm() + CONFIG_naive_k4 * fabs(delta_angle);

  for (Particle& particle: particles_) {
    // Update the position/angle of the particles based on Motion Model
    Vector2f noise(rng_.Gaussian(0.0, tangential_variance), rng_.Gaussian(0.0, radial_variance));
    particle.loc += Eigen::Rotation2Df(particle.angle)*(delta_pos + Eigen::Rotation2Df(arc_angle).inverse()*noise);

    particle.angle += delta_angle + rng_.Gaussian(0.0, angle_variance);
  }
}


}  // namespace particle_filter
