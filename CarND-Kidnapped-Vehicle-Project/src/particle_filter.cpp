/**
 * particle_filter.cpp
 *
 * 
 * Author: Santosh Balajee Banisetty
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

//using std::string;
//using std::vector;
using namespace std;
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  weights.resize(num_particles);
  
  //resize particles
  particles.resize(num_particles);
  default_random_engine gen;
  
  //normal distribution x, y, theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  //init
  for (int idx = 0; idx < num_particles; ++idx)
  {
    particles[idx].id = idx;
    particles[idx].x = dist_x(gen);
    particles[idx].y = dist_y(gen);
    particles[idx].theta = dist_theta(gen);
    particles[idx].weight = 1.0;
  }

  //set init flag
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
    default_random_engine rand_eng;
  // Creating normal distributions
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  // Calculate new state
  for (int i = 0; i < num_particles; ++i) 
  {

  	double theta = particles[i].theta;

    if ( fabs(yaw_rate) < 0.000001 ) 
    { // When yaw is not changing
      particles[i].x += velocity * delta_t * cos( theta );
      particles[i].y += velocity * delta_t * sin( theta );
      
    } else // if yaw continues to be the same
    {
      particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
      particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
      particles[i].theta += yaw_rate * delta_t;
    }

    particles[i].x += dist_x(rand_eng);
    particles[i].y += dist_y(rand_eng);
    particles[i].theta += dist_theta(rand_eng);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  
  for (unsigned int i = 0; i < observations.size(); i++)
  {
    double min_dist = numeric_limits<double>::max();
    double current_dist;
    int map_id = -1;
    LandmarkObs obs = observations[i];
    for (unsigned int j = 0; j < predicted.size(); j++)
    {
      LandmarkObs pred = predicted[j];
      current_dist = dist(obs.x, obs.y, pred.x, pred.y);
      if(current_dist < min_dist) // update min
      {
        min_dist = current_dist;
        map_id = predicted[j].id;
      }
    }
    observations[i].id = map_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for (int i = 0; i < num_particles; i++)
  {
    // extract each particle
    double temp_x = particles[i].x;
    double temp_y = particles[i].y;
    double temp_theta = particles[i].theta;
    
    //Landmarks in range and on the map coordinate system
    vector<LandmarkObs> landmarkInRange;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;
      // if in range
      double distance = dist(temp_x, temp_y, lm_x, lm_y);
      if (distance <= sensor_range)
        landmarkInRange.push_back(LandmarkObs{lm_id, lm_x, lm_y});
    }
    // transformed into map aka world coordinates
    vector<LandmarkObs> tfMap;
    for (unsigned int k = 0; k < observations.size(); k++) 
    {
      double t_x = cos(temp_theta)*observations[k].x - sin(temp_theta)*observations[k].y + temp_x;
      double t_y = sin(temp_theta)*observations[k].x + cos(temp_theta)*observations[k].y + temp_y;
      int t_id = observations[k].id;
      tfMap.push_back(LandmarkObs{ t_id, t_x, t_y });
    }
    //call data association to associate each particle
    dataAssociation(landmarkInRange, tfMap);
    particles[i].weight=1.0;//initialize weight again
    
    for (unsigned int k = 0; k < tfMap.size(); k++) 
    {      
      // temps
      double pr_x, pr_y;
      double o_x = tfMap[k].x;
      double o_y = tfMap[k].y;

      int associated_id = tfMap[k].id;

      // get the x,y coordinates of the prediction associated with the current observation
      for (unsigned int l = 0; l < landmarkInRange.size(); l++) 
      {
        if (landmarkInRange[l].id == associated_id) 
        {
          pr_x = landmarkInRange[l].x;
          pr_y = landmarkInRange[l].y;
        }
      }
      //update factor for new weight
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];
      
      double deltaX = -(o_x - pr_x);
      double deltaY = -(o_y - pr_y);
      
      double std_weight = ( 1/(2*M_PI*std_x*std_y)) * exp( -( deltaX*deltaX/(2*std_x*std_x) + (deltaY*deltaY/(2*std_y*std_y)) ) );
      if (std_weight == 0.0)
      {
        particles[i].weight = 0.000001;
      } else
      {
      particles[i].weight *= std_weight;
      }
    }
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
 double beta = 0.0;
 vector<double> weights;
 default_random_engine gen;
 // creating resampling particles
 vector<Particle> resampledParticles;
 
 double maxWeight = numeric_limits<double>::min();//init max weight
  
 for(int i = 0; i < num_particles; ++i) 
 {
   weights.push_back(particles[i].weight);
   // cout << weights[i] << endl;
   if ( particles[i].weight > maxWeight ) //finding max
     maxWeight = particles[i].weight;
 }
  
 // Creating distributions
 uniform_real_distribution<double> distDouble(0.0, maxWeight);
 uniform_int_distribution<int> distInt(0, num_particles - 1);

 // Generating index
 int index = distInt(gen);

 for(int i = 0; i < num_particles; ++i) 
 {
   beta += distDouble(gen) * 2.0;
    
   while( beta > weights[index]) 
   {
     beta -= weights[index];
     index = (index + 1) % num_particles;
   }
   resampledParticles.push_back(particles[index]);
 }

 particles = resampledParticles; //update the particles
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}