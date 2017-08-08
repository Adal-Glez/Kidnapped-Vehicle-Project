/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    
    
    default_random_engine gen;
    double std_x, std_y, std_theta; // Standard deviations for x, y, and psi
    
    // TODO: Set standard deviations for x, y, and psi.
    std_x  =std[0]; //2;
    std_y  =std[1];//2;
    std_theta=std[2]; //std_psi=std[2]//0.05;
    
    
    // This line creates a normal (Gaussian) distribution for x  y and psi..
    normal_distribution<double> dist_x(x, std_x);//(gps_x, std_x);
    normal_distribution<double> dist_y(y, std_y);//(gps_y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);
    
    //Set the number of particles
    num_particles = 100;
    
    for (int i = 0; i < num_particles; ++i) {
        //double sample_x, sample_y, sample_psi;
        
        Particle p;
        p.id    = i;
        p.x     = dist_x(gen);
        p.y     = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight= 1.0;
        particles.push_back(p);
        weights.push_back(1.0);
        
    }
    
    is_initialized = true;
    
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // Add measurements to each particle and add random Gaussian noise.
    
    default_random_engine gen;
    
    //add random gausian noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    
    for (int i=0; i< num_particles; ++i) {
        Particle p = particles[i];
        //double theta = p.theta;
        
        if( fabs(yaw_rate) > 0.001){
            p.x     = p.x +  (velocity / yaw_rate) * ( sin(p.theta + yaw_rate * delta_t) - sin(p.theta)) + dist_x(gen);
            p.y     = p.y +  (velocity / yaw_rate) * (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta)) + dist_y(gen);
            //p.theta = p.theta + delta_t * yaw_rate;
        }else{
            p.x     = p.x + velocity * delta_t * cos(p.theta) + dist_x(gen);
            p.y     = p.y + velocity * delta_t * sin(p.theta) + dist_y(gen);
        }
        p.theta += yaw_rate * delta_t + dist_theta(gen);
        particles[i] = p;
    }
    
    
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    
    for (int i=0 ; i<observations.size(); ++i) {
        double closest_dist, new_x, new_y;
        
        for(int j=0;j<predicted.size(); ++j){
            double distance = dist(predicted[j].x,predicted[j].y, observations[i].x,observations[i].y);
            if (distance < closest_dist) {
                new_x        = predicted[j].x;
                new_y        = predicted[j].y;
                closest_dist = distance;
            }
        }
        
        observations[i].x= new_x;
        observations[i].y= new_y;
    }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks) {
    // Update the weights of each particle using a mult-variate Gaussian distribution.
    
    // predict landmarks within sensor_range
    
    // transformation based on p_x, p_y, p_theta and observed landmark positions.
    
    // Association
    
    // extract corresponding prediction // update weight
    
   
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    
    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
