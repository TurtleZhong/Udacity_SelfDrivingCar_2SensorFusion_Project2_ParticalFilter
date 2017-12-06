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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    /*step1: Set the number of Particles to 1000*/
    this->num_particles = 1000;

    /*step2: Initialize the Partical filter's position and add random Gaussian noise to each*/
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    default_random_engine gen;
    // creates a normal (Gaussian) distribution for x.
    normal_distribution<double> dist_x(x, std_x);

    //Create normal distributions for y and theta.
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    this->particles.reserve(num_particles);
    for(int i = 0; i < num_particles; i++)
    {
        double sample_x     = dist_x(gen);
        double sample_y     = dist_y(gen);
        double sample_theta = dist_theta(gen);

        Particle particle;
        particle.id = i;
        particle.x      = sample_x;
        particle.y      = sample_y;
        particle.theta  = sample_theta;
        particle.weight = 1.0;
        particles.push_back(particle);
    }

    /*step3: The partical filter is finished so we need to set the is_initialized to true*/
    this->is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    /*predict each particle's next position and add random Gaussian noise*/

    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    for(std::vector<Particle>::iterator iter_p = particles.begin(); iter_p!=particles.end();iter_p++)
    {
        /*now iter_p is a point to each particle*/

        if(yaw_rate > 0.001)
        {
            iter_p->x     = iter_p->x + velocity * (sin(iter_p->theta + yaw_rate * delta_t) - sin(iter_p->theta)) / yaw_rate;
            iter_p->y     = iter_p->y + velocity * (cos(iter_p->theta) - cos(iter_p->theta + yaw_rate * delta_t)) / yaw_rate;
            iter_p->theta = iter_p->theta + yaw_rate * delta_t;
        }
        else
        {
            iter_p->x     = iter_p->x + velocity * delta_t * cos(iter_p->theta);
            iter_p->y     = iter_p->y + velocity * delta_t * sin(iter_p->theta);
            iter_p->theta = iter_p->theta + yaw_rate * delta_t;

        }

        /*up to now, we have predict the position by the motion model*/
        /*it seems like we have to add noise*/

        default_random_engine gen;
        // creates a normal (Gaussian) distribution for x, y and theta.
        normal_distribution<double> dist_x(iter_p->x, std_x);
        normal_distribution<double> dist_y(iter_p->y, std_y);
        normal_distribution<double> dist_theta(iter_p->theta, std_theta);

        double sample_x     = dist_x(gen);
        double sample_y     = dist_y(gen);
        double sample_theta = dist_theta(gen);

        iter_p->x     = sample_x;
        iter_p->y     = sample_y;
        iter_p->theta = sample_theta;

    }


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    /*note that we should calc for each particle and set to particle.
     * 	std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    */

    for(int i = 0; i < predicted.size(); i++)
    {
        /*find the closest from the observations*/
        LandmarkObs predict = predicted[i];
        LandmarkObs closest_observe;
        double min = 20.0;
        int index = 0;

        for(int j = 0; j < observations.size(); j++)
        {
            double distance = dist(predict.x,predict.y,observations[j].x, observations[j].y);
            if(distance < min)
            {
                index = j;
            }
        }
        closest_observe = observations[index];
        predicted[i] = closest_observe;

    }




}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
