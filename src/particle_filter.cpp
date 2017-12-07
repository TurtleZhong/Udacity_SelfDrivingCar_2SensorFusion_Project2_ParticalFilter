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
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    /*step1: Set the number of Particles to 1000*/
    this->num_particles = 80;

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

    default_random_engine gen;
    // creates a normal (Gaussian) distribution for x, y and theta.
    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_theta(0, std_theta);

    for(std::vector<Particle>::iterator iter_p = particles.begin(); iter_p!=particles.end();iter_p++)
    {
        /*now iter_p is a point to each particle*/

        if(fabs(yaw_rate) > 0.0001)
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

        double sample_x     = dist_x(gen);
        double sample_y     = dist_y(gen);
        double sample_theta = dist_theta(gen);

        iter_p->x     += sample_x;
        iter_p->y     += sample_y;
        iter_p->theta += sample_theta;

    }


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.


    for(int i = 0; i < observations.size(); i++)
    {
        double min = 1000.0;
        int index = -1;

        for(int j = 0; j < predicted.size(); j++)
        {
            double distance = dist(predicted[j].x,predicted[j].y,observations[i].x, observations[i].y);
            if(distance < min)
            {
                min = distance;
                index = j;
            }
        }

        observations[i].id = predicted[index].id;
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

    /*for each particle*/
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];

    for(int i = 0; i < num_particles; i++)
    {
        /*step1: since the sensor_range is available, we can calc the lanmark in range*/
        std::vector<LandmarkObs> LandmarksInRange;
        double x     = particles[i].x;
        double y     = particles[i].y;
        double theta = particles[i].theta;

//        particles[i].weight = 1.0;

        /*check the map*/
        for(int j = 0; j < map_landmarks.landmark_list.size(); j++)
        {
            if (dist(x,y,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f) <= sensor_range)
            {
                LandmarksInRange.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,
                                                       map_landmarks.landmark_list[j].x_f,
                                                       map_landmarks.landmark_list[j].y_f});
            }
        }

        /*step2: Transform the observation which in Local coor(to each particle) to the Map coor*/

        std::vector<LandmarkObs> ObservationsInMap;
        ObservationsInMap.reserve(observations.size());
        for(int j = 0; j < observations.size(); j++)
        {
            double xo = observations[j].x;
            double yo = observations[j].y;
            double xm = cos(theta) * xo - sin(theta) * yo + x;
            double ym = sin(theta) * xo + cos(theta) * yo + y;

            ObservationsInMap.push_back(LandmarkObs{observations[j].id,xm,ym});
        }

        /*step3: after we transform, we need to associate the observation to the landmark*/

        dataAssociation(LandmarksInRange, ObservationsInMap);

        /*update the weights*/
        particles[i].weight = 1.0;

        for(int j = 0; j < ObservationsInMap.size(); j++)
        {
            /*we need find the landmark's pose*/
            int k = 0;
            double x_landmark;
            double y_landmark;

            while (k < LandmarksInRange.size())
            {
                /*find the match landmark*/
                if( LandmarksInRange[k].id == ObservationsInMap[j].id)
                {
                    x_landmark = LandmarksInRange[k].x;
                    y_landmark = LandmarksInRange[k].y;
                    break;
                }
                k++;

            }

            double x_observe = ObservationsInMap[j].x;
            double y_observe = ObservationsInMap[j].y;

            double x_diff = x_observe - x_landmark;
            double y_diff = y_observe - y_landmark;

            /*calc the weight*/
            double weight = CalcWeight(x_diff,y_diff,sigma_x,sigma_y);
            if (weight < 0.0001)
                weight = 0.0001;

            particles[i].weight *= weight;

        }


    }





}

double ParticleFilter::CalcWeight(const double &x_diff, const double &y_diff, const double &sigma_x, const double &sigma_y)
{
    double weight = exp(-1.0*(x_diff*x_diff / (2.0* sigma_x * sigma_x) + y_diff*y_diff / (2.0* sigma_y * sigma_y))) / (2.0 * M_PI * sigma_x * sigma_y);
    return weight;
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    /*step1: featch all the particle weights to a vector*/
    vector<double> vWeights;
    std::vector<Particle> particles_origin = particles;
    for(int i =0; i < num_particles; i++)
    {
        vWeights.push_back(particles[i].weight);
    }

    discrete_distribution<> d(vWeights.begin(),vWeights.end());
    default_random_engine gen;

    map<int, int> m;
    for(int n=0; n<num_particles; ++n)
    {
        ++m[d(gen)];
    }
    particles.clear();
    particles.reserve(num_particles);
    for(auto p : m)
    {
        //std::cout << p.first << " generated " << p.second << " times\n";
        for(int count = 0; count < p.second; count++)
        {
            particles.push_back(particles_origin[p.first]);
        }

    }

    /*now we get the resample weights*/


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
