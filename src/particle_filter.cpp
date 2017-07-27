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

    num_particles = 30;

    normal_distribution<double> dist_x(0, std[0]);
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);
    default_random_engine gen;

    this->particles = vector<Particle>(num_particles);
    for (int i=0; i<num_particles; ++i){
        this->particles[i] = Particle{i, x+dist_x(gen), y+dist_y(gen), theta+dist_theta(gen), 1.0};
    }

    this->weights = vector<double>(num_particles, 1.0);

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;

    double x_f, y_f, theta_f;
    Particle p;
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    for (int i=0; i<num_particles; ++i){
        p = this->particles[i];

        if (std::abs(yaw_rate) > 0.0001){
            x_f = p.x + (velocity/yaw_rate)*(sin(p.theta + yaw_rate*delta_t)-sin(p.theta));
            y_f = p.y + (velocity/yaw_rate)*(cos(p.theta) - cos(yaw_rate*delta_t + p.theta));
            theta_f = p.theta + yaw_rate*delta_t;
        }
        else{
            x_f = p.x + velocity*cos(p.theta)*delta_t;
            y_f = p.y + velocity*sin(p.theta)*delta_t;
            theta_f = p.theta;
        }

        this->particles[i] = Particle{i, x_f+dist_x(gen), y_f+dist_y(gen), theta_f+dist_theta(gen), p.weight};
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    for (LandmarkObs & o_lm : observations){
        double min_dist = HUGE_VAL;

        for(LandmarkObs & p_lm : predicted){
            double distance = dist(p_lm.x, p_lm.y, o_lm.x, o_lm.y);
            if (distance < min_dist){
                min_dist = distance;
                o_lm.id = p_lm.id;
            }

        }
    }

}

LandmarkObs vehicle2MapPoint(LandmarkObs &vehicleObs, Particle &p){
    double map_x = vehicleObs.x * cos(p.theta) - vehicleObs.y * sin(p.theta) + p.x;
    double map_y = vehicleObs.x * sin(p.theta) + vehicleObs.y * cos(p.theta) + p.y;

    return LandmarkObs{p.id, map_x, map_y};

}

double calcNewWeight(const LandmarkObs &mesurement, const LandmarkObs &prediction, double * std_landmark){

    double x = pow((mesurement.x - prediction.x),2)/(std_landmark[0]*std_landmark[0]);
    double y = pow((mesurement.y - prediction.y),2)/(std_landmark[1]*std_landmark[1]);
    double w = exp(-0.5*(x+y))/(2*M_PI*std_landmark[0] * std_landmark[1]);

    return w;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks) {
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

    weights.clear();

    std::vector<LandmarkObs> wholeMapLandmarks;
    for( Map::single_landmark_s & globalLandmark : map_landmarks.landmark_list){
        LandmarkObs lm = LandmarkObs{globalLandmark.id_i, globalLandmark.x_f, globalLandmark.y_f};
        wholeMapLandmarks.push_back(lm);
    }

    for (Particle &p : this->particles){
        std::vector<LandmarkObs> observationsOnMap;
        observationsOnMap.clear();
        std::vector<LandmarkObs> candidateLandmarks;
        candidateLandmarks.clear();


        for (LandmarkObs obs : observations){
            LandmarkObs mapObservation = vehicle2MapPoint(obs, p);
            observationsOnMap.push_back(mapObservation);
        }


        for( Map::single_landmark_s & globalLandmark : map_landmarks.landmark_list){
            LandmarkObs lm = LandmarkObs{globalLandmark.id_i, globalLandmark.x_f, globalLandmark.y_f};
            if (dist(p.x, p.y, globalLandmark.x_f, globalLandmark.y_f) < sensor_range){
                candidateLandmarks.push_back(lm);
            }
        }

        this->dataAssociation(candidateLandmarks, observationsOnMap);

        //weight calculation

        p.weight = 1;
        for (LandmarkObs &obs : observationsOnMap){
            for (LandmarkObs &pred : candidateLandmarks){
                if(obs.id == pred.id){
                    p.weight *= calcNewWeight(obs, pred, std_landmark);
                }
            }
        }

        p.weight = max(p.weight, 0.0001);
        weights.push_back(p.weight);
    }
}


void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());

    vector<Particle> newParticles(num_particles);
    for (int i = 0; i < num_particles; ++i ){
        newParticles[i] = particles[d(gen)];
    }
    particles = newParticles;
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
