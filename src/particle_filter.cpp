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
	num_particles =100;

	std::default_random_engine gen;

	std::normal_distribution<double> N_x(x, std[0]);
	std::normal_distribution<double> N_y(y, std[1]);
	std::normal_distribution<double> N_theta(theta, std[2]);

	for (int i=0; i<num_particles; i++)
	{
		Particle particle;
		particle.id =i;
		particle.x = N_x(gen); 
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight =1;

		particles.push_back(particle);
		weights.push_back(1);
	}

	is_initialized =true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	for (int i=0; i<num_particles; i++)
	{
		double new_x;
		double new_y;
		double new_theta; 

		if (yaw_rate ==0)
		{
			new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
			new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else
		{
			new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t; 
		}

		normal_distribution<double> N_x(new_x, std_pos[0]);
		normal_distribution<double> N_y(new_y, std_pos[1]);
		normal_distribution<double> N_theta(new_theta, std_pos[2]);

		particles[i].x =N_x(gen);
		particles[i].y =N_y(gen);
		particles[i].theta =N_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// for (int i=0; i<observations.size(); i++)
	// {
	// 	double closest_landmark = 100000;
	// 	int closest_landmark_id =-1; 

	// 	for(int j=0; j<predicted.size(); j++)
	// 	{
	// 		double predict_dist = sqrt(pow(predicted[j].x-observations[i].x,2) + pow(predicted[j].y - observations[i].y, 2));

	// 		if(predict_dist <closest_landmark)
	// 		{
	// 			closest_landmark = predict_dist;
	// 			closest_landmark_id = predicted[j].id;
	// 		}
	// 	}

	// 	observations[i].id = closest_landmark_id;

	// }

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

	// process for each particle
	for (int p=0; p<num_particles; p++)
	{
		vector<int> associations; //landmark the particle associated with
		vector<double> sense_x; //used obversation array
		vector<double> sense_y;

		vector<LandmarkObs> trans_observations;
		LandmarkObs obs;

		//observation from particle coordinates to map transformation
		for (int i=0; i < observations.size(); i++)
		{
			LandmarkObs trans_obs;
			obs = observations[i];

			//perform the space transformation from vehicle to map 
			trans_obs.x = particles[p].x + (obs.x * cos(particles[p].theta) - obs.y * sin(particles[p].theta));
			trans_obs.y = particles[p].y + (obs.x * sin(particles[p].theta) + obs.y * cos(particles[p].theta));
			trans_observations.push_back(trans_obs);
		}

		particles[p].weight =1.0;

		//find closest landmark association for each observation in range 
		//then multiplied with Guassian probability from valid observation point to its closest landmark 
		for (int i=0; i<trans_observations.size(); i++)
		{
			//find closest landmark for trans_observations[i]
			double closest_dis = sensor_range;
			int association = -1;

			for (int j=0; j<map_landmarks.landmark_list.size(); j++)
			{
				double landmark_x = map_landmarks.landmark_list[j].x_f;
				double landmark_y = map_landmarks.landmark_list[j].y_f;

				double calc_dist = sqrt(pow(trans_observations[i].x-landmark_x,2) + pow(trans_observations[i].y-landmark_y,2));
				if(calc_dist <closest_dis)
				{
					closest_dis = calc_dist;
					association = j;//map_landmarks.landmark_list[j].id_i;
				}
			}
			//cout<<"association is "<<association<<endl;
			//compute Gaussian if closest landmark found 
			if(association!=-1)
			{
				double meas_x = trans_observations[i].x;
				double meas_y = trans_observations[i].y;
				double mu_x = map_landmarks.landmark_list[association].x_f;
				double mu_y = map_landmarks.landmark_list[association].y_f;
				long double multiplier =1 / (2*M_PI*std_landmark[0]*std_landmark[1]) * exp(-pow(meas_x-mu_x,2)/2/pow(std_landmark[0],2) - pow(meas_y-mu_y,2)/2/pow(std_landmark[1],2));
			
				// cout<<"meas_x, mu_x: "<< meas_x <<" "<<mu_x<<endl;
				// cout<<"meas_y, mu_y: "<< meas_y <<" "<<mu_y<<endl;

				// cout<<"multiplier: "<< multiplier<<endl;
				if(multiplier > 0)
				{
					particles[p].weight *= multiplier;
				}
				associations.push_back(map_landmarks.landmark_list[association].id_i);
				sense_x.push_back(trans_observations[i].x);
				sense_y.push_back(trans_observations[i].y);
			}
			particles[p] = SetAssociations(particles[p],associations,sense_x,sense_y);
			weights[p] = particles[p].weight;
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles; 

	for (int i=0; i<num_particles; i++)
	{ 
		resample_particles.push_back(particles[distribution(gen)]);
	}

	particles = resample_particles; 

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
