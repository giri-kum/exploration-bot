#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <random>
#include <algorithm>

using namespace std;
ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{

    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
/*    for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].parent_pose = pose;
        posterior_[i].pose = pose;
        posterior_[i].weight = (float) 1 / kNumParticles_;
    }
    //std::cout<<"Particles initialized to pose : ("<<pose.x<<", "<<pose.y<<", "<<pose.theta<<", "<<pose.utime<<", "<<posterior_[kNumParticles_-1].weight<<") \n";
    */
   initializeUniformPosteriorDistribution(pose);
}

void ParticleFilter::initializeGaussianPosteriorDistribution(const pose_xyt_t& pose)
{
   default_random_engine generator_initial_particle_distribution; //must to seeded only once
   float stddev[] = {0.05,0.05,0.0}; // stddev in x,y,theta
   normal_distribution<float> x_distribution(0,stddev[0]), y_distribution(0,stddev[1]), theta_distribution(0,stddev[2]); // Consider uniform distribution for kidnapped robot case
   
   for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].parent_pose.x = pose.x + x_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.y = pose.y + y_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.theta = pose.theta + theta_distribution(generator_initial_particle_distribution);
        posterior_[i].pose = posterior_[i].parent_pose;
        posterior_[i].weight = 1; //(float) 1 / kNumParticles_;
    }
}

void ParticleFilter::initializeUniformPosteriorDistribution(const pose_xyt_t& pose)
{
   default_random_engine generator_initial_particle_distribution; //must to seeded only once
   float dev[] = {0.05,0.05,0.0}; // stddev in x,y,theta
   uniform_real_distribution<float> x_distribution(-dev[0],dev[0]), y_distribution(-dev[1],dev[1]), theta_distribution(-dev[2],dev[2]); // Consider uniform distribution for kidnapped robot case
    
   for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].parent_pose.x = pose.x + x_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.y = pose.y + y_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.theta = pose.theta + theta_distribution(generator_initial_particle_distribution);
        posterior_[i].pose = posterior_[i].parent_pose;
        posterior_[i].weight = 1; //(float) 1 / kNumParticles_;
    }
}



pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        /*
        for (int i = 0; i < kNumParticles_; i++) 
        {
            posterior_[i] = actionModel_.applyAction(posterior_[i]);
            posterior_[i].weight = 1; //1 / kNumParticles_;
        }
        */
        auto prior = new_resamplePosteriorDistribution(); // resample before applying the action because you don't reset the weights        
        auto proposal = computeProposalDistribution(prior); //you apply action model onto the particles in this function
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // you update the weights using sensor model here (don't forget to normalize)
        posteriorPose_ = estimatePosteriorPose(posterior_); // you compute the pose using max or mean of particles locaiton here
    }

    // DEBUG
    //posteriorPose_.x = 0;
    //posteriorPose_.y = 0;
    //posteriorPose_.theta = 0;
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

default_random_engine generator_resample; //must to seeded only once, so must be global.


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    std::vector<particle_t> prior(kNumParticles_);
    
    std::vector<float> cdf(kNumParticles_,0.0);
    uniform_real_distribution<double> distribution(0,1); // Uniform distribution from 0 to 1
    float threshold = distribution(generator_resample)/kNumParticles_;
    int index = 0;
    cdf[0] = posterior_[0].weight;
    for(int i = 1; i<kNumParticles_; i++)
        cdf[i] = cdf[i-1] + posterior_[i].weight;
                
    for(int j = 0; j<kNumParticles_; j++)
    {
        while(threshold > cdf[index])
            {
                index++;
            }
        prior[j] = posterior_[index];
        prior[j].weight = (float) 1/kNumParticles_;
        threshold = threshold + (float) (1/kNumParticles_);
    }
//    cout<<"From resamplePosteriorDistribution " <<prior[0].pose.x<<" " << posterior_[0].pose.x<<endl;
    
//    prior = posterior_; don't uncomment this unless u want to remove resampling
    return prior;
}

bool compare_this(particle_t a,particle_t b)
{
    return a.weight < b.weight;
}

std::vector<particle_t> ParticleFilter::new_resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    int new_size = kNumParticles_; // Make sure this number can perfect divide kNumParticles_ (which is 200 by default) without any remainder
    std::vector<particle_t> prior(kNumParticles_), temp(new_size);
    prior = posterior_;    
    std::sort(prior.begin(),prior.end(),compare_this);
    prior.resize(new_size);
    temp = prior;
    for(int i = 0 ;i < kNumParticles_/new_size; i++)
        prior.insert(prior.end(),temp.begin(),temp.end());
    
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal(kNumParticles_);
    for (int i = 0; i < kNumParticles_; i++) 
        {
            proposal[i] = actionModel_.applyAction(prior[i]);
        }
//    cout<<"From computeProposalDistribution " <<proposal[0].pose.x<<" " << prior[0].pose.x<<endl;
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior(kNumParticles_);
    float sum = 0;
    for (int i = 0; i < kNumParticles_; i++) 
    {
        posterior[i] = proposal[i];
        posterior[i].weight = sensorModel_.likelihood(proposal[i], laser, map);
        //std::cout << "i   "  << i << " "<<posterior[i].pose.x<< " "<<posterior[i].pose.y<<" "<<posterior[i].pose.theta<<" "<<posterior[i].pose.utime; 
        //std::cout << "   weight  " << posterior[i].weight << std::endl;
        //posterior[i].weight = (float) 1 / kNumParticles_; // Get weights from sensor model here
        sum = sum + posterior[i].weight;
    }
    for (int i = 0; i < kNumParticles_; i++) // Normalization
    {
        posterior[i].weight = posterior[i].weight/sum;
    }
//    cout<<"From computeNormalizedPosterior " <<posterior[0].pose.x<<" " << proposal[0].pose.x<<endl;
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
//    float cart_x = 0, cart_y = 0;
    for(int i = 0; i< kNumParticles_; i++)
    {
        pose.x = pose.x + posterior[i].weight*posterior[i].pose.x;
        pose.y = pose.y + posterior[i].weight*posterior[i].pose.y;
        //cart_x = cos(pose.theta) + posterior[i].weight*cos(posterior[i].pose.theta);
        //cart_y = sin(pose.theta) + posterior[i].weight*sin(posterior[i].pose.theta);
        pose.theta = pose.theta + posterior[i].weight*posterior[i].pose.theta;//atan2(cart_y,cart_x);
    }
//    pose.x = pose.x/kNumParticles_;
//    pose.y = pose.y/kNumParticles_;
//    pose.theta = pose.theta/kNumParticles_;
    pose.utime = posterior[kNumParticles_-1].pose.utime;    
    return pose;
}
