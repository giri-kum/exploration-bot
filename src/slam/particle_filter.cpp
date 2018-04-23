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
    std::cout<<"particles initialized \n";
   initializeUniformPosteriorDistribution(pose); 
//   initializeGaussianPosteriorDistribution(pose); 
}

void ParticleFilter::initializeGaussianPosteriorDistribution(const pose_xyt_t& pose)
{
   default_random_engine generator_initial_particle_distribution; //must to seeded only once
   float stddev[] = {0.0,0.0,0.0}; // stddev in x,y,theta
   normal_distribution<float> x_distribution(0,stddev[0]), y_distribution(0,stddev[1]), theta_distribution(0,stddev[2]); // Consider uniform distribution for kidnapped robot case
   
   for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].parent_pose.x = pose.x + x_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.y = pose.y + y_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.theta = pose.theta + theta_distribution(generator_initial_particle_distribution);
        posterior_[i].pose = posterior_[i].parent_pose;
        posterior_[i].weight = (float) 1.0 / kNumParticles_; //u must use normalized weights when you pass it to lowvarianceresampler
    }
}

void ParticleFilter::initializeUniformPosteriorDistribution(const pose_xyt_t& pose)
{
   default_random_engine generator_initial_particle_distribution; //must to seeded only once
   float dev[] = {0.0,0.0,0.0};//{0.05,0.05,0.0}; // stddev in x,y,theta
   uniform_real_distribution<float> x_distribution(-dev[0],dev[0]), y_distribution(-dev[1],dev[1]), theta_distribution(-dev[2],dev[2]); // Consider uniform distribution for kidnapped robot case
    
/*
    OccupancyGrid map = OccupancyGrid(3,3,.1);
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> xdistr(-map.widthInMeters()/2, map.widthInMeters()/2);
    std::uniform_int_distribution<> ydistr(-map.heightInMeters()/2, map.heightInMeters()/2);
    std::uniform_int_distribution<> thetadistr(-1*M_PI, M_PI);
    Point<float> mapOrigin = map.originInGlobalFrame();

    for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].parent_pose.x = mapOrigin.x + xdistr(eng);
        posterior_[i].parent_pose.y = mapOrigin.y + ydistr(eng);
        posterior_[i].parent_pose.theta =thetadistr(eng);
        posterior_[i].pose = posterior_[i].parent_pose;
        posterior_[i].weight = (float) 1.0 / kNumParticles_;
    }

*/
   for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].parent_pose.x = pose.x + x_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.y = pose.y + y_distribution(generator_initial_particle_distribution);
        posterior_[i].parent_pose.theta = pose.theta + theta_distribution(generator_initial_particle_distribution);
        posterior_[i].pose = posterior_[i].parent_pose;
        posterior_[i].weight = (float) 1.0 / kNumParticles_;
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
        
        /*for (int i = 0; i < kNumParticles_; i++) 
        {
            posterior_[i] = actionModel_.applyAction(posterior_[i]);
            posterior_[i].weight = 1;//1 / kNumParticles_;
        }
        */
        auto prev_pose = posteriorPose_;
        auto prev_posterior_ = posterior_;
        auto prior = resamplePosteriorDistribution(); // resample before applying the action because you don't reset the weights        
        auto proposal = computeProposalDistribution(prior); //you apply action model onto the particles in this function
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // you update the weights using sensor model here (don't forget to normalize)
        posteriorPose_  = estimatePosteriorPose(posterior_); // you compute the pose using max or mean of particles locaiton here
        float change_in_theta = fabs(prev_pose.theta - posteriorPose_.theta);
        //std::cout << "change in theat: " << change_in_theta;
        if( change_in_theta > M_PI/4.0 && change_in_theta < 15.0*M_PI/8.0)
            {
                posterior_ = prev_posterior_;
                posteriorPose_ = prev_pose;
            }
        //std::cout << "Slam Pose: (" << posteriorPose_.x << ", " << posteriorPose_.y << ", " << posteriorPose_.theta << ")\n";
    }

    // DEBUG
//    posteriorPose_.x = 0;
//    posteriorPose_.y = 0;
//    posteriorPose_.theta = 0;
    
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
//        prior[j].weight = (float) 1/kNumParticles_;
        threshold = threshold + (float) (1.0/kNumParticles_);
    }
//    cout<<"From resamplePosteriorDistribution " <<prior[0].pose.x<<" " << posterior_[0].pose.x<<endl;
    
//    prior = posterior_; don't uncomment this unless u want to remove resampling
    return Normalize(prior);
}

bool compare_this(particle_t a,particle_t b)
{
    return a.weight < b.weight;
}

std::vector<particle_t> ParticleFilter::new_resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    int new_size = kNumParticles_/4; // Make sure this number can perfect divide kNumParticles_ (which is 200 by default) without any remainder
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
    for (int i = 0; i < kNumParticles_; i++) 
    {
        posterior[i] = proposal[i];
        posterior[i].weight = sensorModel_.likelihood(proposal[i], laser, map);
    }

    return Normalize(posterior);
}


 std::vector<particle_t> ParticleFilter::Normalize(const std::vector<particle_t>& given_particles)
 {
    std::vector<particle_t> result = given_particles;
    float sum = 0;
    for (unsigned int i = 0; i < result.size(); i++) 
        sum = sum + result[i].weight;
    for (unsigned int i = 0; i < result.size(); i++) 
        {
            if(sum == 0) result[i].weight = 0;
            else         result[i].weight = result[i].weight/sum;
        }
    return result;  
 }


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    float cart_x = 0, cart_y = 0;
    for(int i = 0; i< kNumParticles_; i++)
    {

        //std::cout << "x: " << posterior[i].pose.x << "  y: " << posterior[i].pose.y << "  weight: " << posterior[i].weight << std::endl;
        pose.x = pose.x + posterior[i].weight*posterior[i].pose.x;
        pose.y = pose.y + posterior[i].weight*posterior[i].pose.y;
        cart_x += posterior[i].weight*cos(posterior[i].pose.theta);
        cart_y += posterior[i].weight*sin(posterior[i].pose.theta);     
    }
    pose.theta = atan2(cart_y,cart_x); //pose.theta + posterior[i].weight*posterior[i].pose.theta;
//    pose.x = pose.x/kNumParticles_; // Debug, comment later
//    pose.y = pose.y/kNumParticles_; // Debug, comment later
//    pose.theta = pose.theta/kNumParticles_; // Debug, comment later
    pose.utime = posterior[kNumParticles_-1].pose.utime;    
    return pose;
}
