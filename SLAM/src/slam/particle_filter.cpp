#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    actionModel_.SetAction(pose);
    
    // Initialize particles
    std::default_random_engine generator;
    std::normal_distribution<float> distribution_x (pose.x, 0.01);
    std::normal_distribution<float> distribution_y (pose.y, 0.01);
    std::normal_distribution<float> distribution_theta (pose.theta, 0.0001);
    pose_xyt_t rand_pose;
    for(int i = 0; i < kNumParticles_; i++){
        rand_pose.x = distribution_x(generator);
        rand_pose.y = distribution_y(generator);
        rand_pose.theta = distribution_theta(generator);
        rand_pose.tb_angles[0] = 0.0f;
        rand_pose.tb_angles[1] = 0.0f;
        rand_pose.tb_angles[2] = 0.0f;
        posterior_[i].pose = rand_pose;
        posterior_[i].parent_pose = rand_pose;
        posterior_[i].weight = 1.0/kNumParticles_;
    }

    posteriorPose_ = pose;

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
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    posteriorPose_.tb_angles[0] = odometry.tb_angles[0];
    posteriorPose_.tb_angles[1] = odometry.tb_angles[1];
    posteriorPose_.tb_angles[2] = odometry.tb_angles[2];
    
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


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior;

    double r = (double)std::rand()/RAND_MAX/kNumParticles_;
    double c = posterior_[0].weight;
    int i = 0;

    for(int k = 1; k <= kNumParticles_; k++){
        double u = r + (double)(k-1)/kNumParticles_;
    	while(u > c){
    	    i += 1;
    	    c += posterior_[i].weight;
    	}
        prior.push_back(posterior_[i]);
    }

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(int i = 0; i < kNumParticles_; i++){
        proposal.push_back(actionModel_.applyAction(prior[i]));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    posterior.resize(kNumParticles_);
    double weight_sum = 0;
  
    // Compute the weight
    for(int i = 0; i < kNumParticles_; i++){
        posterior[i] = proposal[i];
        
    	posterior[i].weight = sensorModel_.likelihood(proposal[i], laser, map);
        weight_sum += posterior[i].weight;
    }
    
    // Normalize the weight
    for(int i = 0; i < kNumParticles_; i++){
        posterior[i].weight /= weight_sum;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose = posterior[0].pose;
    double max = posterior[0].weight;

    for(int i = 1; i < kNumParticles_; i++){
        if(posterior[i].weight > max){
            pose = posterior[i].pose;
            max = posterior[i].weight;
        }
    }

    return pose;
}
