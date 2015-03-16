#include "moveit_compute_default_collisions.h"

MoveitComputeDefaultCollisions::MoveitComputeDefaultCollisions(const std::string &urdf_path,
                                                               const std::string &srdf_path)
{
    config_data_.reset(new moveit_setup_assistant::MoveItConfigData());
    config_data_->urdf_path_ = urdf_path;
    config_data_->srdf_path_ = srdf_path;

    if (!config_data_->urdf_model_->initFile(urdf_path))
    {
        std::cout<<"Failed to parse urdf robot model"<<std::endl;
        throw new std::runtime_error("Failed to parse urdf robot model");
    }
    else
    {
        config_data_->srdf_->srdf_model_.reset(new srdf::Model());
        if(!config_data_->srdf_->srdf_model_->initFile(*config_data_->urdf_model_,
                                                       srdf_path))
        {
            std::cout<<"Failed to parse SRDF robot model!"<<std::endl;
            throw new std::runtime_error("Failed to parse SRDF robot model!");

        }
        else
        {
            config_data_->srdf_->initModel(*(config_data_->urdf_model_),
                                           *(config_data_->srdf_->srdf_model_));
        }
    }
}


void MoveitComputeDefaultCollisions::print()
{
    for ( moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs_.begin();
          pair_it != link_pairs_.end();
          ++pair_it)
    {
        // Add link pair row if 1) it is disabled from collision checking or 2) the SHOW ALL LINK PAIRS checkbox is checked
        if( pair_it->second.disable_check )
        {

        std::cout << "Disabled pair "
                  << pair_it->first.first.c_str()
                  << " - "
                  << pair_it->first.second.c_str()
                  << "; Reason: "
                  << moveit_setup_assistant::disabledReasonToString( pair_it->second.reason )
                  << std::endl;
        }
    }
}


bool MoveitComputeDefaultCollisions::save()
{
    // reset the data in the SRDF Writer class
    config_data_->srdf_->disabled_collisions_.clear();

    // Create temp disabled collision
    srdf::Model::DisabledCollision dc;

    // copy the data in this class's LinkPairMap datastructure to srdf::Model::DisabledCollision format
    for ( moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs_.begin();
          pair_it != link_pairs_.end(); ++pair_it)
    {
      // Only copy those that are actually disabled
      if( pair_it->second.disable_check )
      {
        dc.link1_ = pair_it->first.first;
        dc.link2_ = pair_it->first.second;
        dc.reason_ = moveit_setup_assistant::disabledReasonToString( pair_it->second.reason );
        config_data_->srdf_->disabled_collisions_.push_back( dc );
      }
    }

    if(!config_data_->srdf_->writeSRDF(config_data_->srdf_path_))
        return false;

    return true;
}

bool MoveitComputeDefaultCollisions::computeDefaultCollisions(unsigned int num_trials)
{
    double min_frac = .95;

    const bool verbose = true; // Output benchmarking and statistics
    const bool include_never_colliding = true;
    unsigned int collision_progress = 0;

    // clear previously loaded collision matrix entries
    config_data_->getPlanningScene()->getAllowedCollisionMatrixNonConst().clear();

    // Find the default collision matrix - all links that are allowed to collide
    link_pairs_ =
      moveit_setup_assistant::computeDefaultCollisions( config_data_->getPlanningScene(),
                                                        &collision_progress, include_never_colliding, num_trials,
                                                        min_frac, verbose);

    collision_progress = 100;
    return true;
}
