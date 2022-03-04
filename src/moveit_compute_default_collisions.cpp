#include "moveit_compute_default_collisions.h"

#if ROS_MINOR_VERSION >= 14
    #define DC_VAR_NAME disabled_collision_pairs_
    #define DC_CLASS_NAME CollisionPair
#else 
    #define DC_VAR_NAME disabled_collisions_
    #define DC_CLASS_NAME DisabledCollision
#endif


// construct vector
KDL::Vector toKdl(const urdf::Vector3& v)
{
  return KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
KDL::Rotation toKdl(const urdf::Rotation& r)
{
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
KDL::Frame toKdl(const urdf::Pose& p)
{
  return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct vector
urdf::Vector3 toUrdf(const KDL::Vector& v)
{
    return urdf::Vector3(v.x(), v.y(), v.z());
}

// construct rotation
urdf::Rotation toUrdf(const KDL::Rotation& r)
{
    double x,y,z,w;
    r.GetQuaternion(x,y,z,w);
    return urdf::Rotation(x, y, z, w);
}

// construct pose
urdf::Pose toUrdf(const KDL::Frame& f)
{
    urdf::Pose p;
    p.position = toUrdf(f.p);
    p.rotation = toUrdf(f.M);
    return p;
}


void MoveitComputeDefaultCollisions::convertCylindersToCapsules(MCDC_SHARED_PTR<urdf::Model> urdf_model)
{
    std::vector<MCDC_SHARED_PTR<urdf::Link> > links; urdf_model->getLinks(links);
    typedef std::vector<MCDC_SHARED_PTR<urdf::Link> >::iterator link_it;
    for(link_it it = links.begin();
        it != links.end(); ++it) {
        MCDC_SHARED_PTR<urdf::Link> link = *it;
        if( link->collision &&
            link->collision->geometry &&
            link->collision->geometry->type == urdf::Geometry::CYLINDER )
        {
            Capsule c(  toKdl(link->collision->origin),
                        MCDC_DYN_PTR_CAST<urdf::Cylinder>(link->collision->geometry)->radius,
                        MCDC_DYN_PTR_CAST<urdf::Cylinder>(link->collision->geometry)->length);
            KDL::Vector ep1, ep2;
            c.getEndPoints(ep1,ep2);

            MCDC_SHARED_PTR<urdf::Collision> c1(new urdf::Collision());
#if(moveit_compute_default_collisions_use_xenial == 1)
            c1->group_name = link->collision->group_name;
#endif
            c1->origin.position = toUrdf(ep1);
            c1->origin.rotation = link->collision->origin.rotation;
            MCDC_SHARED_PTR<urdf::Sphere> s1(new urdf::Sphere());
            s1->radius = c.getRadius(); s1->type = urdf::Geometry::SPHERE;
            c1->geometry = s1;

            MCDC_SHARED_PTR<urdf::Collision> c2(new urdf::Collision());
            c2->origin.position = toUrdf(ep2);
            c2->origin.rotation = link->collision->origin.rotation;
            MCDC_SHARED_PTR<urdf::Sphere> s2(new urdf::Sphere());
            s2->radius = c.getRadius(); s2->type = urdf::Geometry::SPHERE;
            c2->geometry = s2;

            // ATM apparently moveit_setup_assistant takes only first collision geometry
            link->collision_array.push_back(link->collision);
            link->collision_array.push_back(c1);
            link->collision_array.push_back(c2);
            link->collision.reset();
        }
    }
}

MoveitComputeDefaultCollisions::MoveitComputeDefaultCollisions()
{

}

void MoveitComputeDefaultCollisions::initFromPath(const std::string &urdf_path,
                                                               const std::string &srdf_path,
                                                               const bool& cylinders_to_capsules)
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
        if(cylinders_to_capsules)
        {
            std::cout << "Converting cylinders to capsules..";
            this->convertCylindersToCapsules(config_data_->urdf_model_);
            std::cout << "ok" << std::endl;
        }

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

void MoveitComputeDefaultCollisions::initFromString(const std::string& urdf_string,
                                               const std::string& srdf_string,
                                               const bool& cylinders_to_capsules)
{
    config_data_.reset(new moveit_setup_assistant::MoveItConfigData());

    if (!config_data_->urdf_model_->initString(urdf_string))
    {
        std::cout<<"Failed to parse urdf robot model"<<std::endl;
        throw new std::runtime_error("Failed to parse urdf robot model");
    }
    else
    {
        if(cylinders_to_capsules)
        {
            std::cout << "Converting cylinders to capsules..";
            this->convertCylindersToCapsules(config_data_->urdf_model_);
            std::cout << "ok" << std::endl;
        }

        config_data_->srdf_->srdf_model_.reset(new srdf::Model());
        if(!config_data_->srdf_->srdf_model_->initString(*config_data_->urdf_model_, srdf_string))
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
    if(!config_data_->srdf_->writeSRDF(config_data_->srdf_path_))
        return false;

    return true;
}

bool MoveitComputeDefaultCollisions::save(const std::string& path)
{
    if(!config_data_->srdf_->writeSRDF(path))
        return false;

    return true;
}

std::string MoveitComputeDefaultCollisions::getXmlString()
{
    if(config_data_)
        return config_data_->srdf_->getSRDFString();
    else
        return "";
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


    // reset the data in the SRDF Writer class
    config_data_->srdf_->DC_VAR_NAME.clear();

    // Create temp disabled collision
    srdf::Model::DC_CLASS_NAME dc;

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
        config_data_->srdf_->DC_VAR_NAME.push_back( dc );
      }
    }


    return true;
}
