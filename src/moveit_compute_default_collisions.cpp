#include "moveit_compute_default_collisions.h"
#include <fstream>
#include <moveit/planning_scene/planning_scene.h>


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
    node_ = rclcpp::Node::make_shared("moveit_compute_default_collisions");
}

void MoveitComputeDefaultCollisions::setVerbose(const bool& verbose){
    
    // namespace rc = ros::console;

    // std::map<std::string, rc::levels::Level> ros_logger_levels;

    // rc::Level verbosity_level;
    // if(verbose){
    //     verbosity_level = rc::levels::Info;
    // }
    // else{
    //     verbosity_level = rc::levels::Error;
    // }
    
    // rc::get_loggers(ros_logger_levels);

    // for(auto p : ros_logger_levels)
    // {
    //     rc::set_logger_level(p.first, verbosity_level);
    // }
}

void MoveitComputeDefaultCollisions::initFromPath(const std::string &urdf_path,
                                                               const std::string &srdf_path,
                                                               const bool& cylinders_to_capsules)
{
    // Load URDF from file
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file)
    {
        std::cout << "Failed to open URDF file: " << urdf_path << std::endl;
        throw std::runtime_error("Failed to open URDF file");
    }
    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    urdf_file.close();

    // Load SRDF from file
    std::ifstream srdf_file(srdf_path);
    if (!srdf_file)
    {
        std::cout << "Failed to open SRDF file: " << srdf_path << std::endl;
        throw std::runtime_error("Failed to open SRDF file");
    }
    std::string srdf_string((std::istreambuf_iterator<char>(srdf_file)), std::istreambuf_iterator<char>());
    srdf_file.close();

    initFromString(urdf_string, srdf_string, cylinders_to_capsules);
}

void MoveitComputeDefaultCollisions::initFromString(const std::string& urdf_string,
                                               const std::string& srdf_string,
                                               const bool& cylinders_to_capsules)
{
    auto urdf_model = std::make_shared<urdf::Model>();
    if (!urdf_model->initString(urdf_string))
    {
        std::cout << "Failed to parse urdf robot model" << std::endl;
        throw std::runtime_error("Failed to parse urdf robot model");
    }

    if (cylinders_to_capsules)
    {
        std::cout << "Converting cylinders to capsules..";
        this->convertCylindersToCapsules(urdf_model);
        std::cout << "ok" << std::endl;
    }

    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, srdf_string))
    {
        std::cout << "Failed to parse SRDF robot model!" << std::endl;
        throw std::runtime_error("Failed to parse SRDF robot model!");
    }

    srdf_writer_.initModel(*urdf_model, *srdf_model);

    // Create PlanningScene
    robot_model_loader::RobotModelLoader::Options opts;
    opts.urdf_string_ = urdf_string;
    opts.srdf_string = srdf_string;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader =
        std::make_shared<robot_model_loader::RobotModelLoader>(node_, opts);

    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
    if (!robot_model)
    {
        std::cout << "Failed to construct RobotModel from URDF and SRDF" << std::endl;
        throw std::runtime_error("Failed to construct RobotModel");
    }

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model);
}


void MoveitComputeDefaultCollisions::print()
{
    for ( auto pair_it = link_pairs_.begin();
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
                  << moveit_setup::srdf_setup::disabledReasonToString( pair_it->second.reason )
                  << std::endl;
        }
    }
}


bool MoveitComputeDefaultCollisions::save()
{
    throw std::runtime_error("Save path not specified");
}

bool MoveitComputeDefaultCollisions::save(const std::string& path)
{
    return srdf_writer_.writeSRDF(path);
}

std::string MoveitComputeDefaultCollisions::getXmlString()
{
    return srdf_writer_.getSRDFString();
}

bool MoveitComputeDefaultCollisions::computeDefaultCollisions(unsigned int num_trials)
{
    double min_frac = .95;

    const bool verbose = true; // Output benchmarking and statistics
    const bool include_never_colliding = true;
    unsigned int collision_progress = 0;

    // clear previously loaded collision matrix entries
    planning_scene_->getAllowedCollisionMatrixNonConst().clear();

    // Find the default collision matrix - all links that are allowed to collide
    link_pairs_ =
      moveit_setup::srdf_setup::computeDefaultCollisions( planning_scene_,
                                                        &collision_progress, include_never_colliding, num_trials,
                                                        min_frac, verbose);

    collision_progress = 100;

    // Create temp disabled collision
    srdf::Model::CollisionPair dc;
    std::vector<srdf::Model::CollisionPair> disabled_collisions;

    // copy the data in this class's LinkPairMap datastructure to srdf::Model::DisabledCollision format
    for ( auto pair_it = link_pairs_.begin();
          pair_it != link_pairs_.end(); ++pair_it)
    {
      // Only copy those that are actually disabled
      if( pair_it->second.disable_check )
      {
        dc.link1_ = pair_it->first.first;
        dc.link2_ = pair_it->first.second;
        dc.reason_ = moveit_setup::srdf_setup::disabledReasonToString( pair_it->second.reason );
        disabled_collisions.push_back( dc );
      }
    }

    srdf_writer_.disabled_collision_pairs_ = disabled_collisions;

    return true;
}
