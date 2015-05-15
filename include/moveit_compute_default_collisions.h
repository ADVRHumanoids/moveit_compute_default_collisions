/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __MOVEIT_COMPUTE_DEFAULT_COLLISIONS_H__
#define __MOVEIT_COMPUTE_DEFAULT_COLLISIONS_H__

#include <boost/shared_ptr.hpp>
#include <kdl/frames.hpp>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include <moveit/setup_assistant/tools/srdf_writer.h>

/**
 * @brief The MoveitComputeDefaultCollisions class computes the ACM (allowed collision matrix)
 * for a kinematic structure with collision information loaded from an URDF file, and appends the ACM
 * to an existing (eventually empty) SRDF file. The class supports parsing URDF files with capsules information
 * encoded as cylinders, and if instructed to do so, it will convert all cylinder primitives into capsules.
 */
class MoveitComputeDefaultCollisions {

    /**
     * @brief The Capsule class represents a capsule shape expressed in an arbitrary frame
     */
    class Capsule {
        KDL::Vector ep1;
        KDL::Vector ep2;
        double radius;
        double length;
    public:
        /**
         * @brief Capsule constructs a capsule object
         * @param origin the frame located in the end-point number 1, with z-axis aligned with the capsule axis
         * @param radius the capsule radius
         * @param length the capsule length
         */
        Capsule(const KDL::Frame& origin, const double radius, const double length) :
            radius(radius), length(length)
        {
            ep1 = origin.p - .5*length * origin.M.UnitZ();
            ep2 = origin.p + .5*length * origin.M.UnitZ();
        }

        double getLength() { return this->length; }
        double getRadius() { return this->radius; }
        void getEndPoints(KDL::Vector& ep1, KDL::Vector& ep2) { ep1 = this->ep1; ep2 = this->ep2; }
    };

    /// main storage of link pair data
    moveit_setup_assistant::LinkPairMap link_pairs_;

    /// Contains all the configuration data for the setup assistant
    moveit_setup_assistant::MoveItConfigDataPtr config_data_;

    /**
     * @brief convertCylindersToCapsules traverses an urdf, and for links where a cylinder collision
     * shape has been defined, it will construct a list composed of a cylinder plus two spheres, representing a capsule.
     * @param urdf_model the urdf model to traverse
     */
    void convertCylindersToCapsules(boost::shared_ptr<urdf::Model> urdf_model);
public:
    typedef boost::shared_ptr<MoveitComputeDefaultCollisions> Ptr;

    MoveitComputeDefaultCollisions(const std::string& urdf_path,
                                   const std::string& srdf_path,
                                   const bool& cylinders_to_capsules);

    /**
     * @brief print prints a list of disabled collision pairs
     */
    void print();

    /**
     * @brief save stores the new list of disabled collisions in the specified srdf
     * @return true on success
     */
    bool save();

    /**
     * @brief computeDefaultCollisions computes the default collision for the selected urdf,srdf pair
     * @param num_trials the number of trials of compute default collision to run.
     *        Should be a number between 2000 and 100000. Higher numbers are suggested
     * @return true on success
     */
    bool computeDefaultCollisions(unsigned int num_trials = 75000);
};

#endif
