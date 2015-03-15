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

class MoveitComputeDefaultCollisions {
public:
    typedef boost::shared_ptr<MoveitComputeDefaultCollisions> Ptr;

    MoveitComputeDefaultCollisions(const std::string& urdf_path,
                                   const std::string& srdf_path);

    /**
     * @brief print prints a list of disabled collision pairs
     */
    void print();

    /**
     * @brief save stores the new list of disabled collisions in the specified srdf
     * @return true on success
     */
    bool save();
};

#endif
