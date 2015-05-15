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

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "moveit_compute_default_collisions.h"

int main(int argc, char* argv[])
{
    std::string appName = boost::filesystem::basename(argv[0]);
    std::string urdf_path;
    std::string srdf_path;
    unsigned int num_trials;
    bool cylinders_to_capsules;

    // Define and parse the program options
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
      ("help", "this help message")
      ("urdf_path",
       po::value<std::string>(&urdf_path)->required(),
       "path of urdf file to load")
      ("srdf_path",
       po::value<std::string>(&srdf_path)->required(),
       "path of srdf file to load and store the disabled collision pairs")
      ("num_trials",
       po::value<unsigned int>(&num_trials)->default_value(75000),
       "number of collision trials (each with a randomly sampled joint configuration) for collision checking")
      ("cylinders_to_capsules",
       po::bool_switch(&cylinders_to_capsules));

    po::variables_map vm;
    try
    {
      po::store(po::parse_command_line(argc, argv, desc),
                vm); // can throw

      /** --help option
       */
      if ( vm.count("help")  )
      {
        std::cout << "moveit_compute_default_collision, a command line tool from the moveit setup assistant" << std::endl
                  << "USAGE: moveit_compute_default_collision --urdf_path [urdf file path] --srdf_path [srdf file path] "
                  << std::endl << std::endl;
        std::cout << desc << std::endl;
;
        return 0;
      }

      po::notify(vm); // throws on error, so do after help in case
                      // there are any problems
    }
    catch(po::error& e)
    {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return 1;
    }

    try {
        std::cout << "Loading urdf/srdf pair.." << std::endl;
        MoveitComputeDefaultCollisions::Ptr defaultCollisions(
            new MoveitComputeDefaultCollisions(urdf_path, srdf_path,cylinders_to_capsules));

        std::cout << "Computing disabled collision pairs..." << std::endl;
        if(!defaultCollisions->computeDefaultCollisions())
            std::cout << "Error while computing disabled collision pairs" << std::endl;
        else {
            defaultCollisions->print();
            std::cout << "Saving collision pairs to srdf.. " << std::endl;
            defaultCollisions->save();
        }
    }
    catch(std::exception& e)
    {
      std::cerr << "Unhandled Exception reached the top of main: "
                << e.what() << ", application will now exit" << std::endl;
      return 2;

    }

    return 0;
}
