#! /usr/bin/env python

from moveit_compute_default_collisions import pymcdc
import os

print "BEFORE RUN THE TEST PLEASE BE SURE THAT THE robot6.srdf IS CLEAN OF DISABLED COLLISIONS!"

path =  os.path.dirname(os.path.abspath(__file__))

urdf_path = path + "/robot6.urdf"
srdf_path = path + "/robot6.srdf"

attempt = 100000

mcdc = pymcdc.MoveitComputeDefaultCollisions()
mcdc.initFromPath(urdf_path, srdf_path, False)

if not mcdc.computeDefaultCollisions(attempt):
    print "Error while computing disabled collision pairs"
else:
    mcdc.printDisabledCollisions()
    print "Saving collision pairs to srdf.. "
    mcdc.save()
