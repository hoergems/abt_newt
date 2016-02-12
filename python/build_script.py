import subprocess
import os
import sys

"""
Usage: build_scripy.py start_index end_index jobs
"""

abs_path = os.path.dirname(os.path.abspath(__file__))
k = int(sys.argv[1])
k_end = int(sys.argv[2]) + 1
jobs = 3
if len(sys.argv) > 4:
    jobs = int(sys.argv[3])    

cmake_cmd = "cmake -DCMAKE_BUILD_TYPE=Release .."
cmake_clean_cmd = "rm -rf *" 
cmake_mkdir_cmd = "mkdir build"
cmake_make_cmd = "make -j" + str(jobs)

for i in xrange(k, k_end):
    if not os.path.exists(abs_path + "/abt" + str(i)):        
        cp_cmd = "cp -r abt abt" + str(i)
        os.system(cp_cmd)
        #popen = subprocess.Popen(cp_cmd, cwd=abs_path, shell=True)
        #popen.wait()
        
    #args = "cd abt" + str(i) + "/src/problems/shared && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .."
    shared_path = abs_path + "/abt" + str(i) + "/src/problems/shared"    
    shared_build_path = shared_path + "/build"
    make_path = abs_path + "/abt" + str(i) + "/src/problems/manipulator_discrete"
    
    if not os.path.exists(shared_build_path):         
        popen = subprocess.Popen(cmake_mkdir_cmd, cwd=shared_path, shell=True)
        popen.wait()
    
    popen = subprocess.Popen(cmake_clean_cmd, cwd=shared_build_path, shell=True)
    popen.wait()
    
    popen = subprocess.Popen(cmake_cmd, cwd=shared_build_path, shell=True)
    popen.wait()
    
    popen = subprocess.Popen(cmake_make_cmd, cwd=shared_build_path, shell=True)
    popen.wait()
    
    popen = subprocess.Popen(cmake_make_cmd, cwd=make_path, shell=True)
    popen.wait()
    
    print "built abt" + str(i)
    
    
       
    