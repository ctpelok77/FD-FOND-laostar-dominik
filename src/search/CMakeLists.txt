##### NOTE: 
## If CMAKE does not work out for you 
## you can easily revert to the old build system 
## with one make in search folder file by remaning the 	    	
######

cmake_minimum_required (VERSION 2.6)
project (FastDownward)

## Set USE_BLISS to the version (0.5 or 0.72) to enable the BLISS tool. Otherwise, set to 0.
SET(USE_BLISS  "0.72")
SET(BLISS_DIR  bliss-${USE_BLISS})

# Compiler Flags
SET (NORMAL_FLAGS "-std=c++11 -Wall -Wextra  -static-libstdc++ -D USE_BLISS  -Iext -I${BLISS_DIR} -O3 -DNDEBUG -g -pg -lrt")

# Change CXX_FLAGS to switch from NORMAL_FLAGS to DEBUG_FLAGS
SET(CXX_FLAGS ${NORMAL_FLAGS})
SET(CMAKE_CXX_FLAGS ${CXX_FLAGS})

file(GLOB BLISS_CCFILES "${BLISS_DIR}/*.cc")
file(GLOB BASE_CCFILES "*.cc")
file(GLOB MERGE_AND_SHRINK_CCFILES "merge_and_shrink/*.cc")
file(GLOB LANDMARKS_CCFILES "landmarks/*.cc")
file(GLOB LEARNING_CCFILES "learning/*.cc")
file(GLOB PDBS_CCFILES "pdbs/*.cc")
file(GLOB SYMMETRIES_CCFILES "symmetries/*.cc")
file(GLOB LAOSTAR_CCFILES "laostar/*.cc")
file(GLOB GRAPH_ALGORITHMS_CCFILES "graph_algorithms/*.cc")


# .cc files for Modules (Feel free to comment out if not needed right now)
add_library(Libs SHARED
	      ${BASE_CCFILES}
	      ${LAOSTAR_CCFILES}
	      ${LANDMARKS_CCFILES}
	      ${SYMMETRIES_CCFILES}
	      ${LAOSTAR_CCFILES}
	      ${GRAPH_ALGORITHMS_CCFILES}
	      )
add_library(bliss SHARED ${BLISS_CCFILES})
		 
add_executable(downward-release planner.cc)

# for testing
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test)
file(GLOB CATCH_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc") 
#add_executable(fd_test ${CATCH_SOURCES} ${TEST_SOURCES})

# all directories that contain header files
include_directories(${PROJECT_SOURCE_DIR}
 		    #${PROJECT_SOURCE_DIR}/merge_and_shrink
		    ${PROJECT_SOURCE_DIR}/landmarks
		    #${PROJECT_SOURCE_DIR}/learning
 		    #${PROJECT_SOURCE_DIR}/pdbs
		    ${PROJECT_SOURCE_DIR}/por
		    ${PROJECT_SOURCE_DIR}/symmetries
		    ${PROJECT_SOURCE_DIR}/laostar
		    ${PROJECT_SOURCE_DIR}/graph_algorithms
		   )
		   
target_link_libraries (downward-release Libs bliss)
#target_link_libraries (fd_test Libs bliss)
