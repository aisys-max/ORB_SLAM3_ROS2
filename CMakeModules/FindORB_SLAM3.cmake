# Try to find ORB_SLAM3
# Set alternative paths to search for using ORB_SLAM3_DIR
# Once done this will define
# You should ensure your ORB_SLAM3 can run correctly
#
# To help the search, set ORB_SLAM3_ROOT_DIR via -DORB_SLAM3_ROOT_DIR=... (colcon --cmake-args)
# or as an environment variable before configuring. Falls back to the upstream default.
if(DEFINED ENV{ORB_SLAM3_ROOT_DIR} AND NOT ORB_SLAM3_ROOT_DIR)
  set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT_DIR}")
elseif(NOT ORB_SLAM3_ROOT_DIR)
  set(ORB_SLAM3_ROOT_DIR "~/Install/ORB_SLAM/ORB_SLAM3")
endif()

# message(${ORB_SLAM3_ROOT_DIR})
# message(${ORB_SLAM3_ROOT_DIR}/include)
# message(${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/DBoW2)

# Find ORB_SLAM3
find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
          PATHS ${ORB_SLAM3_ROOT_DIR}/include)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${ORB_SLAM3_ROOT_DIR}/lib)

# Find built-in DBoW2
find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${ORB_SLAM3_ROOT_DIR})

find_library(DBoW2_LIBRARY NAMES DBoW2
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib)

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib)



include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM3  DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY )

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR} ${DBoW2_INCLUDE_DIR})
