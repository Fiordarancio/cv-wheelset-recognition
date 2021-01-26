# Good general option for cmake
cgal_create_CMakeLists -s classification
cmake -DCGAL_DIR=$HOME/CGAL-4.13.1 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=11 .
make 

# for a verbose makefile too
cmake -DCGAL_DIR=$HOME/CGAL-4.13.1 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=11 -DCMAKE_VERBOSE_MAKEFILE=ON .

# for boost libraries (THE HELL)
- update your cmake version first
- build ALL from source, each specific signaled separated
- IMPORTANT use at least -lboost_serialization in make: here is the last line of CMakeLists.txt
target_link_libraries(executablename   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${BOOST_LIBRARIES} -lboost_serialization)

# Modifications to remember (if the previous is not sufficient)
Into CMakeLists.txt, add:
	set (CMAKE_CXX_STANDARD 11)
	
In some other cases, specify it in Makefile too (last lines) with -std=c++11 (better to use cmake option as above)

# External
Could be needed 
	cmake_policy(SET CMP0054 NEW) 
into CMakeCache.txt of main CGAL dir

# ONLY FOR FULL CUSTOM CGAL INSTALLATION
# build with everything it should be needed, instead of searching around
# this code refers to the building of the source itself
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_examples=ON -DWITH_demos=ON 
-DWITH_BLAS=ON -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda 
-DCUDA_SDK_ROOT_DIR=$HOME/opt/cuda-samples/common 
-DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=ON -DWITH_CGAL_ImageIO=ON 
-DWITH_CGAL_Qt5=ON -DWITH_GMP=ON -DWITH_LAPACK=ON -DWITH_MPFI=ON 
-DWITH_MPFR=ON -DWITH_NTL=ON -DWITH_OpenGL=ON -DWITH_ZLIB=ON  
-DWITH_QGLViewer=ON -DWITH_CGAL_Core=ON
 -DQGLVIEWER_INCLUDE_DIR=$HOME/opt/libQGLViewer/include 
-DQGLVIEWER_LIBRARY_RELEASE=$HOME/opt/libQGLViewer/lib/libQGLViewer-qt5.so 
-DCIMG_INCLUDE_DIR=$HOME/cimg -DWITH_tests=ON ..
	
# Other problemAS
Meglio mettere la roba in locale: chmod anche con sudo fa le bizze sulla pennina. Potrebbe essere semplicemente colpa del sospendi e riaccendi, ma non essendo pesante si puo' tranquillamente mettere in home as usual.

Il boost al riavvio non viene identificato nella versione piu' recente, ma siccome non ho voglia di smattarci dell'altro, lascio fare a quella istallata dalle librerie di ubuntu (la 1.58). Gli va bene, quindi lascio correre.
