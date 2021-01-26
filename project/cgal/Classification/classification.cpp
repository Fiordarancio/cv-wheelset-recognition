/*
 * Program to be executed by matlab: example to perform a classification.
 * Save the result in another point cloud that will be recovered later.
 * We exploit the reading of ply with properties to retrieve the color information
 * and use it for classification as well
 * Courtesy of: 
 * (classification) https://doc.cgal.org/lates-/Classification/Classification_2example_classification_8cpp-example.html
 * (read ply props) https://doc.cgal.org/latest/Point_set_processing_3/index.html#Point_set_processing_3Example_ply_read
 */
// optimization directives
#if defined (_MSC_VER) && !defined (_WIN64)
// converts 64 to 32 bits integers
#pragma warning (disable:4244) // boost::number_distance::distance
#endif

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

// classification
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Classification.h>
#include <CGAL/bounding_box.h>
// read ply
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>
// control the time needed by the algorithm
#include <CGAL/Real_timer.h>

#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

typedef CGAL::Simple_cartesian<double> SC_kernel;	// Base work on cartesian world (addressed as GeomTraits)
typedef CGAL::Exact_predicates_inexact_constructions_kernel EPIC_kernel; // kernel for extrating features


typedef SC_kernel::Point_3 Point; 												// we're working in 3D
typedef CGAL::Classification::RGB_Color RGB_Color;				// color on RGB evaluation (array<unsigned char, 3>)
typedef CGAL::Classification::HSV_Color	HSV_Color;				// color on HSV notation
typedef SC_kernel::Iso_cuboid_3 Iso_cuboid_3;							// check out ?
typedef std::vector<Point> Point_range;										// vector of all point cloud
typedef std::vector<HSV_Color> HSV_Color_range;						// vector of all colors of the point cloud
typedef CGAL::Identity_property_map<Point> Pmap; 					// property map related to this kind of point ?
typedef CGAL::Identity_property_map<HSV_Color> HSV_Cmap;	// map for hsv colors

typedef EPIC_kernel::Vector_3 Vector;
typedef CGAL::cpp11::array<unsigned char, 3> Color;
typedef std::vector<Color> Color_range;

// define a type for a point with normal, color and intensity (P-N-C-I)
typedef CGAL::cpp11::tuple<Point, Vector, Color, int> 	PNCI;				// this is the tuple of properties we want to extract
typedef CGAL::Nth_of_tuple_property_map<0, PNCI>				Point_map;	// group of tuples to focus on points??
typedef CGAL::Nth_of_tuple_property_map<1, PNCI>				Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNCI>				Color_map;
typedef CGAL::Nth_of_tuple_property_map<3, PNCI>				Intensity_map;


namespace Classification = CGAL::Classification;

// in any case, we need to choose a classifier. Now, we're using this one
typedef Classification::Sum_of_weighted_features_classifier Classifier;

// maps definition related to the chosen classifier and objects
typedef Classification::Planimetric_grid<SC_kernel, Point_range, Pmap>					Planimetric_grid;
typedef Classification::Point_set_neighborhood<SC_kernel, Point_range, Pmap>		Neighborhood;
typedef Classification::Local_eigen_analysis																		Local_eigen_analysis; // evaluation of local eigenvalues

typedef Classification::Label_handle																						Label_handle;
typedef Classification::Feature_handle																					Feature_handle;
typedef Classification::Label_set																								Label_set;
typedef Classification::Feature_set																							Feature_set;

// definition of features that are gonna be used
typedef Classification::Feature::Distance_to_plane<Point_range, Pmap>						Distance_to_plane;
typedef Classification::Feature::Elevation<SC_kernel, Point_range, Pmap>						Elevation;
typedef Classification::Feature::Vertical_dispersion<SC_kernel, Point_range, Pmap>	Dispersion;

// User-defined feature that identifies if the point (or the area?) uder consideration lies inside a color
// range defined through HSV channels (or RGB ones). This feature takes value 1 for points that lie inside 
// the area and 0 for the others.
class HSV_color_span : public CGAL::Classification::Feature_base
{
  const HSV_Color_range& range;	// group of points
  HSV_Color min, max;			 			// intervals
public:
  HSV_color_span (const HSV_Color_range& range,
              HSV_Color min, HSV_Color max)
    : range(range), min(min), max(max)
  {
    this->set_name ("Color range identification feature");
  }
  // taken a point index in point cloud, tell if it is inside the color range
  float value (std::size_t pt_index) 
  {
    if (min.at(0) < range[pt_index].at(0) && range[pt_index].at(0) < max.at(0) && // hue inside
        min.at(1) < range[pt_index].at(1) && range[pt_index].at(1) < max.at(1) && // saturation inside
        min.at(2) < range[pt_index].at(2) && range[pt_index].at(2) < min.at(2))		// value inside
      return 1.f;
    else
      return 0.f;
  }
    
};

// variant with generic uchar3 color 
class Color_span : public CGAL::Classification::Feature_base
{
  const Color_range& range;			
  Color min, max;			 			// intervals
public:
  Color_span (const Color_range& range,
              Color min, Color max)
    : range(range), min(min), max(max)
  {
    this->set_name ("Color range identification feature");
  }
  // taken a point index in point cloud, tell if it is inside the color range
  float value (std::size_t pt_index) 
  {
    if (min.at(0) < range[pt_index].at(0) && range[pt_index].at(0) < max.at(0) && // hue inside
        min.at(1) < range[pt_index].at(1) && range[pt_index].at(1) < max.at(1) && // saturation inside
        min.at(2) < range[pt_index].at(2) && range[pt_index].at(2) < min.at(2))		// value inside
      return 1.f;
    else
      return 0.f;
  }
    
};

int main (int argc, char** argv)
{
	if (argc < 3 || argc > 5) 
	{
		std::cerr << "ERROR: no arguments." << std::endl;
		std::cerr << "\tUsage: $ classification [-v|--verbose] [--with-properties] <input_file.ply> <output_file.ply>\n";
		return EXIT_FAILURE;
	}
	
	bool 										with_properties, verbose;
	std::string							input_file, output_file;
	switch(argc)
	{
		case 3:
			with_properties = verbose = false;
			input_file = argv[1];
			output_file = argv[2];
			break;
		case 4:
			with_properties = (std::strcmp(argv[1], "--with-properties") == 0)? true : false;
			verbose = ((std::strcmp(argv[1],"-v") == 0) || (std::strcmp(argv[1],"--verbose") == 0))? true : false;
			input_file = argv[2];
			output_file = argv[3];
			break;
		case 5:
			verbose = ((std::strcmp(argv[1],"-v") == 0) || (std::strcmp(argv[1],"--verbose") == 0))? true : false;
			with_properties = (std::strcmp(argv[2], "--with-properties") == 0)? true : false;
			input_file = argv[3];
			output_file = argv[4];
			break;
	}

	std::ifstream 					in (input_file.c_str());
	std::vector<Point>			point_cloud;
	std::vector<HSV_Color>	hsv_color_cloud;
	std::vector<Color>			color_cloud;
	std::vector<PNCI> 			point_cloud_with_properties; 		
	
	std::cerr << "Reading input..." << std::endl;
	if (!with_properties)
	{
		if (!in || !(CGAL::read_ply_points (in, std::back_inserter(point_cloud))))
		{
			std::cerr << "ERROR: cannot read file " << input_file << std::endl;
			return EXIT_FAILURE;
		}
		std::cerr << "File read successfully!" << std::endl;
		
		// file is open, let's store its values
		float 				grid_resolution = 0.34f;
		unsigned int	number_of_neighbors = 6; // max nearest-neighbors to consider
	
		std::cerr << "Computing useful structures..." << std::endl;
	
		Iso_cuboid_3 bbox = CGAL::bounding_box (point_cloud.begin(), point_cloud.end());
	
		// a planimetric grid is built up putting the point_cloud inside the bbox into
		// a 2D grid, where lines have resolution given
		Planimetric_grid grid (point_cloud, Pmap(), bbox, grid_resolution);
		Neighborhood neighborhood (point_cloud, Pmap());
		Local_eigen_analysis eigen = Local_eigen_analysis::create_from_point_set(	point_cloud, Pmap(), 
																																							neighborhood.k_neighbor_query(number_of_neighbors));
																																						
		float radius_neighbors = 0.05f; //1.7f;
		float radius_dtm = 0.7; //15.0f; 			// ??
	
		std::cerr << "Computing features..." << std::endl;
		Feature_set features;
	
// exploit GPU enhancements
#ifdef CGAL_LINKED_WITH_TBB
	features.begin_parallel_additions();
#endif
	
		// declaration of the properties
		Feature_handle distance_to_plane = features.add<Distance_to_plane> (point_cloud, Pmap(), eigen);
		Feature_handle dispersion = features.add<Dispersion> (point_cloud, Pmap(), grid, radius_neighbors);
		Feature_handle elevation = features.add<Elevation> (point_cloud, Pmap(), grid, radius_dtm);
	
		HSV_Color min_color, max_color;
		min_color[0] = 0; min_color[1] = 0; min_color[2] = 0;
		max_color[0] = 40; max_color[1] = 60; max_color[2] = 80;
		Feature_handle color_span = features.add<HSV_color_span> (hsv_color_cloud, min_color, max_color);
	
#ifdef CGAL_LINKED_WITH_TBB
	features.end_parallel_additions();
#endif

		// setting up the required labels (we modify for what we need)
		Label_set labels;
		Label_handle generic = labels.add("generic");
		Label_handle top_engine = labels.add("top_engine");
		Label_handle axle = labels.add("axle");
	
		// setting weights: a fine tuning should be needed!!
		std::cerr << "Setting weights..." << std::endl;
		Classifier classifier (labels, features);
		classifier.set_weight (distance_to_plane, 6.75e-2f);
		classifier.set_weight (dispersion, 5.45e-1f);
		classifier.set_weight (elevation, 1.47e1f);
		classifier.set_weight (color_span, 1.47e1f); // should be more important
	
		// each feature has a kind of peculiarity associated, that helps recognizing features
		std::cerr << "Setting effects..." << std::endl;
		classifier.set_effect (generic, distance_to_plane, Classifier::NEUTRAL);
		classifier.set_effect (generic, dispersion, Classifier::FAVORING);
		classifier.set_effect (generic, elevation, Classifier::NEUTRAL);
		classifier.set_effect (generic, color_span, Classifier::PENALIZING);
	
		classifier.set_effect (top_engine, distance_to_plane, Classifier::NEUTRAL);
		classifier.set_effect (top_engine, dispersion, Classifier::NEUTRAL);
		classifier.set_effect (top_engine, elevation, Classifier::FAVORING);
		classifier.set_effect (top_engine, color_span, Classifier::PENALIZING);
	
		classifier.set_effect (axle, distance_to_plane, Classifier::FAVORING);
		classifier.set_effect (axle, dispersion, Classifier::PENALIZING);
		classifier.set_effect (axle, elevation, Classifier::NEUTRAL);
		classifier.set_effect (axle, color_span, Classifier::FAVORING);
	

	
		// run classification
		std::cerr << "Classifying..." << std::endl;
	
		std::vector<int> label_indices (point_cloud.size(), -1);
	
		CGAL::Real_timer t;
		t.start();
		Classification::classify<Concurrency_tag> (point_cloud, labels, classifier, label_indices);
		t.stop();
		std::cerr << "Raw classification performed in " << t.time() << " seconds [s]" << std::endl;
		t.reset();
	
		t.start();
		Classification::classify_with_local_smoothing<Concurrency_tag> (	point_cloud, Pmap(), labels, classifier,
																																			neighborhood.sphere_neighbor_query(radius_neighbors),
																																			label_indices);
		t.stop();
		std::cerr << "Classification with local smoothing performed in " << t.time () << " seconds [s]" << std::endl;
		t.reset();
	
		t.start();
		Classification::classify_with_graphcut<Concurrency_tag> (	point_cloud, Pmap(), labels, classifier,
																															neighborhood.k_neighbor_query(12), 0.2f, 4, label_indices);
		t.stop();
		std::cerr << "Classification with graphcut performed in " << t.time() << " seconds [s]" << std::endl;
		t.reset();
	
		// save the output in another colored PLY format
		std::ofstream f (output_file);
		f << "ply " << std::endl
			<< "format ascii 1.0" << std::endl
			<< "element vertex " << point_cloud.size() << std::endl
			<< "property float x" << std::endl
			<< "property float y" << std::endl
			<< "property float z" << std::endl
			<< "property uchar red" << std::endl
			<< "property uchar green" << std::endl
			<< "property uchar blue" << std::endl
			<< "end_header" << std::endl; 

		for (std::size_t i=0; i<point_cloud.size(); ++i)
		{
			f << point_cloud[i] << " ";
		
			Label_handle label = labels[std::size_t(label_indices[i])];
			if (label==generic)
				f << "50 50 50" << std::endl;
			else if (label==top_engine)
				f << "0 230 27" << std::endl;
			else if (label==axle)
				f << "255 0 170" << std::endl;
			else
			{
				f << "0 0 0" << std::endl;
				std::cerr << "ERROR: unknonwn classification label" << std::endl;
			}
		}
	}
	else
	{
		if (!in || !(CGAL::read_ply_points_with_properties(	in, std::back_inserter(point_cloud_with_properties), 
																												CGAL::make_ply_point_reader (Point_map()),
																												std::make_pair (Intensity_map(), CGAL::PLY_property<int>("intensity")),
																												std::make_tuple (	Color_map(),
																																					CGAL::Construct_array(),
																																					CGAL::PLY_property<unsigned char>("red"),
																																					CGAL::PLY_property<unsigned char>("green"),
																																					CGAL::PLY_property<unsigned char>("blue")),
																												CGAL::make_ply_normal_reader (Normal_map())
																												)))
		{
			std::cerr << "ERROR: cannot read file " << input_file << " with given properties" << std::endl;
			return EXIT_FAILURE;
		}
		std::cerr << "File with properties (point, normal, color, intensity) read successfully!" << std::endl;
		
		//-----------------------------------------------------------------------------------------------------------------------------------
		std::vector<Point> point_cloud_to_purge;
		for (std::size_t i=0; i<point_cloud_with_properties.size(); i++)
		{
				point_cloud.push_back(get<0>(point_cloud_with_properties[i]));
				point_cloud_to_purge.push_back(get<0>(point_cloud_with_properties[i]));
				color_cloud.push_back(get<2>(point_cloud_with_properties[i]));
		}

		// now we have the point cloud: clean it and save somewhere to check
		// remove outliers using erase-remove idiom (again Identity_property_map is optional)
		const int nb_neighbors = 24; // consider 24 nearest neighbor points

		// Estimate scale of the point set with average spacing
		const double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(point_cloud, nb_neighbors);

		std::cerr << "Point cloud size is: " << (double)(point_cloud_to_purge.size()) << std::endl;

		// FIRST OPTION
		// We don't know the ratio of outliers present in the point set
		std::vector<Point>::iterator first_to_remove;
		first_to_remove = CGAL::remove_outliers(	point_cloud_to_purge, nb_neighbors,
																							CGAL::parameters::threshold_percent (100.). // no limit on the number of outliers to remove
																							threshold_distance(1.5*average_spacing)); 	// points with distance above thresh are outliers
		std::cerr << "Points to cut off: " << std::distance(first_to_remove, point_cloud_to_purge.end());
		std::cerr << std::endl;
		std::cerr	<< (100. * std::distance( first_to_remove, point_cloud_to_purge.end()) / (double)(point_cloud_to_purge.size())) 
							<< "% of the points are considered outliers when using a distance threshold of "
							<< 1.5 * average_spacing << std::endl;
							
							
		std::cerr << "Erasing outliers..." << std::endl;					
		// now use the iterator to find out the other points
		std::vector<Point>::const_iterator pi = point_cloud.begin();
		std::vector<Color>::const_iterator ci = color_cloud.begin();
		while (pi != point_cloud.end())
		{
			// if the point is an outlier, remove it
			if (*first_to_remove == *pi)
			{
				// (*ci) : gets the pointed, an array<uchar, 3>
				// .at() : gets element at given position
				if (verbose) 
				{
					std::cerr << "Erased point " << *pi << " and color ";
					std::cerr << int((*ci).at(0)) << " " << int((*ci).at(1)) << " " << int((*ci).at(2)) << std::endl;
				}
				point_cloud.erase(pi);
				color_cloud.erase(ci);
				// when erased, we get the iterator to the next position, so there is no need
				// to advance. We should instead advance with first_to_remove, and restart from
				// the beginning with the others (high complexity, that's true...)
				first_to_remove++;
				pi = point_cloud.begin();
				ci = color_cloud.begin();
			}
			else
			{
				pi++;
				ci++;
			}
		}
		
		// optional: after erase() use the Scott-Meyer's "swap trick" to trim excess capacity
		std::vector<Point>(point_cloud).swap(point_cloud);
		if (verbose)
			std::cerr << "Point cloud size is now: " << (double)(point_cloud.size()) << std::endl;
		//-----------------------------------------------------------------------------------------------------------------------------------
		// file is open, let's store its values
		float 				grid_resolution = 0.34f;
		unsigned int	number_of_neighbors = 6; // max nearest-neighbors to consider
	
		std::cerr << "Computing useful structures..." << std::endl;
	
		Iso_cuboid_3 bbox = CGAL::bounding_box (point_cloud.begin(), point_cloud.end());
	
		// a planimetric grid is built up putting the point_cloud inside the bbox into
		// a 2D grid, where lines have resolution given
		Planimetric_grid grid (point_cloud, Pmap(), bbox, grid_resolution);
		Neighborhood neighborhood (point_cloud, Pmap());
		Local_eigen_analysis eigen = Local_eigen_analysis::create_from_point_set(	point_cloud, Pmap(), 
																																							neighborhood.k_neighbor_query(number_of_neighbors));
																																						
		float radius_neighbors = 0.05f; //1.7f;
		float radius_dtm = 0.7; //15.0f; 			// ??
	
		std::cerr << "Computing features..." << std::endl;
		Feature_set features;
	
// exploit GPU enhancements
#ifdef CGAL_LINKED_WITH_TBB
	features.begin_parallel_additions();
#endif
	
		// declaration of the properties
		Feature_handle distance_to_plane = features.add<Distance_to_plane> (point_cloud, Pmap(), eigen);
		Feature_handle dispersion = features.add<Dispersion> (point_cloud, Pmap(), grid, radius_neighbors);
		Feature_handle elevation = features.add<Elevation> (point_cloud, Pmap(), grid, radius_dtm);
	
		Color min_color, max_color;
		min_color[0] = 0; min_color[1] = 0; min_color[2] = 0;
		max_color[0] = 40; max_color[1] = 60; max_color[2] = 80;
		Feature_handle color_span = features.add<Color_span> (color_cloud, min_color, max_color);
	
#ifdef CGAL_LINKED_WITH_TBB
	features.end_parallel_additions();
#endif

		// setting up the required labels (we modify for what we need)
		Label_set labels;
		Label_handle generic = labels.add("generic");
		Label_handle top_engine = labels.add("top_engine");
		Label_handle axle = labels.add("axle");
	
		// setting weights: a fine tuning should be needed!!
		std::cerr << "Setting weights..." << std::endl;
		Classifier classifier (labels, features);
		classifier.set_weight (distance_to_plane, 6.75e-2f);
		classifier.set_weight (dispersion, 5.45e-1f);
		classifier.set_weight (elevation, 1.47e1f);
		classifier.set_weight (color_span, 1.47e1f);
	
		// each feature has a kind of peculiarity associated, that helps recognizing features
		std::cerr << "Setting effects..." << std::endl;
		classifier.set_effect (generic, distance_to_plane, Classifier::NEUTRAL);
		classifier.set_effect (generic, dispersion, Classifier::FAVORING);
		classifier.set_effect (generic, elevation, Classifier::NEUTRAL);
		classifier.set_effect (generic, color_span, Classifier::PENALIZING);
	
		classifier.set_effect (top_engine, distance_to_plane, Classifier::NEUTRAL);
		classifier.set_effect (top_engine, dispersion, Classifier::NEUTRAL);
		classifier.set_effect (top_engine, elevation, Classifier::FAVORING);
		classifier.set_effect (top_engine, color_span, Classifier::PENALIZING);
	
		classifier.set_effect (axle, distance_to_plane, Classifier::FAVORING);
		classifier.set_effect (axle, dispersion, Classifier::PENALIZING);
		classifier.set_effect (axle, elevation, Classifier::NEUTRAL);
		classifier.set_effect (axle, color_span, Classifier::FAVORING);
	

	
		// run classification
		std::cerr << "Classifying..." << std::endl;
	
		std::vector<int> label_indices (point_cloud.size(), -1);
	
		CGAL::Real_timer t;
		t.start();
		Classification::classify<Concurrency_tag> (point_cloud, labels, classifier, label_indices);
		t.stop();
		std::cerr << "Raw classification performed in " << t.time() << " seconds [s]" << std::endl;
		t.reset();
	
		t.start();
		Classification::classify_with_local_smoothing<Concurrency_tag> (	point_cloud, Pmap(), labels, classifier,
																																			neighborhood.sphere_neighbor_query(radius_neighbors),
																																			label_indices);
		t.stop();
		std::cerr << "Classification with local smoothing performed in " << t.time () << " seconds [s]" << std::endl;
		t.reset();
	
		t.start();
		Classification::classify_with_graphcut<Concurrency_tag> (	point_cloud, Pmap(), labels, classifier,
																															neighborhood.k_neighbor_query(12), 0.2f, 4, label_indices);
		t.stop();
		std::cerr << "Classification with graphcut performed in " << t.time() << " seconds [s]" << std::endl;
		t.reset();
	
		// save the output in another colored PLY format
		std::ofstream f (output_file);
		f << "ply " << std::endl
			<< "format ascii 1.0" << std::endl
			<< "element vertex " << point_cloud.size() << std::endl
			<< "property float x" << std::endl
			<< "property float y" << std::endl
			<< "property float z" << std::endl
			<< "property uchar red" << std::endl
			<< "property uchar green" << std::endl
			<< "property uchar blue" << std::endl
			<< "end_header" << std::endl; 

		for (std::size_t i=0; i<point_cloud.size(); ++i)
		{
			f << point_cloud[i] << " ";
		
			Label_handle label = labels[std::size_t(label_indices[i])];
			if (label==generic)
				f << "50 50 50" << std::endl; // grey
			else if (label==top_engine)
				f << "0 230 27" << std::endl; // green
			else if (label==axle)
				f << "255 0 170" << std::endl;// magenta
			else
			{
				f << "0 0 0" << std::endl;
				std::cerr << "ERROR: unknonwn classification label" << std::endl;
			}
		}
	}
		
	std::cerr << std::endl << "Classification completed." << std::endl << std:: endl;
	return EXIT_SUCCESS;	
}

