/*
 * REMOVE OUTLIERS FROM A CLOUD
 * note: no problems with a file with no normals: they'll be written down using [0 0 0]
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>

#include <utility>
#include <vector>
#include <fstream>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel; // Kernel we use
typedef Kernel::FT FT;																							// a model of FieldNumberType
typedef Kernel::Point_3 Point; 											
typedef Kernel::Vector_3 Vector;
typedef CGAL::cpp11::array<unsigned char, 3> Color; // a color is a vector of 3 unsigned chars (values 0, 255)		

// define a type for a point with normal, color and (P-N-C)
typedef CGAL::cpp11::tuple<Point,Vector,Color> 		PNC;					// this is the tuple of properties we want to extract
typedef CGAL::Nth_of_tuple_property_map<0, PNC>		Point_map;		// group of tuples to focus on points??
typedef CGAL::Nth_of_tuple_property_map<1, PNC>		Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNC>		Color_map;

int main(int argc, char** argv)
{
	if (argc < 3 || argc > 4)
	{
		std::cerr << " ERROR: wrong arguments.\n\tUsage: $ outliers [-v] <input_file.ply> <output_file.ply>\n";
		return EXIT_FAILURE;
	}
	
	std::string					input_file, output_file;
	bool								verbose;
	if (argc == 3)
	{
		input_file = argv[1];
		output_file = argv[2];
		verbose = false;
	}
	else
	{
		input_file = argv[2];
		output_file = argv[3];
		verbose = true;
	}
	std::vector<PNC> 	point_cloud_with_properties;
	std::vector<Point>	point_cloud; 		
	std::vector<Color>	color_cloud;
	std::ifstream in (input_file);	
	if (!in || !(CGAL::read_ply_points_with_properties(	in, std::back_inserter(point_cloud_with_properties), 
																											CGAL::make_ply_point_reader (Point_map()),
																											std::make_tuple (	Color_map(),
																																				CGAL::Construct_array(),
																																				CGAL::PLY_property<unsigned char>("red"),
																																				CGAL::PLY_property<unsigned char>("green"),
																																				CGAL::PLY_property<unsigned char>("blue")),
																											CGAL::make_ply_normal_reader (Normal_map())
																											)))
	{
		std::cerr << "ERROR: cannot read file " << input_file << std::endl;
		return EXIT_FAILURE;
	}

	//-----------------------------------------------------------------------------------------------------------------------------------
	std::vector<Point> 	point_cloud_to_purge;
	std::vector<Vector> normal_cloud;
	for (std::size_t i=0; i<point_cloud_with_properties.size(); i++)
	{
			point_cloud.push_back(get<0>(point_cloud_with_properties[i]));
			point_cloud_to_purge.push_back(get<0>(point_cloud_with_properties[i]));
			normal_cloud.push_back(get<1>(point_cloud_with_properties[i]));
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
						
	// now use the iterator to find out the other points
	std::cerr << "Erasing points...\n";
	std::vector<Point>::const_iterator 	pi = point_cloud.begin();
	std::vector<Color>::const_iterator 	ci = color_cloud.begin();
	std::vector<Vector>::const_iterator ni = normal_cloud.begin();
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
			normal_cloud.erase(ni);
			// when erased, we get the iterator to the next position, so there is no need
			// to advance. We should instead advance with first_to_remove, and restart from
			// the beginning with the others (high complexity, that's true...)
			first_to_remove++;
			pi = point_cloud.begin();
			ci = color_cloud.begin();
			ni = normal_cloud.begin();
		}
		else
		{
			pi++;
			ci++;
			ni++;
		}
	}
	
	// optional: after erase() use the Scott-Meyer's "swap trick" to trim excess capacity
	std::vector<Point>(point_cloud).swap(point_cloud);
	std::cerr << "Point cloud size is now: " << (double)(point_cloud.size()) << std::endl;
	//-----------------------------------------------------------------------------------------------------------------------------------

	// save the output in another colored PLY format
	std::ofstream out (output_file);
	out	<< "ply " << std::endl
			<< "format ascii 1.0" << std::endl
			<< "element vertex " << point_cloud.size() << std::endl
			<< "property float x" << std::endl
			<< "property float y" << std::endl
			<< "property float z" << std::endl;
	out << "property float nx" << std::endl
			<< "property float ny" << std::endl
			<< "property float nz" << std::endl;
	out	<< "property uchar red" << std::endl
			<< "property uchar green" << std::endl
			<< "property uchar blue" << std::endl
			<< "end_header" << std::endl; 

	for (std::size_t i=0; i<point_cloud.size(); ++i)
	{
		out << point_cloud[i] << " ";
		out << normal_cloud[i] << " ";
		Color& c = color_cloud[i];
		out << int(c[0]) << " " << int(c[1]) << " " << int(c[2]) << std::endl;
	}
	
	return EXIT_SUCCESS;	
}

