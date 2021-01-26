/*
 * Clear a point cloud basing on colors
 * 1) read a ply 2) write on another ply only point_cloud_with_properties with desised color
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include <utility>
#include <vector>
#include <fstream>

#include "../utils/colors.hpp"

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel EPIC_kernel; 
typedef EPIC_kernel::FT FT;																							
typedef EPIC_kernel::Point_3 Point; 											
typedef EPIC_kernel::Vector_3 Vector;
typedef CGAL::cpp11::array<unsigned char, 3> Color; // a color is a vector of 3 unsigned chars (values 0, 255)		

// define a type for a point with normal, color (P-N-C)
typedef CGAL::cpp11::tuple<Point,Vector,Color> 		PNC;			
typedef CGAL::Nth_of_tuple_property_map<0, PNC>	Point_map;	
typedef CGAL::Nth_of_tuple_property_map<1, PNC>	Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNC>	Color_map;



int main(int argc, char** argv)
{
	if (argc < 3 || argc > 4)
	{
		std::cerr << "ERROR: wrong arguments.\n\tUsage: $ clear_shape [--keep-color] <input_file.ply> <output_file.ply>\n";
		return EXIT_FAILURE;
	}
	if (strcmp(argv[1], "--help") == 0)
	{
		std::cerr << "\tclear_shape [--keep-color] <input_file.ply> <output_file.ply>\n";
		std::cerr << "\nClear a point cloud of points with unidentified colors (the grey ones), "
							<< "or color non belonging to cylindrical shapes.\n"
							<< "Use --keep_color to remove grey points only; else only cylinders will be maintained\n";
		return EXIT_FAILURE;
	}
	
	bool 				keep_color = (strcmp(argv[1], "--keep-color") == 0)? true : false;
	std::string input_file = (argc == 3)? argv[1] : argv[2];
	std::string output_file = (argc == 3)? argv[2]: argv[3];
	
	std::vector<PNC> point_cloud_with_properties; 		
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
	in.close();
	std::cerr << "Read successfully " << point_cloud_with_properties.size() << " point(s)" << std::endl;
	
	// erase PNCs with "wrong" color
	std::cerr << "Selecting points with desired classification...\n";
	std::vector<PNC> point_cloud;
	for (std::vector<PNC>::iterator i = point_cloud_with_properties.begin(); i != point_cloud_with_properties.end(); i++)
	{
		Color& c = get<2>(*i);
		if (keep_color && is_color_value(c))
			point_cloud.push_back(*i);
		else if (is_blue_value(c))
			point_cloud.push_back(*i);
	}
	std::cerr << "Final cloud has " << point_cloud.size() << " point(s)\n";
	
	// saving
	std::ofstream out (output_file);
	std::cerr << "Saving output...\n";
	out	<< "ply " << std::endl
			<< "format ascii 1.0" << std::endl
			<< "element vertex " << point_cloud.size() << std::endl
			<< "property float x" << std::endl
			<< "property float y" << std::endl
			<< "property float z" << std::endl
			<< "property float nx" << std::endl
			<< "property float ny" << std::endl
			<< "property float nz" << std::endl
			<< "property uchar red" << std::endl
			<< "property uchar green" << std::endl
			<< "property uchar blue" << std::endl
			<< "end_header" << std::endl; 

	for (std::vector<PNC>::iterator i = point_cloud.begin(); i != point_cloud.end(); ++i)
	{
		out << get<0>(*i) << " " << get<1>(*i) << " ";
		out << int(get<2>(*i)[0]) << " " << int(get<2>(*i)[1]) << " " << int(get<2>(*i)[2]) << std::endl;
	}
	out.close();
	
	return EXIT_SUCCESS;	
}

 
