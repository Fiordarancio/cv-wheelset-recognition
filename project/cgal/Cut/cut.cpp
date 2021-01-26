/*
 * CUT FROM A CLOUD PARTS THAT FOR SURE ARE NOT RELATIVE TO THE AXLE
 * Reads Point - Normal - Color and saves it as well
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>

#include <utility>
#include <vector>
#include <fstream>

#define X_UP_LIMIT		100.0
#define Y_UP_LIMIT		0.6
#define Z_UP_LIMIT 		1.0
#define X_DOWN_LIMIT	(-100.0)
#define Y_DOWN_LIMIT	(-0.6)
#define Z_DOWN_LIMIT	0.3

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
	if (argc != 4)
	{
		std::cerr << "ERROR: wrong arguments.\n\tUsage: $ cut <input_file.ply> <output_file.ply> <limits.ply>\n";
		return EXIT_FAILURE;
	}
	
	std::string input_file, output_file, limits_file;
	input_file = argv[1];
	output_file = argv[2];
	limits_file = argv[3];

	std::vector<PNC> point_cloud_with_properties;
	std::vector<PNC> limits;
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
	std::cerr << "Read successfully " << point_cloud_with_properties.size() << " point(s)\n";
	// read limits
	in.open(limits_file);
	if (!in || !CGAL::read_ply_points_with_properties(	in, std::back_inserter(limits), 
																											CGAL::make_ply_point_reader (Point_map())
																											))
	{
		std::cerr << "ERROR: cannot read file " << input_file << std::endl;
		return EXIT_FAILURE;
	}
	if (limits.size() != 2)
	{
		std::cerr << "ERROR: wrong limits size" << std::endl;
		return EXIT_FAILURE;
	}
	const double XMIN = (get<0>(limits[0]))[0];
	const double XMAX = (get<0>(limits[1]))[0];
	const double YMIN = (get<0>(limits[0]))[1];
	const double YMAX = (get<0>(limits[1]))[1];
	const double ZMIN = (get<0>(limits[0]))[2];
	const double ZMAX = (get<0>(limits[1]))[2];
	std::cerr << "[min, max] on x: [" << XMIN << ", " << XMAX << "]\n";
	std::cerr << "[min, max] on y: [" << YMIN << ", " << YMAX << "]\n";
	std::cerr << "[min, max] on z: [" << ZMIN << ", " << ZMAX << "]\n";
	
	std::vector<PNC> point_cloud;
	for (std::vector<PNC>::iterator i = point_cloud_with_properties.begin(); i != point_cloud_with_properties.end(); i++)
	{
		Point& p = get<0>(*i);
		if (p[0] >= XMIN && p[0] <= XMAX)
			if (p[1] >= YMIN && p[1] <= YMAX)
				if (p[2] >= ZMIN && p[2] <= ZMAX)
					point_cloud.push_back(*i);
	}
	
	std::cerr << "Cut cloud has now " << point_cloud.size() << " point(s) ("  
						<< (100.0*double(point_cloud_with_properties.size() - point_cloud.size())/double(point_cloud_with_properties.size()))
						<< " % less)\n";
						
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

	for (std::vector<PNC>::iterator i=point_cloud.begin(); i!=point_cloud.end(); i++)
	{
		out << get<0>(*i) << " " << get<1>(*i) << " ";
		Color& c = get<2>(*i);
		out << int(c[0]) << " " << int(c[1]) << " " << int(c[2]) << std::endl;
	}
	
	return EXIT_SUCCESS;	
}

