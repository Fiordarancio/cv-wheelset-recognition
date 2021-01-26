/*
 * PLY READING EXAMPLE - courtesy of CGAL 4.14 Documentation
 * Using the homonymous function, we are able to read a point set along with its 
 * normals, RGB colors and intensity etc. and store these attributes in a 
 * user-defined container. As usual, CGAL is tremendously general, so we have to
 * point out some things to avoid messing up.
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include <utility>
#include <vector>
#include <fstream>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel; // Kernel we use
typedef Kernel::FT FT;																							// a model of FieldNumberType
typedef Kernel::Point_3 Point; 											
typedef Kernel::Vector_3 Vector;
typedef CGAL::cpp11::array<unsigned char, 3> Color; // a color is a vector of 3 unsigned chars (values 0, 255)		

// define a type for a point with normal, color and intensity (P-N-C-I)
typedef CGAL::cpp11::tuple<Point,Vector,Color,int> 	PNCI;				// this is the tuple of properties we want to extract
typedef CGAL::Nth_of_tuple_property_map<0, PNCI>		Point_map;	// group of tuples to focus on points??
typedef CGAL::Nth_of_tuple_property_map<1, PNCI>		Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNCI>		Color_map;
typedef CGAL::Nth_of_tuple_property_map<3, PNCI>		Intensity_map;


int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cerr << "ERROR: wrong arguments.\n";
		std::cerr << "\tUsage: $ read_ply_with_properties <filename.ply>\n";
		return EXIT_FAILURE;
	}
	
	std::string filename = argv[1];
	std::vector<PNCI> points; 		// to store points
	std::ifstream in (filename);	
	if (!in || !(CGAL::read_ply_points_with_properties(	in, std::back_inserter(points), 
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
		std::cerr << "ERROR: cannot read file " << filename << std::endl;
		return EXIT_FAILURE;
	}
	
	// display points read: it is important to notice that the previous function automatically detects,
	// thanks to the header of the PLY file, which and where the relative information are stored
	for (std::size_t i=0; i<points.size(); i++)
	{
		const Point& 	p = get<0>(points[i]);
		const Vector& n = get<1>(points[i]);
		const Color& 	c = get<2>(points[i]);
		int 					I = get<3>(points[i]);
		std::cerr << "Point (" << p << ") with normal (" << n
							<< "), color (" << int(c[0]) << " " << int(c[1]) << " " << int(c[2]) 
							<< ") and intensity " << I << std::endl;
	}
	return EXIT_SUCCESS;	
}

