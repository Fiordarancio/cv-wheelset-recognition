/*
 * The following example applies shape detection followed by structuring to a point set
 * See: https://doc.cgal.org/latest/Point_set_processing_3/index.html#Point_set_processing_3Example_9
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/IO/read_xyz_points.h> // bad: we need more properties
//#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>

// shape detection library
#include <CGAL/Shape_detection_3.h>
#include <CGAL/structure_point_set.h>

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <list>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel	EPIC_kernel;
typedef EPIC_kernel::Point_3 Point;
typedef EPIC_kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> Point_with_normal;
typedef std::vector<Point_with_normal> Pwn_vector; // Point With Normal Vector
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

// We use the algorithm Efficient RANSAC: types
typedef CGAL::Shape_detection_3::Shape_detection_traits<EPIC_kernel, Pwn_vector, Point_map, Normal_map>	Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits> Efficient_RANSAC;
typedef CGAL::Shape_detection_3::Plane<Traits> Plane;

// types
typedef EPIC_kernel::FT FT;																							// a model of FieldNumberType
typedef CGAL::cpp11::array<unsigned char, 3> Color; // a color is a vector of 3 unsigned chars (values 0, 255)		

// define a type for a point with normal, color and intensity (P-N-C-I)
typedef CGAL::cpp11::tuple<Point,Vector,Color,int> 	PNCI;				// this is the tuple of properties we want to extract
typedef CGAL::Nth_of_tuple_property_map<0, PNCI>		PNCI_Point_map;	// group of tuples to focus on point_cloud_with_properties??
typedef CGAL::Nth_of_tuple_property_map<1, PNCI>		PNCI_Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNCI>		Color_map;
typedef CGAL::Nth_of_tuple_property_map<3, PNCI>		Intensity_map;
// pair <point, normal> both vectors
typedef std::pair<Point, Vector> 										Point_with_normal;



int main (int argc, char** argv)
{
	// get an input and output file as usual
	if (argc != 3)
	{
		std::cerr << "ERROR: wrong arguments.\n";
		std::cerr << "\tUsage: structuring <input_file.ply> <output_file.ply>\n";
		return EXIT_FAILURE;
	}
	
	std::string input_file = argv[1];
	std::string output_file = argv[2];
	std::ifstream in (input_file);
	
	// points
	Pwn_vector point_cloud;
	std::vector<PNCI> point_cloud_with_properties; 	
	if (!in || !(CGAL::read_ply_points_with_properties(	in, std::back_inserter(point_cloud_with_properties), 
																											CGAL::make_ply_point_reader (PNCI_Point_map()),
																											std::make_pair (Intensity_map(), CGAL::PLY_property<int>("intensity")),
																											std::make_tuple (	Color_map(),
																																				CGAL::Construct_array(),
																																				CGAL::PLY_property<unsigned char>("red"),
																																				CGAL::PLY_property<unsigned char>("green"),
																																				CGAL::PLY_property<unsigned char>("blue")),
																											CGAL::make_ply_normal_reader (PNCI_Normal_map())
																											)))
	{
		std::cerr << "ERROR: cannot read file " << input_file << std::endl;
		return EXIT_FAILURE;
	}
	else	
		std::cerr << point_cloud_with_properties.size() << " point(s) read\n";
	
	// divide the information
	for (std::vector<PNCI>::iterator i = point_cloud_with_properties.begin(); i != point_cloud_with_properties.end(); i++)
	{
		Point_with_normal pwn = Point_with_normal(get<0>(*i), get<1>(*i));
		point_cloud.push_back(pwn);
	}
	
	// shape detection
	Efficient_RANSAC ransac;
	ransac.set_input(point_cloud);
	ransac.add_shape_factory<Plane>();
	ransac.detect();
	
	Efficient_RANSAC::Plane_range planes = ransac.planes();
	
	// building up the structured point set
	Pwn_vector structured_point_cloud;
	CGAL::structure_point_set (	point_cloud,
															planes,
															std::back_inserter(structured_point_cloud),
															0.015, // epsilon for structuring points
															CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()).
															plane_map (CGAL::Shape_detection_3::Plane_map<Traits>()).
															plane_index_map (CGAL::Shape_detection_3::Point_to_shape_index_map<Traits>(point_cloud, planes)));
	std::cerr << structured_point_cloud.size() << " structured point(s) generated.\n";
	
	// likely, points have been rearranged, and not all points can have been successfully structured.
	// So, colors have to be reordered coherently
	std::cerr << "Reordering point cloud..." << std::endl;
	std::vector<Color> color_cloud;
	for (Pwn_vector::iterator i = structured_point_cloud.begin(); i != structured_point_cloud.end(); i++)
	{
		for (std::vector<PNCI>::iterator j = point_cloud_with_properties.begin(); j != point_cloud_with_properties.end(); j++)
		{
			if (get<0>(*i) == get<0>(*j))
			{
				color_cloud.push_back(get<2>(*j));
				break;
			}
		}
	}
	
	std::ofstream out (output_file);
	// 	print over xyz
//	out.precision(17);
//	CGAL::write_xyz_points (out, structured_point_cloud,
//													CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));
	
	// print over ply as usual
	std::cerr << "Saving file...\n";
	std::ofstream f (output_file);
	f << "ply " << std::endl
		<< "format ascii 1.0" << std::endl
		<< "element vertex " << structured_point_cloud.size() << std::endl
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

	std::vector<Color>::iterator c = color_cloud.begin();
	for (Pwn_vector::iterator i = structured_point_cloud.begin(); i != structured_point_cloud.end(); ++i)
	{
		Point& p = get<0>(*i);
		Vector& n = get<1>(*i);
		f << p << " " << n << " ";
		f << int((*c)[0]) << " " << int((*c)[1]) << " " << int((*c)[2]) << std::endl;
		c++;
	}

	out.close();
	
	return EXIT_SUCCESS;
}
 
