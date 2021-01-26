/* 
 * COMPUTE ORIENTED NORMALS OF A POINT CLOUD
 * In this file, we read a point cloud from a ply file and we aestimate normal direction and
 * orientation on each point. At the end, we save the file into another ply, useful for 
 * point structuring. 
 * IMPORTANT NOTE: this work can be done also by matlab
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>

#include <utility>
#include <vector>
#include <fstream>
#include <list>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel EPIC_kernel; // EPIC_kernel we use
typedef EPIC_kernel::FT FT;																							// a model of FieldNumberType
typedef EPIC_kernel::Point_3 Point; 											
typedef EPIC_kernel::Vector_3 Vector;
typedef CGAL::cpp11::array<unsigned char, 3> Color; // a color is a vector of 3 unsigned chars (values 0, 255)		

// define a type for a point with normal, color and intensity (P-N-C-I)
typedef CGAL::cpp11::tuple<Point,Vector,Color,int> 	PNCI;				// this is the tuple of properties we want to extract
typedef CGAL::Nth_of_tuple_property_map<0, PNCI>		Point_map;	// group of tuples to focus on point_cloud_with_properties??
typedef CGAL::Nth_of_tuple_property_map<1, PNCI>		Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNCI>		Color_map;
typedef CGAL::Nth_of_tuple_property_map<3, PNCI>		Intensity_map;
// pair <point, normal> both vectors
typedef std::pair<Point, Vector> 										Point_with_normal;

// concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag 		Concurrency_tag;
#else
typedef CGAL::Sequential_tag	Concurrency_tag;
#endif



int main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cerr << "ERROR: no arguments.\n\tUsage: $ compute_onormals <input_file.ply> <output_file.ply>\n";
		return EXIT_FAILURE;
	}
	
	std::string input_file = argv[1];
	std::string output_file = argv[2];
	std::ifstream in (input_file);	
	
	std::vector<PNCI> point_cloud_with_properties; 	
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
		std::cerr << "ERROR: cannot read file " << input_file << std::endl;
		return EXIT_FAILURE;
	}
	
	std::cerr << "File read successfully!\n";
	// divide the information
	std::list<Point_with_normal> 	point_cloud;
	std::vector<Color> 						color_cloud;
	for (std::size_t i=0; i<point_cloud_with_properties.size(); i++)
	{
		Point_with_normal e = Point_with_normal(get<0>(point_cloud_with_properties[i]), Vector(0,0,0));
		point_cloud.push_back(e);
	}
	
	std::cerr << "Estimating normal direction...\n";
	// Estimate normal direction. Note that pca_estimate_normals() (and jet, as well) requires 
	// a range of points as well as property maps to access each point's position and normal
	const int nb_neighbors = 18; // k-nearest neighbors -> 3 rings of 6
	// we can't use jet just because it wants a different type of map for the normal map
	CGAL::pca_estimate_normals<Concurrency_tag> (	point_cloud, nb_neighbors,
																								CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
																								normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
																								
	std::cerr << "Orienting the normals...\n";
	// Orient norals (same note as above)
	// NOTE: the order of the points is modified, so that the points with non-classified 
	// orientation lay in the final part of the array of points.
	std::list<Point_with_normal>::iterator unoriented_points_begin;
	unoriented_points_begin = CGAL::mst_orient_normals(	point_cloud, nb_neighbors,
																											CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
																											normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
	
	// optional: delete points with unoriented normals (useful is reconstruction is needed)
	// if the points to remove end up in being too much ( > 40 %) don't do anything
	std::size_t to_erase = std::distance(unoriented_points_begin, point_cloud.end());
	std::size_t psize = point_cloud.size();
	std::cerr << "Point cloud has " << psize << " pairs point-normal" << std::endl;
	if (double(to_erase) / double(psize) < 0.4)
	{
		std::cerr << "Erasing " << to_erase << " points...\n";
		point_cloud.erase(unoriented_points_begin, point_cloud.end());		
	}
	else
	{
		std::cerr << "Too many points to erase (" << 100 * double(to_erase) / double(psize) << " %): ";
		std::cerr << "no action performed." << std::endl;
	}
	
	// point cloud is reorganized: we need to assign colors 
	std::cerr << "Reorganizing cloud..." << std::endl;
	for (std::list<Point_with_normal>::iterator i = point_cloud.begin(); i != point_cloud.end(); i++)
	{
		for (std::vector<PNCI>::iterator j = point_cloud_with_properties.begin(); j != point_cloud_with_properties.end(); j++)
		{
			if (get<0>(*i) == get<0>(*j))
			{
				color_cloud.push_back(get<2>(*j));
				break; // skip immediately to next piece
			}
		}
	}
				
	// save onto another file
	std::cerr << "Saving file...\n";
	std::ofstream out (output_file);
	out << "ply " << std::endl
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

	std::vector<Color>::iterator ci = color_cloud.begin(); // scroll for color vector
	for (std::list<Point_with_normal>::iterator i = point_cloud.begin(); i != point_cloud.end(); ++i)
	{
		Point& p = get<0>(*i);
		Vector& n = get<1>(*i);
		out << p << " " << n << " ";
		Color& c = (*ci); ci++;
		out << int(c[0]) << " " << int(c[1]) << " " << int(c[2]) << std::endl;
	}
	
	return EXIT_SUCCESS;	
}
 

