/*
 * REGION GROWING SHAPE DETECTION
 * This algorithm is not based on random consensus as RANSAC, so it is deterministic, but a bit slower.
 * Trying to identify into a point cloud the plane shape
 * See: https://doc.cgal.org/latest/Point_set_shape_detection_3/index.html#Point_set_shape_detection_3Usage_parameters
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
// shape detection
#include <CGAL/Shape_detection_3.h>
// control the time needed by the algorithm
#include <CGAL/Real_timer.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "../../utils/colors.hpp"
#include "../../utils/checks.hpp"

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel		EPIC_kernel;
typedef EPIC_kernel::FT																				FT;
typedef EPIC_kernel::Point_3																	Point;
typedef EPIC_kernel::Vector_3																	Vector;
typedef std::pair<Point, Vector>															Point_with_normal;
typedef std::vector<Point_with_normal>												Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>		Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> 	Normal_map;
typedef CGAL::cpp11::array<unsigned char, 3> 									Color;

// in Shape_detection_traits, the basic types like Point and Vector are as well an
// iterator type and property maps to be defined
typedef CGAL::Shape_detection_3::Shape_detection_traits<EPIC_kernel, Pwn_vector, Point_map, Normal_map> Traits;
typedef CGAL::Shape_detection_3::Region_growing<Traits>				Region_growing;
// types for the shapes we want to identify
typedef CGAL::Shape_detection_3::Cylinder<Traits>							Cylinder;
typedef CGAL::Shape_detection_3::Plane<Traits>								Plane;
typedef CGAL::Shape_detection_3::Sphere<Traits>								Sphere;
typedef CGAL::Shape_detection_3::Cone<Traits>									Cone;
typedef CGAL::Shape_detection_3::Torus<Traits>								Torus;

typedef CGAL::Real_timer 																			Real_timer;

// dynamic parameter setting
Region_growing::Parameters set_parameters(std::size_t cloud_size)	
{
	Region_growing::Parameters parameters;	
	std::cerr << "\tType -1 if you want to use the default value for the given parameters.\n";
	// detect shapes with at least min_points points (vary under 1000)
	std::cerr << "Set minimum number of points to be assigned to an identified shape (default: 1% of the whole cloud)\n";
	parameters.min_points = grant_sizet_input("minp >> ", "number must be greater than 0", ((double)cloud_size)/100.0, 1, cloud_size);
	// set maimum Eucliedean distance between a point and a shape: higher epsilon leads to
	// lower precision in distinguishing little shapes
	std::cerr << "Set epsilon: maximum Euclidean space between a point and a shape (default: 1% of the diagonal of the bounding box)\n";
	parameters.epsilon = grant_FT_input("epsl >> ", "value must be greater than 0", 0.025, 0.0);
	// set maximum Eucliedean distance between points to be clustered together: higher cluster_epsilon
	// leads to lower precision in isolating the different clusters
	std::cerr << "Set cluster epsilon: maximum Euclidean distance between points clustered together (default: as epsilon)\n";
	parameters.cluster_epsilon = grant_FT_input("clep >> ", "value must be greater than 0", 0.0125);
	// set maximum normal deviation
	// 0.9 < dot(surface_normal, point_normal)
	std::cerr << "Set maximum normal deviation: angle in radiance of how much a normal vector distances the ground truth (default: 0.9)\n";
	parameters.normal_threshold = grant_FT_input("maxd >> ", "deviation must lay in range [0, pi]", 0.9, 0.0, 3.14);
	return parameters;
}


int main(int argc, char** argv)
{
	if (argc < 3 || argc > 5)
	{
		std::cerr << "ERROR: wrong arguments." << std::endl;
		std::cerr << "\tUsage: detect_shapes_rg [-v|--verbose] <input_file.ply> <output_file.ply>\n";
		std::cerr << "\n\tDetect PLANES ONLY over a point cloud with normals, using Region Growing algorithm.\n";
		std::cerr << "\tIf you specify verbose mode, information about those can be found into the log file\n";
		return EXIT_FAILURE;
	}
	
	std::ifstream 			in 	((argc == 3)? argv[1] : argv[2]);
	std::string					outfile = (argc == 3)? argv[2] : argv[3];
	std::ofstream 			out (outfile);
	bool								verbose = (strcmp(argv[1], "-v")==0 || strcmp(argv[1],"--verbose")==0)? true : false;
	Pwn_vector 					point_cloud;
	std::vector<Color>	color_cloud;
	Pwn_vector					output_point_cloud;
	Color								c;
	
	outfile = outfile.substr(0, outfile.find(".ply")).append("_log.txt");
	if (verbose)
		std::cerr << "Saving log in " << outfile << std::endl;
	std::ofstream	out_det (outfile);
	
	if (!in || !CGAL::read_ply_points_with_properties(	in, std::back_inserter(point_cloud), 
																											CGAL::make_ply_point_reader (Point_map()),
																											CGAL::make_ply_normal_reader (Normal_map())
																											))
	{
		std::cerr << "ERROR: cannot read file " << argv[1] << std::endl;
		return EXIT_FAILURE;
	}
	std::cerr << "Read successfully " << point_cloud.size() << " point(s) with properties...\n";
	
//	for (std::vector<PNC>::iterator i = point_cloud_with_properties.begin(); i != point_cloud_with_properties.end(); i++)
//		point_cloud.push_back(Point_with_normal(get<0>(*i), get<1>(*i)));
		
	std::cerr << "Setting parameters for shape detection...\n";
	// instantiate shape detection engine
	Region_growing region_grow;
	// provide input data
	region_grow.set_input (point_cloud);
	// register shapes for detection
	region_grow.add_shape_factory<Plane>();
//	region_grow.add_shape_factory<Sphere>();
	region_grow.add_shape_factory<Cylinder>();
//	region_grow.add_shape_factory<Cone>();
//	region_grow.add_shape_factory<Torus>();
	
	//------------------------------------------------------------------------------------------
	// set parameters for shape detection: we make this step interactive, because parameters 
	// should be adjusted according to the point cloud characteristics
	//------------------------------------------------------------------------------------------
	Region_growing::Parameters parameters = set_parameters(point_cloud.size());	
	if (verbose)
	{
		out_det << "min points "			<< parameters.min_points << std::endl
						<< "epsilon "					<< parameters.epsilon << std::endl
						<< "cluster epsilon " << parameters.cluster_epsilon << std::endl
						<< "normal deviation "<< parameters.normal_threshold << std::endl;
	}
	
	// detect shape 
	Real_timer t;
	std::cerr << "Seeking shapes...\n";
	t.start();
	region_grow.detect(parameters);
	t.stop();
	
	// after the work is done, print the number of detected shapes and unassigned points too
	std::cerr << region_grow.shapes().end() - region_grow.shapes().begin() << " detected shapes, ";
	std::cerr << region_grow.number_of_unassigned_points() << " unassigned point(s) ";
	std::cerr << "after " << t.time() << " seconds" << std::endl;
	
	// Region_growing.shapes() provides also an iterator range to the detected shapes
	Region_growing::Shape_range	shapes = region_grow.shapes();
	std::size_t cylinders = 0, planes = 0;//, spheres = 0, cones = 0, toruses = 0;
	for (Region_growing::Shape_range::iterator s = shapes.begin(); s != shapes.end(); s++)
	{
		// for each shape, assign it a color 
		// (blue for axle, yellow for plane, green for spheres, grey for unassigned)
		
		// each shape has a vector of assigned points
		std::vector<std::size_t>::const_iterator index_s = (*s)->indices_of_assigned_points().begin();
		// get specific parameters dependinf on detected shape
		if (Plane* plane = dynamic_cast<Plane*>(s->get()))
		{
			Vector normal = plane->plane_normal();
			if (verbose)
			{
				out_det	<< "Plane " << planes << " with normal [" << normal << "] > ";
				// plane shape can also be converted into EPIC_kernel::Plane_3
				out_det << "Kernel::Plane_3 [" << static_cast<EPIC_kernel::Plane_3>(*plane) << "]\n";
			}
			
//			c = get_color_value(PLANE);
			c = get_yellow_value(planes);
			while (index_s != (*s)->indices_of_assigned_points().end())
			{
				// retrieve the point
				const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
				// add this point to the final point cloud plus relative color
				output_point_cloud.push_back(p);
				color_cloud.push_back(c);
				index_s++;
			}
			planes++;
		}
		else if (Cylinder* cyl = dynamic_cast<Cylinder*>(s->get()))
		{
			EPIC_kernel::Line_3 axis = cyl->axis();
			FT radius = cyl->radius();
			if (verbose)
				out_det << "Cylinder " << cylinders << " with axis [" << axis
								<< "] and radius " << radius;
			if (radius > 1.0)
			{
				std::cerr << "Cylinder with too high radius: non classified\n";
				if (verbose)
					out_det << " not classified\n";

				c = get_color_value(BIGCYL);
				while (index_s != (*s)->indices_of_assigned_points().end())
				{
					// retrieve the point
					const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
					// add this point to the final point cloud plus relative color
					output_point_cloud.push_back(p);
					color_cloud.push_back(c);
					index_s++;
				}
			}
			else
			{
				if (verbose)
					out_det << std::endl;
					
				c = get_blue_value(cylinders);
				while (index_s != (*s)->indices_of_assigned_points().end())
				{
					// retrieve the point
					const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
					// add this point to the final point cloud plus relative color
					output_point_cloud.push_back(p);
					color_cloud.push_back(c);
					index_s++;
				}
			}
			cylinders++;
		}
		else
		{
			// print the parameters of the detected shape. This function is 
			// available for any type of shape, so have a look to the structures
			// to find out the information that you need
			if (verbose) out_det << (*s)->info() << std::endl;
		}
	}
//	std::cerr << "Found " << cylinders << " cylinders, "
//						<< planes << " planes, " << spheres << " spheres
//						<< cones << " cones and " << toruses << " toruses\n";
	std::cerr << "Found " << cylinders << " cylinders and " << planes << " planes\n";
	out_det.close();
	in.close();
	
	// iterate on all other points
	std::cerr << "Reordering unassigned points...\n";
	Region_growing::Point_index_range::iterator index_s = region_grow.indices_of_unassigned_points().begin();
	c = get_color_value(GREY);
	while (index_s != region_grow.indices_of_unassigned_points().end())
	{
		// retrieve the point
		const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
		// add this point to the final point cloud plus relative color
		output_point_cloud.push_back(p);
		color_cloud.push_back(c);
		index_s++;
	}
	
	// save file
	std::cerr << "Saving output...\n";
	out	<< "ply " << std::endl
			<< "format ascii 1.0" << std::endl
			<< "element vertex " << output_point_cloud.size() << std::endl
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

	std::vector<Color>::iterator ci = color_cloud.begin();
	for (Pwn_vector::iterator i = output_point_cloud.begin(); i != output_point_cloud.end(); ++i)
	{
		Point& p = get<0>(*i);
		Vector& n = get<1>(*i);
		out << p << " " << n << " ";
		out << int((*ci)[0]) << " " << int((*ci)[1]) << " " << int((*ci)[2]) << std::endl;
		ci++;
	}
	out.close();
	
	return EXIT_SUCCESS;
}

