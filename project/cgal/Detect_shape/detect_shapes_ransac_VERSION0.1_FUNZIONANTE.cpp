/*
 * Efficient RANSAC SHAPE DETECTION
 * This algorithm based on a random sampling of points in the cloud in order to retrieve various basic shapes by consensus.
 * The algorithm is not deterministic, but quite fast
 * See: https://doc.cgal.org/latest/Point_set_shape_detection_3/index.html#Point_set_shape_detection_3Method_RANSAC
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
// shape detection
#include <CGAL/Shape_detection_3.h>
#include <CGAL/Line_3.h>
// real time
#include <CGAL/Real_timer.h>

#include <iostream>
#include <fstream>
#include <vector>

// user: colors
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
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>			Efficient_ransac;
// types for the shapes we want to identify
typedef CGAL::Shape_detection_3::Cylinder<Traits>							Cylinder;
typedef CGAL::Shape_detection_3::Plane<Traits>								Plane;
typedef CGAL::Shape_detection_3::Sphere<Traits>								Sphere;
//typedef CGAL::Shape_detection_3::Cone<Traits>									Cone;
//typedef CGAL::Shape_detection_3::Torus<Traits>								Torus;

typedef CGAL::Real_timer																			Real_timer;


// dynamic parameter setting
Efficient_ransac::Parameters set_parameters(std::size_t cloud_size, bool apply_defaults)	
{
	Efficient_ransac::Parameters parameters;	
	if (apply_defaults)
	{
		parameters.probability		 	= 0.01;
		parameters.min_points 			= std::size_t(2.0 * double(cloud_size) / 100);
		parameters.epsilon 					= 0.05;
		parameters.cluster_epsilon	= parameters.epsilon / 2;
		parameters.normal_threshold = 0.9;
	}
	else
	{
		std::cerr << "\tType -1 if you want to use the default value for the given parameters.\n";
		// set probability to miss the largest primitive in each iteration (check for tuning)
		// the lower the probability is, the higher the reliability we ask to the algorithm in 
		// assign points to a certain shape. Default is 0.05
		std::cerr << "Set probability of missing a point. It's about the control of search endurance (default: 0.05)\n";
		parameters.probability = grant_FT_input("prob >> ", "probability must be inside [0, 1]", 0.05, 0.0, 1.0);
		// detect shapes with at least min_points points (vary under 1000)
		std::cerr << "Set minimum number of points to be assigned to an identified shape (default: 1% of the whole cloud)\n";
		parameters.min_points = grant_sizet_input("minp >> ", "number must be greater than 0", ((double)cloud_size)/100.0, 0);
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
		std::cerr << "Set maximum normal deviation: difference in radiants between normal vector and ground truth (default: 0.9)\n";
		parameters.normal_threshold = grant_FT_input("maxd >> ", "deviation must lay in range [0, pi]", 0.9, 0.0, 3.14);
	}
	return parameters;
}

int main(int argc, char** argv)
{
	if (argc < 2 || argc > 6)
	{
		std::cerr << "ERROR: wrong arguments. Tap --help for more info" << std::endl;
		std::cerr << "\tUsage: detect_shapes_ransac [--verbose] [--defaults] <input_file.ply> <output_file.ply>\n";
		return EXIT_FAILURE;
	}
	if (strcmp(argv[1], "--help") == 0)
	{
		std::cerr << "\n\tUsage: detect_shapes_ransac [-v|--verbose] <input_file.ply> <output_file.ply>\n";
		std::cerr << "\nThis program detects shapes inside the point cloud, with particular attention to cylinders.\n";
		std::cerr << "Detectable shapes are: planes, cylinders, spheres, toruses, cones. (check the source)\n";
		std::cerr << "\n--v, --verbose\tinformation about found shapes can be found into the a log file\n";
		std::cerr << "--defaults\tif specified, RANSAC parameters will not be asked in input but default values "
							<< "(0.01/2% size/0.05/0.025/0.9) will be applied\n";
		std::cerr << "--help\t\tdisplay information\n";
		return EXIT_FAILURE;
	}
	
	std::string		infile;
	std::string 	outfile;
	bool					apply_defaults;
	bool 					verbose;
	switch (argc)
	{
		case 3:
			infile = argv[1];
			outfile = argv[2];
			apply_defaults = false;
			verbose = false;
			break;
		case 4:
			infile = argv[2];
			outfile = argv[3];
			verbose = (strcmp("--verbose", argv[1]) == 0)? true : false;
			apply_defaults = (strcmp("--defaults", argv[1]) == 0)? true : false;
			break;
		case 5:
			infile = argv[3];
			outfile = argv[4];
			verbose = (strcmp("--verbose", argv[1]) == 0)? true : false;
			apply_defaults = (strcmp("--defaults", argv[2]) == 0)? true : false;
			break;
	}
	
	std::ifstream 			in 	(infile);
	std::ofstream 			out (outfile);
	Pwn_vector 					point_cloud;
	std::vector<Color>	color_cloud;
	Pwn_vector					output_point_cloud;
	Color								c;
	Real_timer					t;
	
	// logging
	outfile = outfile.substr(0, outfile.find(".ply")).append("_log.txt");
	if (verbose)
		std::cerr << "Saving log in " << outfile << std::endl;
	std::ofstream	out_det (outfile);
	
	// read point cloud
	if (!in || !CGAL::read_ply_points_with_properties(	in, std::back_inserter(point_cloud), 
																											CGAL::make_ply_point_reader (Point_map()),
																											CGAL::make_ply_normal_reader (Normal_map())
																											))
	{
		std::cerr << "ERROR: cannot read file " << argv[1] << std::endl;
		return EXIT_FAILURE;
	}
	
	std::cerr << "Read successfully " << point_cloud.size() << " point(s) with properties...\n";
	std::cerr << "Setting parameters for shape detection...\n";

	// instantiate shape detection engine
	Efficient_ransac ransac;
	// provide input data
	ransac.set_input (point_cloud);
	// register shapes for detection
	ransac.add_shape_factory<Plane>();
	ransac.add_shape_factory<Sphere>();
	ransac.add_shape_factory<Cylinder>();
//	ransac.add_shape_factory<Cone>();
//	ransac.add_shape_factory<Torus>();
	
	//------------------------------------------------------------------------------------------
	// set parameters for shape detection: we make this step interactive, because parameters 
	// should be adjusted according to the point cloud characteristics
	//------------------------------------------------------------------------------------------
	Efficient_ransac::Parameters parameters = set_parameters(point_cloud.size(), apply_defaults);	
	if (verbose)
	{
		out_det << "probability " 		<< parameters.probability << std::endl
						<< "min points "			<< parameters.min_points << std::endl
						<< "epsilon "					<< parameters.epsilon << std::endl
						<< "cluster epsilon " << parameters.cluster_epsilon << std::endl
						<< "normal deviation "<< parameters.normal_threshold << std::endl;
	}
	
	// detect shape 
	std::cerr << "Seeking shapes...\n";
	t.start();
	ransac.detect(parameters);
	t.stop();
	
	// after the work is done, print the number of detected shapes and unassigned points too
	std::cerr << ransac.shapes().end() - ransac.shapes().begin() << " detected shapes, "
						<< ransac.number_of_unassigned_points() << " unassigned point(s) "
						<< "after " << t.time() << " second(s) (" << int(t.time())/60 << " min(s))\n";
	
	// Efficient_ransac.shapes() provides also an iterator range to the detected shapes
	Efficient_ransac::Shape_range	shapes = ransac.shapes();
	std::size_t 									cylinders = 0, planes = 0, spheres = 0, cones = 0, toruses = 0;
	for (Efficient_ransac::Shape_range::iterator s = shapes.begin(); s != shapes.end(); s++)
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
			EPIC_kernel::Line_3 axis = cyl->axis(); // the axis is a point and a direction
			FT radius = cyl->radius();
			if (verbose)
				out_det << "Cylinder " << cylinders << " with axis [" << axis
								<< "] and radius " << radius;
			// calculate axis length in order to cut those with a too short one
//			FT axis_len = sqrt((axis.to_vector()).squared_length()); // ERROR: axis is normalized!!
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
		else if (Sphere* sph = dynamic_cast<Sphere*>(s->get()))
		{
			Point	center = sph->center();
			FT 		radius = sph->radius();
			if (verbose)
				out_det << "Sphere " << spheres
								<< "found with center [" << center 
								<< "] and radius " << radius << std::endl;
			
			c = get_color_value(SPHERE);
			while (index_s != (*s)->indices_of_assigned_points().end())
			{
				// retrieve the point
				const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
				// add this point to the final point cloud plus relative color
				output_point_cloud.push_back(p);
				color_cloud.push_back(c);
				index_s++;
			}
			spheres++;				
		}
//		else if (Cone* con = dynamic_cast<Cone*>(s->get()))
//		{
//			Point 	apex = con->apex();
//			Vector	axis = con->axis();
//			FT			angle = con->angle();
//			if (verbose)	
//				out_det << "Cone " << cones << " found with "
//								<< "apex [" << apex << "] " 
//								<< "axis [" << axis << "] "
//								<< "and base angle " << angle << " rad\n";
//								
//			c = get_color_value(CONE);
//			while (index_s != (*s)->indices_of_assigned_points().end())
//			{
//				// retrieve the point
//				const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
//				// add this point to the final point cloud plus relative color
//				output_point_cloud.push_back(p);
//				color_cloud.push_back(c);
//				index_s++;
//			}
//			cones++;
//		}
//		else if (Torus* tor = dynamic_cast<Torus*>(s->get()))
//		{
//			Point 	center = tor->center();
//			Vector	axis	 = tor->axis();
//			FT			major_radius = tor->major_radius();
//			FT 			minor_radius = tor->minor_radius(); 
//			if (verbose)	
//				out_det << "Torus " << toruses << " found with "
//								<< "center [" << center << "] " 
//								<< "axis [" << axis << "] "
//								<< "major r: " << major_radius 
//								<< " minor r:" << minor_radius << std::endl;
//								
//			c = get_color_value(TORUS);
//			while (index_s != (*s)->indices_of_assigned_points().end())
//			{
//				// retrieve the point
//				const Point_with_normal &p = *(point_cloud.begin() + (*index_s));
//				// add this point to the final point cloud plus relative color
//				output_point_cloud.push_back(p);
//				color_cloud.push_back(c);
//				index_s++;
//			}
//			toruses++;
//		}

		else
		{
			// print the parameters of the detected shape. This function is 
			// available for any type of shape, so have a look to the structures
			// to find out the information that you need
			if (verbose) std::cerr << (*s)->info() << std::endl;
		}
	}
	std::cerr << "Found " << cylinders << " cylinders, "
						<< planes << " planes, " << spheres << " spheres"
//						<< ", " << cones << " cones and " << toruses << " toruses\n";
						<< std::endl;
	out_det.close();
	in.close();
	
	// iterate on all other points
	std::cerr << "Reordering unassigned points...\n";
	Efficient_ransac::Point_index_range::iterator index_s = ransac.indices_of_unassigned_points().begin();
	c = get_color_value(UNDEF);
	while (index_s != ransac.indices_of_unassigned_points().end())
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

