#ifndef COLORS_HPP
#define COLORS_HPP

// types
typedef CGAL::cpp11::array<unsigned char, 3> Color;

Color get_blue_value(std::size_t pos)
{
	Color c;
	switch (pos%10)
	{
		case 0: c[0] = 0; 	c[1] = 255;	c[2] = 255; break;
		case 1: c[0] = 100; c[1] = 200; c[2] = 255; break;
		case 2: c[0] = 50;	c[1] = 150; c[2] = 255; break;
		case 3: c[0] = 0; 	c[1] = 100; c[2] = 255; break;
		case 4: c[0] = 100; c[1] = 150; c[2] = 255; break;
		case 5: c[0] = 0; 	c[1] = 50; 	c[2] = 255; break;
		case 6: c[0] = 150; c[1] = 200; c[2] = 255; break;
		case 7: c[0] = 150; c[1] = 150; c[2] = 255; break;
		case 8: c[0] = 100; c[1] = 0; 	c[2] = 255; break;
		case 9: c[0] = 50; 	c[1] = 200; c[2] = 255; break;
	}
	return c;
}

bool is_blue_value(Color c)
{
	if (c[0] == 0 && 		c[1] == 255 &&	c[2] == 255)	return true;
	if (c[0] == 100 &&	c[1] == 200 &&	c[2] == 255)	return true;
	if (c[0] == 50 && 	c[1] == 150 &&	c[2] == 255)	return true;
	if (c[0] == 0 && 		c[1] == 100 &&	c[2] == 255)	return true;
	if (c[0] == 100 && 	c[1] == 150 &&	c[2] == 255)	return true;
	if (c[0] == 0 &&	 	c[1] == 50 && 	c[2] == 255)	return true;
	if (c[0] == 150 && 	c[1] == 200 &&	c[2] == 255)	return true;
	if (c[0] == 150 && 	c[1] == 150 &&	c[2] == 255)	return true;
	if (c[0] == 100 && 	c[1] == 0 && 		c[2] == 255)	return true;
	if (c[0] == 50 && 	c[1] == 200 && 	c[2] == 255)	return true;
	return false;
}

Color get_yellow_value (size_t pos)
{
	Color c;
	switch (pos%10)
	{
		case 0: c[0] = 255; c[1] = 255;	c[2] = 0; 	break;
		case 1: c[0] = 255; c[1] = 255; c[2] = 50; 	break;
		case 2: c[0] = 255;	c[1] = 255; c[2] = 100; break;
		case 3: c[0] = 255; c[1] = 255; c[2] = 150; break;
		case 4: c[0] = 255; c[1] = 255; c[2] = 200; break;
		case 5: c[0] = 255; c[1] = 200; c[2] = 0; 	break;
		case 6: c[0] = 255; c[1] = 200; c[2] = 50;	break;
		case 7: c[0] = 255; c[1] = 200; c[2] = 100; break;
		case 8: c[0] = 255; c[1] = 200; c[2] = 150; break;
		case 9: c[0] = 255; c[1] = 200; c[2] = 200; break;
	}
	return c;
}

bool is_yellow_value (Color c)
{
	if (c[0] == 255 &&	c[1] == 255 &&	c[2] == 0) 		return true;
	if (c[0] == 255 &&	c[1] == 255 &&	c[2] == 50) 	return true;
	if (c[0] == 255 &&	c[1] == 255 &&	c[2] == 100)	return true;	
	if (c[0] == 255 &&	c[1] == 255 &&	c[2] == 150) 	return true;
	if (c[0] == 255 &&	c[1] == 255 &&	c[2] == 200) 	return true;
	if (c[0] == 255 &&	c[1] == 200 &&	c[2] == 0) 		return true;
	if (c[0] == 255 &&	c[1] == 200 &&	c[2] == 50) 	return true;
	if (c[0] == 255 &&	c[1] == 200 &&	c[2] == 100) 	return true;
	if (c[0] == 255 &&	c[1] == 200 &&	c[2] == 150) 	return true;
	if (c[0] == 255 &&	c[1] == 200 &&	c[2] == 200) 	return true;
	return false;
}

#define UNDEF		0
#define PLANE		1
#define CONE		2
#define TORUS		3
#define SPHERE	4
#define BIGCYL	5
#define WRNGAX	6
#define CYLIND	7
Color get_color_value (int shape)
{
	Color c;
	switch (shape)
	{
		case UNDEF:		c[0] = 100; c[1] = 100; c[2] = 100; break; // medium grey
		case PLANE:		c[0] = 255; c[1] = 255; c[2] = 0;		break; // full yellow
		case CONE: 		c[0] = 255; c[1] = 0; 	c[2] = 0; 	break; // full red
		case TORUS: 	c[0] = 255; c[1] = 150; c[2] = 0;		break; // full orange
		case SPHERE:	c[0] = 0;		c[1] = 255; c[2] = 0;		break; // full green
		case BIGCYL: 	c[0] = 255;	c[1] = 0;		c[2] = 255; break; // full purple
		case WRNGAX:	c[0] = 255; c[1] = 100; c[2] = 200; break; // medium pink
		case CYLIND:	c[0] = 0; 	c[1] = 0; 	c[2] = 255; break; // full blue
	}
	return c;
}

bool is_color_value (Color c)
{
	if (c[0] == 100 && c[1] == 100 &&	c[2] == 100)
		return false;;
	return true;
}


#endif
