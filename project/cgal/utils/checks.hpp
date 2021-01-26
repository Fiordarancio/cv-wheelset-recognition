// functions for checking quality of an input
#ifndef CHECKS_HPP
#define CHECKS_HPP

#include <iostream>
#include <cstring>
#include <stdexcept>
// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel		EPIC_kernel;
typedef EPIC_kernel::FT																				FT;

FT grant_FT_input (std::string base_msg, std::string err_msg, FT def_val, FT min=0.0, FT max=-1.0)
{
	bool 				isGood = false;
	std::string	val;
	FT					value;
	while (!isGood)
	{
		std::cerr << base_msg;
		std::cin 	>> val;
		try 
		{
			value = std::stod(val);
				if (value == -1)
			{
				value = def_val;
				isGood = true;
			}
			else if (value >= min)		
			{	
				if (max < 0)
					isGood = true;
				else if (value <= max)
					isGood = true;
				else
					std::cerr << "ERROR: " << err_msg << std::endl;	
			}
			else
				std::cerr << "ERROR: " << err_msg << std::endl;
		} 
		catch (const std::invalid_argument& ia) {
	  	std::cerr << "Invalid argument: " << ia.what() << std::endl;
	  }
	}
	return value;
}

std::size_t grant_sizet_input (	std::string base_msg, std::string err_msg, 
																std::size_t def_val, std::size_t min=0, int max=-1)
{
	bool 					isGood = false;
	std::size_t		value;
	std::string 	val;
	while (!isGood)
	{
		std::cerr << base_msg;
		std::cin 	>> val;
		try
		{
			value = std::stoi(val, &value);
			if (value == -1)
			{
				value = def_val;
				isGood = true;
			}
			else if (value >= min)		
			{	
				if (max < 0)
					isGood = true;
				else if (value <= max)
					isGood = true;
				else
					std::cerr << "ERROR: " << err_msg << std::endl;	
			}
			else
				std::cerr << "ERROR: " << err_msg << std::endl;
		}
		catch (const std::invalid_argument& ia) {
	  	std::cerr << "Invalid argument: " << ia.what() << std::endl;
	  }
	}
	return value;
}

#endif

