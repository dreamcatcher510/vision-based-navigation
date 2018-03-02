#include <iostream>
#include "matcherMex.h"

static Matcher *M;

// init using standard parameters and close
void MatcherMex::matcherMex (  std::string command  )
{
	if ( !strcmp( command.c_str(), "init") )	// init	( or using command == "init" )
	{
		 // use standard parameters
		 M = new Matcher();
	}
	else if ( !strcmp(command.c_str(), "close") )	// close
	{
		delete M;
	}
}

// init using given parameters
void MatcherMex::matcherMex (  std::string command, int nms_n, int nms_tau, int match_binsize, int match_radius,
								int match_disp_tolerance, int outlier_disp_tolerance, int outlier_flow_tolerance,
								int multi_stage, int half_resolution, int refinement  )
{
	if ( !strcmp( command.c_str(), "init") )	// init
	{
		  M = new Matcher( (int32_t)nms_n,
                       (int32_t)nms_tau,
                       (int32_t)match_binsize,
                       (int32_t)match_radius,
                       (int32_t)match_disp_tolerance,
                       (int32_t)outlier_disp_tolerance,
                       (int32_t)outlier_flow_tolerance,
                       (int32_t)multi_stage,
                       (int32_t)half_resolution,
                       (int32_t)refinement);
	}
}

// push back or replace single image
void MatcherMex::matcherMex (  std::string command, uint8_t* I1, const int32_t *dims1  )
{
	if ( !strcmp( command.c_str(), "push") || ! strcmp( command.c_str(), "replace" ) )
	{
		// compute features and put them to ring buffer
		M->pushBack( I1, dims1[0], dims1[1], dims1[0], !strcmp(command.c_str(),"replace") );
	}
}

// push back or replace stereo image pair
void MatcherMex::matcherMex ( std::string command, uint8_t* I1, const int32_t *dims1, uint8_t* I2, const int32_t *dims2 )
{
	// check image size
    if ( dims1[0]!=dims2[0] || dims1[1]!=dims2[1] )
		  std::cout << "Input I1 and I2 must be images of same size ... " << std::endl;

	if ( !strcmp( command.c_str(), "push") || ! strcmp( command.c_str(), "replace" ) )
	{
		// compute features and put them to ring buffer
		M->pushBack( I1,I2,dims1[0],dims1[1],dims1[0], !strcmp(command.c_str(),"replace") );
	}
}

// match features
// matching method: 0 = flow, 1 = stereo, 2 = quad matching
void MatcherMex::matcherMex( std::string command, int32_t method )	// now method is not used
{
	if ( !strcmp(command.c_str(),"match") )
	{
		// do matching
		M->matchFeatures(method);
	}
}

// bucketing
void MatcherMex::matcherMex( const std::string command, const int32_t max_features, const float bucket_width, const float bucket_height )
{
	if ( !strcmp(command.c_str(),"bucketing") )
	{
		// bucketing
		M->bucketFeatures(max_features,bucket_width,bucket_height);
	}
}

// get matches
void MatcherMex::matcherMex( std::vector<Matcher::p_match> &matches, std::string command, int32_t method )
{
	if ( !strcmp(command.c_str(),"getmatches") )
	{
		 // grab matches
		 matches = M->getMatches();
	}
}

// match features a bucketed version
void matcherMex( std::string command, const float max_features, const float bucket_width, const float bucket_height);



