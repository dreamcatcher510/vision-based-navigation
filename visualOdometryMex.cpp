#include <iostream>
#include <string.h>
#include <vector>
#include "visualOdometryMex.h"
#include "visualodometry.h"


// initializatino
void VisualOdometryMex:: visualOdometryMex( VisualOdometry *VO,
											std::string command,	// command = "init"
											const double f,			// f ........ focal length (assumes fu=fv)
											const double cu,		// cu, cv ... principal point
											const double cv,
											const double base,		// b ........ baseline (stereo) / height over ground (mono)
											const double pitch )	// p ........ camera pitch (only needed for mono version)
{
	 // init
	if ( !strcmp(command.c_str(),"init") )
	{
		VO = new VisualOdometry();
		VO->setCalibration(f,cu,cv,base,pitch);
	}
}

// close
void VisualOdometryMex:: visualOdometryMex( VisualOdometry *VO, const std::string command )
{
	if ( !strcmp(command.c_str(),"close") )
		delete VO;
}

// updata
bool VisualOdometryMex::visualOdometryMex( VisualOdometry *VO, const std::string command, const double deltaT, const std::vector<Matcher::p_match> matches_i, const std::string method )
{
	if ( !strcmp(command.c_str(),"update") )
	{
		// copy matches
		std::vector<Matcher::p_match> matches;
		Matcher::p_match match;
		int32_t k=0;

		// monocular version
		if ( !strcmp(method.c_str(), "mono") )
		{
			for ( int i=0; i< matches_i.size(); i++ )
			{
				match.u1p = matches_i[i].u1p;
				match.v1p = matches_i[i].v1p;
				match.u1c = matches_i[i].u1c;
				match.v1c = matches_i[i].v1c;
				matches.push_back(match);
			}
		}
		else if (!strcmp(method.c_str(), "stereo"))
		{
			for ( int i=0; i<matches_i.size(); i++ )
			{
				match.u1p = matches_i[i].u1p;
				match.v1p = matches_i[i].v1p;
				match.i1p = matches_i[i].i1p;
				match.u2p = matches_i[i].u2p;
				match.v2p = matches_i[i].v2p;
				match.i2p = matches_i[i].i2p;
				match.u1c = matches_i[i].u1c;
				match.v1c = matches_i[i].v1c;
				match.i1c = matches_i[i].i1c;
				match.u2c = matches_i[i].u2c;
				match.v2c = matches_i[i].v2c;
				match.i2c = matches_i[i].i2c;
				matches.push_back(match);
			}
		}

		// process
		return VO->update( matches, deltaT, !strcmp(method.c_str(), "stereo") );
	}
	else
		return false;

}




