#ifndef VISUALODOMETRYMEX_H
#define VISUALODOMETRYMEX_H

#include "matcher.h"
#include "visualodometry.h"

class VisualOdometryMex{
public:
	// initillization
//	void visualOdometryMex( static VisualOdometry *VO,
//							std::string command,	// command = "init"
//							const double f,			// f ........ focal length (assumes fu=fv)
//							const double cu,		// cu, cv ... principal point
//							const double cv,
//							const double base,		// b ........ baseline (stereo) / height over ground (mono)
//							const double pitch );	// p ........ camera pitch (only needed for mono version)
//
//	// close
//	void visualOdometryMex( static VisualOdometry *VO, const std::string command );
//
//	// updata
//	bool visualOdometryMex( static VisualOdometry *VO, const std::string command, const double deltaT, const std::vector<Matcher::p_match> matches_i, const std::string method );

		void visualOdometryMex( VisualOdometry *VO,
								std::string command,	// command = "init"
								const double f,			// f ........ focal length (assumes fu=fv)
								const double cu,		// cu, cv ... principal point
								const double cv,
								const double base,		// b ........ baseline (stereo) / height over ground (mono)
								const double pitch );	// p ........ camera pitch (only needed for mono version)

		// close
		void visualOdometryMex( VisualOdometry *VO, const std::string command );

		// updata
		bool visualOdometryMex( VisualOdometry *VO, const std::string command, const double deltaT, const std::vector<Matcher::p_match> matches_i, const std::string method );

private:
};

#endif
