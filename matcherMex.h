#ifndef MATCHERMEX_H
#define MATCHERMEX_H

#include <string.h>
#include "matcher.h"

class MatcherMex{
public:
	void matcherMex( std::string command, int nms_n, int nms_tau, int match_binsize, int match_radius,
					int match_disp_tolerance, int outlier_disp_tolerance, int outlier_flow_tolerance,
					int multi_stage, int half_resolution, int refinement );

	void matcherMex( std::string command, uint8_t* I1, const int32_t *dims1 );
	void matcherMex( std::string command, uint8_t* I1, const int32_t *dims1, uint8_t* I2, const int32_t *dims2 );

	// match features
	void matcherMex( std::string command, int32_t method );
	// a bucketed version of match features
	void matcherMex( const std::string command, const int32_t max_features, const float bucket_width, const float bucket_height );

	void matcherMex( std::vector<Matcher::p_match> &matches, std::string command, int32_t method );

	void matcherMex( std::string command );

private:
};

#endif
