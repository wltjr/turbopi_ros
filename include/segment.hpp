#ifndef TURBOPI__SEGMENT_H
#define TURBOPI__SEGMENT_H

#include "joint.hpp"

namespace turbopi
{
	template <int T> class Segment
	{
		public:
			std::array<Joint, T> joints;

			Segment() = default;
			~Segment() = default;
			int size() const { return T; }
	};
}

#endif