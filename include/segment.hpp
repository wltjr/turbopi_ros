#ifndef TURBOPI__SEGMENT_H
#define TURBOPI__SEGMENT_H

#include "joint.hpp"

namespace turbopi
{
	template <int T> class Segment
	{
		private:

		public:
			Joint joints[T];

			Segment() { };
			~Segment() { };
			int size() const { return T; }
	};
}

#endif