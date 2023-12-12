#ifndef TURBOPI__TURBOPI_H
#define TURBOPI__TURBOPI_H

#include <sstream>

#include "segment.hpp"

extern const char* CLASS_NAME;

namespace turbopi
{
	class TurboPi
	{
		private:
		public:
			TurboPi();
			~TurboPi();

			Segment<5> base;
			Segment<2> camera;

			Joint getJoint(std::string jointName);
			void setJoint(turbopi::Joint joint);
	};
}

#endif