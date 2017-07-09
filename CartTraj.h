
#include <LWRCartImpedanceController.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <TypeIRML.h>

#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif

#ifndef NUMBER_OF_CART_DOFS
#define NUMBER_OF_CART_DOFS			6
#endif

#ifndef NUMBER_OF_FRAME_ELEMENTS

#define NUMBER_OF_FRAME_ELEMENTS	12
#endif

#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif

#ifndef X_AXIS
#define X_AXIS			3
#endif

#ifndef Y_AXIS
#define Y_AXIS			7
#endif

#ifndef Z_AXIS
#define Z_AXIS			11
#endif

#define RUN_TIME_IN_SECONDS			2000.0

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

