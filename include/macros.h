
#ifndef VVPX4_MACROS_H
#define VVPX4_MACROS_H


#include <rc_math.h>

// helpers to copying out transforms and rotations to float arrays
static inline void vector_to_float(rc_vector_t T, float f[3])
{
	for(int i=0;i<3;i++) f[i] = T.d[i];
	return;
}

static inline void matrix_to_float(rc_matrix_t R, float f[3][3])
{
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			f[i][j] = R.d[i][j];
		}
	}
	return;
}

static inline void float_to_vector(float f[3], rc_vector_t* T)
{
	rc_vector_alloc(T, 3);
	for(int i=0;i<3;i++) T->d[i] = f[i];
	return;
}

static inline void float_to_matrix(float f[3][3], rc_matrix_t* R)
{
	rc_matrix_alloc(R, 3, 3);
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			R->d[i][j] = f[i][j];
		}
	}
	return;
}



#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif

#ifndef PI
#define PI		3.14159265358979323846264338328
#endif

#ifndef TWO_PI
#define TWO_PI	6.28318530717958647692528676656
#endif

#ifndef PI_F
#define PI_F		3.14159265358979323846264338328f
#endif

#ifndef TWO_PI_F
#define TWO_PI_F	6.28318530717958647692528676656f
#endif

#ifndef PI_2
#define PI_2	1.57079632679489661923132169164
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (PI/180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG  (180.0/PI)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
#endif

#ifndef WRAP_TO_NEGPI_TO_PI
#define WRAP_TO_NEGPI_TO_PI(var)          \
{                                         \
  var = fmodf(var, TWO_PI);               \
  if (var < -PI) { var += TWO_PI; }       \
  else if (var > PI) { var -= TWO_PI; }   \
}
#endif

#ifndef WRAP_TO_NEGPI_TO_PI_F
#define WRAP_TO_NEGPI_TO_PI_F(var)          \
{                                         \
  var = fmodf(var, TWO_PI_F);               \
  if (var < -PI_F) { var += TWO_PI_F; }       \
  else if (var > PI_F) { var -= TWO_PI_F; }   \
}
#endif

#ifndef MAX
#define MAX(a, b)   ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)   ((a) < (b) ? (a) : (b))
#endif


#endif // end #define VVPX4_MACROS_H