#ifndef TRAJECTORY_EVALUATION_H
#define TRAJECTORY_EVALUATION_H

#include <math.h>
#include "trajectory_interface.h"

static double eval_poly_at_t(int n_coef, double *coef, double t)
{
	if (n_coef < 1 || n_coef > TRAJ_MAX_COEFFICIENTS)
	{
		fprintf(stderr,
				"ERROR in %s, number of coefficients must be >=1 && <= %d\n",
				__FUNCTION__, TRAJ_MAX_COEFFICIENTS);
		return NAN;
	}

	double ret = coef[0];
	if (t == 0.0)
	{
		return ret;
	}

	// todo use nested multiplication
	for (int i = 1; i < n_coef; i++)
	{
		ret += coef[i] * pow(t, i);
	}
	return ret;
}

static double eval_vel_at_t(int n_coef, double *coef, double t)
{
	if (n_coef < 1 || n_coef > TRAJ_MAX_COEFFICIENTS)
	{
		fprintf(stderr,
				"ERROR in %s, number of coefficients must be >=1 && <= %d\n",
				__FUNCTION__, TRAJ_MAX_COEFFICIENTS);
		return NAN;
	}
	if (n_coef <= 1)
	{
		return 0.0;
	}

	double ret = coef[1];
	if (t == 0.0)
	{
		return ret;
	}

	// todo use nested multiplication
	for (int i = 2; i < n_coef; i++)
	{
		ret += (double)i * coef[i] * pow(t, i - 1);
	}
	return ret;
}

static double eval_accel_at_t(int n_coef, double *coef, double t)
{
	if (n_coef < 1 || n_coef > TRAJ_MAX_COEFFICIENTS)
	{
		fprintf(stderr,
				"ERROR in %s, number of coefficients must be >=1 && <= %d\n",
				__FUNCTION__, TRAJ_MAX_COEFFICIENTS);
		return NAN;
	}

	if (n_coef <= 2)
	{
		return 0.0;
	}

	double ret = 2.0 * coef[2];

	if (t == 0.0)
	{
		return ret;
	}

	// todo use nested multiplication
	for (int i = 3; i < n_coef; i++)
	{
		ret += (double)(i * (i - 1)) * coef[i] * pow(t, i - 2);
	}
	return ret;
}

#endif