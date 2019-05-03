// Assignment 2 - Motion Capture Interpolation.
// Avishkar Kolahalu - 6138428368

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctime>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  // timer:
  clock_t starter = clock();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  double time_taken = clock() - starter;
  printf("Time Taken: %lf\n", time_taken / (double)CLOCKS_PER_SEC);
  system("pause");

}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

// Euler2Rotation Implemented:
void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	int i, j;
	// convert angles:
	double rad[3] = { radConvert(angles[0]), radConvert(angles[1]), radConvert(angles[2]) };
	Rotation temp, final;

	Rotation T1 = { cos(rad[2]), -sin(rad[2]), 0.0, sin(rad[2]), cos(rad[2]), 0.0, 0.0, 0.0, 1.0 };
	Rotation T2 = { cos(rad[1]), 0.0, sin(rad[1]), 0.0, 1.0, 0.0, -sin(rad[1]), 0.0, cos(rad[1]) };

	theMultiplier(T1, T2, temp);

	Rotation T3 = { 1.0, 0.0, 0.0, 0.0, cos(rad[0]), -sin(rad[0]), 0.0, sin(rad[0]), cos(rad[0]) };

	theMultiplier(temp, T3, final);

	for (i = 0; i <= 2; i++)
		for (j = 0; j <= 2; j++)
			R[j + (3*i)] = final[i][j];
}

// Matrix Multiplier:
void Interpolator::theMultiplier(Rotation M1, Rotation M2, Rotation RES)
{
	// init result matrix.
	for (int i = 0; i <= 2; i++)
		for (int j = 0; j <= 2; j++)
			RES[i][j] = 0;

	// multiplier:
	for (int i = 0; i <= 2; i++)
		for (int j = 0; j <= 2; j++)
			for (int k = 0; k <= 2; k++)
				RES[i][j] += (M1[i][k] * M2[k][j]);

}

// BezierInterpolationEuler Implemented:
void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int length, firstframe, lastframe, prevframe, nextframe, f, b;
	double t;
	Quaternion<double> first_q, last_q;
	vector first, last, prev, next, T1;

	length = pInputMotion->GetNumFrames();		// gets total frames.

	// timer:
	clock_t starter = clock();

	for(firstframe = 0; firstframe + N + 1 < length; firstframe = lastframe)
	{
		lastframe = firstframe + N + 1;

		prevframe = firstframe - N - 1;
		if (firstframe == 0)
			prevframe = firstframe;
		vector * prevRot = pInputMotion->GetPosture(prevframe)->bone_rotation;

		nextframe = lastframe + N + 1;
		if (lastframe + N + 1 >= length)
			nextframe = lastframe;
		vector * nextRot = pInputMotion->GetPosture(nextframe)->bone_rotation;

		// copy start and end keyframe
		Posture * startpos = pInputMotion->GetPosture(firstframe);
		pOutputMotion->SetPosture(firstframe, *startpos);

		Posture * lastpos = pInputMotion->GetPosture(lastframe);
		pOutputMotion->SetPosture(lastframe, *lastpos);

		// Interpolation Loop:
		for (f = 1; f <= N; f++)
		{
			Posture interpolate_pos;
			t = 1.0 * f / (N + 1);

			interpolate_pos.root_pos = startpos->root_pos * (1 - t) + lastpos->root_pos * t;

			// Bone Interpolation Loop:
			for (b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
			{
				// Bezier:
				prev = prevRot[b];
				next = nextRot[b];

				first = startpos->bone_rotation[b];
				last = lastpos->bone_rotation[b];

				// an
				prev = first + first - prev;
				prev = (prev + last) * 0.5;
				prev = first + (prev - first) / 3.0;
				
				// bn+1
				T1 = last + last - first;
				next = (T1 + next) * 0.5;
				next = last + last - next;
				next = last + (next - last) / 3.0;

				interpolate_pos.bone_rotation[b] = DeCasteljauEuler(t, first, prev, next, last);
			}
			pOutputMotion->SetPosture(firstframe + f, interpolate_pos);
		}
	}
	
	for (int f2 = firstframe + 1; f2 < length; f2++)
		pOutputMotion->SetPosture(f2, *(pInputMotion->GetPosture(f2)));

	double time_taken = clock() - starter;
	printf("Time Taken: %lf\n", time_taken / (double)CLOCKS_PER_SEC);
	system("pause");

}

// LinearInterpolationQuaternion Implemented:
void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int length, firstframe, lastframe, f, b;
	double s;
	Quaternion<double> firstQ, lastQ;

	length = pInputMotion->GetNumFrames(); // gets total frames.

	// timer:
	clock_t starter = clock();


	for (firstframe = 0; firstframe + N + 1 < length; firstframe = lastframe)
	{
		lastframe = firstframe + N + 1;

		// Start Key.
		Posture * firstpost = pInputMotion->GetPosture(firstframe);
		pOutputMotion->SetPosture(firstframe, *firstpost);

		// End Key.
		Posture * lastpost = pInputMotion->GetPosture(lastframe);
		pOutputMotion->SetPosture(lastframe, *lastpost);

		// Interpolation Loop:
		for (f = 1; f <= N; f++)
		{
			Posture interpolate_pos;
			s = 1.0 * f / (N + 1);

			// interpolate root position
			interpolate_pos.root_pos = firstpost->root_pos * (1 - s) + lastpost->root_pos * s;

			// interpolate bone rotations & quaternions.
			for (b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
			{ 
				Euler2Quaternion(firstpost->bone_rotation[b].p, firstQ);
				Euler2Quaternion(lastpost->bone_rotation[b].p, lastQ);

				Quaternion2Euler(Slerp(s, firstQ, lastQ), interpolate_pos.bone_rotation[b].p);
			}
			pOutputMotion->SetPosture(firstframe + f, interpolate_pos);
		}
	}

	for (int f2 = firstframe + 1; f2 < length; f2++)
	{
		pOutputMotion->SetPosture(f2, *(pInputMotion->GetPosture(f2)));
	}

	double time_taken = clock() - starter;
	printf("Time Taken: %lf\n", time_taken / (double)CLOCKS_PER_SEC);
	system("pause");

}

// BezierInterpolationQuaternion Implemented:
void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int length, firstframe, lastframe, prevframe, nextframe, f, b;
	double t, first_angles[3], last_angles[3], ip[3], prev_angles[3], next_angles[3];;
	Quaternion<double> first_q, last_q, ip_q, prev, next, T1;
	vector first, last;

	length = pInputMotion->GetNumFrames();		// gets total frames.

	// timer:
	clock_t starter = clock();

	for (firstframe = 0; firstframe + N + 1 < length; firstframe = lastframe)
	{
		lastframe = firstframe + N + 1;

		prevframe = firstframe - N - 1;
		if (firstframe == 0)
			prevframe = firstframe;
		vector * prevRot = pInputMotion->GetPosture(prevframe)->bone_rotation;

		nextframe = lastframe + N + 1;
		if (lastframe + N + 1 >= length)
			nextframe = lastframe;
		vector * nextRot = pInputMotion->GetPosture(nextframe)->bone_rotation;

		// copy start and end keyframe
		Posture * startpos = pInputMotion->GetPosture(firstframe);
		pOutputMotion->SetPosture(firstframe, *startpos);

		Posture * lastpos = pInputMotion->GetPosture(lastframe);
		pOutputMotion->SetPosture(lastframe, *lastpos);

		// Interpolation Loop:
		for (f = 1; f <= N; f++)
		{
			Posture interpolate_pos;
			t = 1.0 * f / (N + 1);

			interpolate_pos.root_pos = startpos->root_pos * (1 - t) + lastpos->root_pos * t;

			// Bone Interpolation Loop:
			for (b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
			{
				startpos->bone_rotation[b].getValue(first_angles);
				Euler2Quaternion(first_angles, first_q);

				lastpos->bone_rotation[b].getValue(last_angles);
				Euler2Quaternion(last_angles, last_q);

				prevRot[b].getValue(prev_angles);
				Euler2Quaternion(prev_angles, prev);

				nextRot[b].getValue(next_angles);
				Euler2Quaternion(next_angles, next);

				// an
				prev = Double(prev, first_q);
				prev = Slerp(0.5, prev, last_q);
				prev = Slerp(1.0 / 3.0, first_q, prev);

				// bn+1
				T1 = Double(first_q, last_q);
				next = Slerp(0.5, T1, next);
				next = Double(next, last_q);
				next = Slerp(1.0 / 3.0, last_q, next);

				ip_q = DeCasteljauQuaternion(t, first_q, prev, next, last_q);
				Quaternion2Euler(ip_q, ip);
				interpolate_pos.bone_rotation[b].setValue(ip);
			}
			pOutputMotion->SetPosture(firstframe + f, interpolate_pos);
		}
	}

	for (int f2 = firstframe + 1; f2 < length; f2++)
		pOutputMotion->SetPosture(f2, *(pInputMotion->GetPosture(f2)));

	double time_taken = clock() - starter;
	printf("Time Taken: %lf\n", time_taken / (double)CLOCKS_PER_SEC);
	system("pause");
}

// Euler2Quaternion Implemented:
void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q_temp) 
{
	// Euler to Rotation, then to Quaternion.
	double t_matrix[9];

	Euler2Rotation(angles, t_matrix);

	q_temp = Quaternion<double>::Matrix2Quaternion(t_matrix);
	q_temp.Normalize();
}

// Quaternion2Euler Implemented:
void Interpolator::Quaternion2Euler(Quaternion<double> & q_temp, double angles[3]) 
{
	// Quaternion to Rotation, then to Euler.
	double t_matrix[9];
	q_temp.Quaternion2Matrix(t_matrix);
	Rotation2Euler(t_matrix, angles);
}

// Slerp Implemented:
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
	Quaternion<double> result, quat;

	double dot;
	float d;

	dot = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();

	if(dot >= 0)
		quat = qEnd_;
	else
	{
		dot *= -1;
		quat.Set(-qEnd_.Gets(), -qEnd_.Getx(), -qEnd_.Gety(), -qEnd_.Getz());
	}

	d = acosf(dot);

	if(d != 0)
	{
		result = ((sinf((1 - t) * d) / sinf(d)) * qStart + (sinf(t * d) / sinf(d)) * quat);
		result.Normalize();
		return result;
	}
	else
		return qEnd_;
	
}

// Implemented:
Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;

  double dot = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();

  result = 2 * (dot) * q - p;

  return result;
}

// DeCasteljauEuler Implemented:
vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector r0 = p1 * t + p0 * (1 - t);
  vector r1 = p2 * t + p1 * (1 - t);
  vector s0 = r1 * t + r0 * (1 - t);

  vector r2 = p3 * t + p2 * (1 - t);
  vector s1 = r2 * t + r1 * (1 - t);
  
  vector result = s1 * t + s0 * (1 - t);

  return result;
}

// DeCasteljauQuaternion Implemented:
Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
	Quaternion<double> r0 = Slerp(t, p0, p1);
	Quaternion<double> r1 = Slerp(t, p1, p2);
	Quaternion<double> s0 = Slerp(t, r0, r1);

	Quaternion<double> r2 = Slerp(t, p2, p3);
	Quaternion<double> s1 = Slerp(t, r1, r2);

	Quaternion<double> result = Slerp(t, s0, s1);
	return result;
}

