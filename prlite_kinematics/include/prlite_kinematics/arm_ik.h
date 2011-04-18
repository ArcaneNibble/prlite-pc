#ifndef _ARM_IK_H_
#define _ARM_IK_H_ 1

#include <ros/ros.h>
#include <prlite_kinematics/SphereCoordinate.h>

#include <cmath>

struct point2D {
    float X;
    float Y;
};

struct point3D {
    float X;
    float Y;
    float Z;
};

struct polar3D {
    float radius;
    float theta;
    float phi;
};

struct circle {
	struct point2D center;
	float radius;	
};

struct sphere {
	struct point3D center;
	float radius;
};

struct arm_ik {
	 // Joint Positions
    float shoulder;
    float elbow;
    float rotation;
    // Arm Dimensions
    float arm_segment1; // Shoulder to Elbow
    float arm_segment2; // Elbow to Gripper Centre
};

struct point2D makePoint2D(float X, float Y);
struct point3D makePoint3D(float X, float Y, float Z);
struct polar3D makePolar3D(float radius, float theta, float phi);
struct circle  makeCircle(float X, float Y, float radius);
struct sphere  makeSphere(float X, float Y, float Z, float radius);

int solveCircles(struct circle c1, struct circle c2, struct point2D *s1, struct point2D *s2);
int solveIK(prlite_kinematics::SphereCoordinate coord, struct arm_ik *ik);

#endif

