#include <prlite_kinematics/arm_ik.h>
#include <stdio.h>

struct point2D makePoint2D(float X, float Y) {
	struct point2D point;
	point.X = X;
	point.Y = Y;
	return point;	
}

struct point3D makePoint3D(float X, float Y, float Z) {
	struct point3D point;
	point.X = X;
	point.Y = Y;
	point.Z = Z;
	return point;
}

struct polar3D makePolar3D(float radius, float theta, float phi) {
	struct polar3D coord;
	coord.theta = theta;
	coord.phi = phi;
	coord.radius = radius;
	return coord;
}
 
struct circle  makeCircle(float X, float Y, float radius) {
	struct circle circ;
	circ.center.X = X;
	circ.center.Y = Y;
	circ.radius = radius;
	return circ;	
}

struct sphere  makeSphere(float X, float Y, float Z, float radius) {
    struct sphere sph;
    sph.center.X = X;
	sph.center.Y = Y;
	sph.center.Z = Z;
	sph.radius = radius;
	return sph;		
}

int solveCircles(struct circle c1, struct circle c2, struct point2D *s1, struct point2D *s2)
{
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;

    /* dx and dy are the vertical and horizontal distances between
    * the circle centers.
    */
    dx = c2.center.X - c1.center.X;
    dy = c2.center.Y - c1.center.Y;

    /* Determine the straight-line distance between the centers. */
    d = hypot(dx,dy); // Suggested by Keith Briggs

    /* Check for solvability. */
    if (d > (c1.radius + c2.radius))
    {
        /* no solution. circles do not intersect. */
        return 1;
    }
    if (d < fabs(c1.radius - c2.radius))
    {
        /* no solution. one circle is contained in the other */
        return 1;
    }

    /* 'point 2' is the point where the line through the circle
    * intersection points crosses the line between the circle
    * centers.  
    */

    /* Determine the distance from point 0 to point 2. */
    a = ((c1.radius*c1.radius) - (c2.radius*c2.radius) + (d*d)) / (2.0 * d) ;

    /* Determine the coordinates of point 2. */
    x2 = c1.center.X + (dx * a/d);
    y2 = c1.center.Y + (dy * a/d);

    /* Determine the distance from point 2 to either of the
    * intersection points.
    */
    h = sqrt((c1.radius*c1.radius) - (a*a));

    /* Now determine the offsets of the intersection points from
    * point 2.
    */
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    s1->X = x2 + rx;
    s2->X = x2 - rx;
    s1->Y = y2 + ry;
    s2->Y = y2 - ry;

    return 0;
}

int solveIK(prlite_kinematics::SphereCoordinate position, struct arm_ik *ik) {
    float goalX = cos(position.phi) * position.radius;
    float goalY = sin(position.phi) * position.radius;
    struct point2D point1, point2;
    if(solveCircles(makeCircle(0, 0, ik->arm_segment1), makeCircle(goalX, goalY, ik->arm_segment2), &point1, &point2)) return 1;
    //printf("Point 1 (%f,%f); Point 2 (%f,%f)\n", point1.X, point1.Y, point2.X, point2.Y);
    ik->shoulder = atan2(point2.Y, point2.X);
    if(ik->shoulder < 0.0) {
        ik->shoulder = atan2(point1.Y, point1.X);
        if(ik->shoulder < 0.0) return 2;
        ik->elbow = atan2(goalY - point1.Y, goalX - point1.X);
    } else ik->elbow = atan2(goalY - point2.Y, goalX - point2.X);
    ik->rotation = position.theta;
    return 0;	
}
