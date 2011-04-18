#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <prlite_kinematics/SphereCoordinate.h>
#include <prlite_kinematics/arm_ik.h>

int main (int argc, char **argv) {
    if(argc < 3) {
        fprintf(stderr, "Usage: %s <radius> <theta> <phi>\n", argv[0]);
        return 1;
    }

    struct arm_ik position;
    position.arm_segment1 = 15.0;
    position.arm_segment2 = 20.0;
    
    prlite_kinematics::SphereCoordinate coord;
    coord.radius = atof(argv[1]);
    coord.theta = atof(argv[2]);
    coord.phi = atof(argv[3]);

    if(solveIK(coord, &position)) {
		fprintf(stderr, "Error - unsolvable\n");
	} else {
		printf("Shoulder: %f\n", position.shoulder);
		printf("Elbow: %f\n", position.elbow);
		printf("Rotation: %f\n", position.rotation);
	}
    return 0;
}
