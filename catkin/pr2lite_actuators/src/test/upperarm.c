#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main (int argc, char **argv)
{
    double input_angle = 0.0;
    printf("Enter desired angle: ");
    scanf("%lf", &input_angle);
    double c = sqrt(-162.2 * cos((87.58 - input_angle) * M_PI / 180.0) + 256.0);
    printf("LinAct Length = %lf\n", c);
    return 0;
}
