#include <iostream>
#include "RotationMatrix.h"
#include "Pose3D.h"
#include "RobotDimensions.h"

# define M_E		2.7182818284590452354	/* e */
# define M_LOG2E	1.4426950408889634074	/* log_2 e */
# define M_LOG10E	0.43429448190325182765	/* log_10 e */
# define M_LN2		0.69314718055994530942	/* log_e 2 */
# define M_LN10		2.30258509299404568402	/* log_e 10 */
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
# define M_PI_4		0.78539816339744830962	/* pi/4 */
# define M_1_PI		0.31830988618379067154	/* 1/pi */
# define M_2_PI		0.63661977236758134308	/* 2/pi */
# define M_2_SQRTPI	1.12837916709551257390	/* 2/sqrt(pi) */
# define M_SQRT2	1.41421356237309504880	/* sqrt(2) */
# define M_SQRT1_2	0.70710678118654752440	/* 1/sqrt(2) */

using namespace std;

namespace BodyPart {
    enum Part {
        neck,
        head,
        left_shoulder,
        left_bicep,
        left_elbow,
        left_forearm,
        right_shoulder,
        right_bicep,
        right_elbow,
        right_forearm,
        left_pelvis,
        left_hip,
        left_thigh,
        left_tibia,
        left_ankle,
        left_foot, // rotated at ankle (used for weight)
        left_bottom_foot, // translated (used for pose)
        right_pelvis,
        right_hip,
        right_thigh,
        right_tibia,
        right_ankle,
        right_foot,
        right_bottom_foot,
        torso,
        NUM_PARTS
    };
}

enum Joint {
    HeadYaw = 0,
    HeadPitch = 1,

    LHipYawPitch = 2,
    LHipRoll = 3,
    LHipPitch = 4,
    LKneePitch = 5,
    LAnklePitch = 6,
    LAnkleRoll = 7,

    RHipYawPitch = 8,
    RHipRoll = 9,
    RHipPitch = 10,
    RKneePitch = 11,
    RAnklePitch = 12,
    RAnkleRoll = 13,

    LShoulderPitch = 14,
    LShoulderRoll = 15,
    LElbowYaw = 16,
    LElbowRoll = 17,

    RShoulderPitch = 18,
    RShoulderRoll = 19,
    RElbowYaw = 20,
    RElbowRoll = 21,

    LToePitch = 22,
    RToePitch = 23,

    NUM_JOINTS = 24
};

int main() {
    RobotDimensions dimensions;
    float angles[NUM_JOINTS];

    Pose3D rel_parts[BodyPart::NUM_PARTS];
    // right shoulder
    BodyPart::Part shoulder = BodyPart::right_shoulder;
    Joint arm0 = RShoulderPitch;
    rel_parts[BodyPart::torso] = Pose3D(0, 0, 0).translation(0, dimensions.upperLegLength+
                                                                dimensions.lowerLegLength+
                                                                dimensions.footHeight, 0);

    angles[arm0 + 0] = (float)M_PI_2;
    angles[arm0 + 3] = -(float)M_PI_2;

    rel_parts[shoulder + 0] = Pose3D(rel_parts[BodyPart::torso])
                              .translate(dimensions.armOffset.x, dimensions.armOffset.y * -1, dimensions.armOffset.z)
                              .rotateY(-angles[arm0 + 0]);
    rel_parts[shoulder + 1] = Pose3D(rel_parts[shoulder + 0])
                              .rotateZ(angles[arm0 + 1] * -1);
    rel_parts[shoulder + 2] = Pose3D(rel_parts[shoulder + 1])
                              .translate(dimensions.upperArmLength, 0, 0)
                              .rotateX(angles[arm0 + 2]);
    rel_parts[shoulder + 3] = Pose3D(rel_parts[shoulder + 2])
                              .rotateZ(angles[arm0 + 3] * -1)
                              .translate(dimensions.lowerArmLength, 0, 0);


    cout << rel_parts[shoulder + 3].rotation * Vector3<float>(0, 0, 0) + rel_parts[shoulder + 3].translation<< endl;
//    cout << rel_parts[shoulder + 0].rotation.c[0] << endl;
//    cout << rel_parts[shoulder + 0].rotation.c[1] << endl;
//    cout << rel_parts[shoulder + 0].rotation.c[2] << endl;
    return 0;
}