//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <prlite_kinematics/SphereCoordinate.h>


//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------


#define CHECK_RC(nRetVal, what)										\
if (nRetVal != XN_STATUS_OK)									\
{																\
    printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	return nRetVal;												\
}

#define ADJUST_VALUE 1000.0

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

double calc3DDist( XnPoint3D p1, XnPoint3D p2 ) {
    double dx = p1.X - p2.X;
    double dy = p1.Y - p2.Y;
    double dz = p1.Z - p2.Z;
    return sqrt(dx*dx+dy*dy+dz*dz);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "PersonTracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
    ROS_INFO("Initalizing Arm Connection....");
    ros::Publisher leftArmPublisher = nh.advertise<prlite_kinematics::SphereCoordinate>("/armik/n0/position", 1000);
    ros::Publisher rightArmPublisher = nh.advertise<prlite_kinematics::SphereCoordinate>("/armik/n1/position", 1000);
    ros::Duration(2.0).sleep();
    prlite_kinematics::SphereCoordinate coord;
    coord.radius = 35.0;
    coord.phi = 0.0;
    coord.theta = 0.0;
    leftArmPublisher.publish(coord);
    rightArmPublisher.publish(coord);

    while (ros::ok()) {

                // ---------- ARM CONTROLLER HACK ------------------

                // Calculate arm length of user
                double lenL = calc3DDist( lShould, lElbow ) + calc3DDist( lElbow, lWrist );
                double lenR = calc3DDist( rShould, rElbow ) + calc3DDist( rElbow, rWrist );
                double distL = calc3DDist(lShould, lWrist);
                double distR = calc3DDist(rShould, rWrist);

                // Calculate positions
                prlite_kinematics::SphereCoordinate lCoord;
                prlite_kinematics::SphereCoordinate rCoord;

                lCoord.radius = 35 * (distL / lenL);
                rCoord.radius = 35 * (distR / lenR);

                lCoord.theta = -atan2(lShould.X - lWrist.X, lShould.Z - lWrist.Z);
                rCoord.theta = -atan2(rShould.X - rWrist.X, rShould.Z - rWrist.Z);
                if(lCoord.theta > 1.0) lCoord.theta = 1.0;
                if(lCoord.theta < -1.0) lCoord.theta = -1.0;
                if(rCoord.theta > 1.0) rCoord.theta = 1.0;
                if(rCoord.theta < -1.0) rCoord.theta = -1.0;

                lCoord.phi = -atan2(lShould.Y - lWrist.Y, lShould.X - lWrist.X);
                rCoord.phi = -atan2(rShould.Y - rWrist.Y, rShould.X - rWrist.X);
                if(lCoord.phi > 1.25) lCoord.phi = 1.25;
                if(lCoord.phi < -0.33) lCoord.phi = -0.33;
                if(rCoord.phi > 1.2) rCoord.phi = 1.25;
                if(rCoord.phi < -0.33) rCoord.phi = -0.33;

                ROS_INFO("User %d: Left (%lf,%lf,%lf), Right (%lf,%lf,%lf)", i, lCoord.radius, lCoord.theta, lCoord.phi, rCoord.radius, rCoord.theta, rCoord.phi);

                // Publish to arms
                leftArmPublisher.publish(rCoord);
                rightArmPublisher.publish(lCoord);

                // ---------- END HACK -----------------

                // Publish Transform
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                std::stringstream body_name, neck_name, lshould_name, rshould_name, head_name, relbow_name, lelbow_name, lwrist_name, rwrist_name;
                std::stringstream lhip_name, rhip_name, lknee_name, rknee_name, lfoot_name, rfoot_name;
                body_name << "Torso (" << i << ")" << std::ends;
                neck_name << "Neck (" << i << ")" << std::ends;
                head_name << "Head (" << i << ")" << std::ends;
                lshould_name << "Shoulder L (" << i << ")" << std::ends;
                rshould_name << "Shoulder R (" << i << ")" << std::ends;
                lelbow_name << "Elbow L (" << i << ")" << std::ends;
                relbow_name << "Elbow R (" << i << ")" << std::ends;
                lwrist_name << "Wrist L (" << i << ")" << std::ends;
                rwrist_name << "Wrist R (" << i << ")" << std::ends;
                lhip_name << "Hip L (" << i << ")" << std::ends;
                rhip_name << "Hip R (" << i << ")" << std::ends;
                lknee_name << "Knee L (" << i << ")" << std::ends;
                rknee_name << "Knee R (" << i << ")" << std::ends;
                lfoot_name << "Foot L (" << i << ")" << std::ends;
                rfoot_name << "Foot R (" << i << ")" << std::ends;

                // Publish Torso
                torsoO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(torso.X / ADJUST_VALUE, torso.Y / ADJUST_VALUE, torso.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", body_name.str().c_str()));  
                // Publish Left Shoulder
                lShouldO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lShould.X / ADJUST_VALUE, lShould.Y / ADJUST_VALUE, lShould.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lshould_name.str().c_str()));
                // Publish Right Shoulder
                rShouldO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rShould.X / ADJUST_VALUE, rShould.Y / ADJUST_VALUE, rShould.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rshould_name.str().c_str())); 
                // Publish Left Elbow
                lElbowO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lElbow.X / ADJUST_VALUE, lElbow.Y / ADJUST_VALUE, lElbow.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lelbow_name.str().c_str()));
                // Publish Right Elbow
                rElbowO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rElbow.X / ADJUST_VALUE, rElbow.Y / ADJUST_VALUE, rElbow.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", relbow_name.str().c_str()));
                // Publish Left Wrist
                lWristO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lWrist.X / ADJUST_VALUE, lWrist.Y / ADJUST_VALUE, lWrist.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lwrist_name.str().c_str()));
                // Publish Right Wrist
                rWristO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rWrist.X / ADJUST_VALUE, rWrist.Y / ADJUST_VALUE, rWrist.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rwrist_name.str().c_str()));
                // Publish Neck
                neckO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(neck.X / ADJUST_VALUE, neck.Y / ADJUST_VALUE, neck.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", neck_name.str().c_str())); 
                // Publish Head
                headO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(head.X / ADJUST_VALUE, head.Y / ADJUST_VALUE, head.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", head_name.str().c_str()));
                // Publish Left Hip
                lHipO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lHip.X / ADJUST_VALUE, lHip.Y / ADJUST_VALUE, lHip.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lhip_name.str().c_str()));
                // Publish Right Hip
                rHipO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rHip.X / ADJUST_VALUE, rHip.Y / ADJUST_VALUE, rHip.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rhip_name.str().c_str())); 
                // Publish Left Knee
                lKneeO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lKnee.X / ADJUST_VALUE, lKnee.Y / ADJUST_VALUE, lKnee.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lknee_name.str().c_str()));
                // Publish Right Knee
                rKneeO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rKnee.X / ADJUST_VALUE, rKnee.Y / ADJUST_VALUE, rKnee.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rknee_name.str().c_str())); 
                // Publish Left Foot
                lFootO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lFoot.X / ADJUST_VALUE, lFoot.Y / ADJUST_VALUE, lFoot.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lfoot_name.str().c_str()));
                // Publish Right Foot
                rFootO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rFoot.X / ADJUST_VALUE, rFoot.Y / ADJUST_VALUE, rFoot.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rfoot_name.str().c_str()));
			}
		}
		ros::spinOnce();
		//loop_rate.sleep();
	}
	g_Context.Shutdown();
	return 0;
}


