using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;

public class UnityRobotController : MonoBehaviour
{
    // ROS Connection
    private RosConnection ros;

    // Robot components
    public Transform baseLink;
    public Transform leftWheel;
    public Transform rightWheel;

    // ROS topic names
    private string cmdVelTopic = "cmd_vel";
    private string odomTopic = "odom";

    // Robot parameters
    public float wheelRadius = 0.1f;
    public float wheelSeparation = 0.5f;

    // Robot state
    private float linearVelocity = 0.0f;
    private float angularVelocity = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        // Get or create ROS connection
        ros = RosConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Twist>(cmdVelTopic);
        ros.RegisterSubscriber<Float32>(cmdVelTopic, OnVelocityCommand);
    }

    // Update is called once per frame
    void Update()
    {
        // Update robot physics based on velocities
        UpdateRobotPhysics();

        // Publish odometry data
        PublishOdometry();
    }

    void OnVelocityCommand(Twist msg)
    {
        linearVelocity = (float)msg.linear.x;
        angularVelocity = (float)msg.angular.z;
    }

    void UpdateRobotPhysics()
    {
        // Calculate wheel velocities based on differential drive kinematics
        float leftWheelVel = (linearVelocity - angularVelocity * wheelSeparation / 2.0f) / wheelRadius;
        float rightWheelVel = (linearVelocity + angularVelocity * wheelSeparation / 2.0f) / wheelRadius;

        // Apply rotation to wheels
        if (leftWheel != null)
            leftWheel.Rotate(Vector3.right, leftWheelVel * Mathf.Rad2Deg * Time.deltaTime);
        if (rightWheel != null)
            rightWheel.Rotate(Vector3.right, rightWheelVel * Mathf.Rad2Deg * Time.deltaTime);

        // Update base position based on velocities
        if (baseLink != null)
        {
            baseLink.Translate(Vector3.forward * linearVelocity * Time.deltaTime);
            baseLink.Rotate(Vector3.up, angularVelocity * Mathf.Rad2Deg * Time.deltaTime);
        }
    }

    void PublishOdometry()
    {
        // Create and publish odometry message
        // Implementation would include position, velocity, and transform publishing
    }
}