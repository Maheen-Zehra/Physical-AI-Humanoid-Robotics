using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class UnityROSBridge : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string jointStatesTopic = "/joint_states";
    [SerializeField] private string imuTopic = "/imu/data";

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;

        // Subscribe to ROS topics
        ros.Subscribe<Float32Msg>(cmdVelTopic, OnVelocityCommandReceived);
        ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStateReceived);

        // Start publishing sensor data
        StartCoroutine(PublishIMUData());
    }

    void OnVelocityCommandReceived(Float32Msg cmdVel)
    {
        // Process velocity command and move humanoid
        float velocity = cmdVel.data;
        transform.Translate(Vector3.forward * velocity * Time.deltaTime);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        // Process joint state updates
        Debug.Log("Received joint states: " + jointState.name.Length + " joints");
    }

    IEnumerator PublishIMUData()
    {
        while (true)
        {
            var imuMsg = new ImuMsg();
            imuMsg.header = new std_msgs.Header();
            imuMsg.header.stamp = new builtin_interfaces.Time();
            imuMsg.header.frame_id = "imu_link";

            // Simulate linear acceleration (with some noise)
            imuMsg.linear_acceleration.x = Random.Range(-0.1f, 0.1f);
            imuMsg.linear_acceleration.y = Random.Range(-0.1f, 0.1f);
            imuMsg.linear_acceleration.z = -9.8f + Random.Range(-0.2f, 0.2f);

            // Publish IMU data
            ros.Publish(imuTopic, imuMsg);

            yield return new WaitForSeconds(0.01f); // 100Hz
        }
    }

    void Update()
    {
        // Update humanoid transforms based on ROS data
        UpdateHumanoidVisuals();
    }

    void UpdateHumanoidVisuals()
    {
        // Update visual representation based on joint states
    }
}