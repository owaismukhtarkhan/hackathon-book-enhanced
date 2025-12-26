using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class UnitySceneSetup : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Environment Settings")]
    public GameObject[] obstacles;
    public Material[] environmentMaterials;

    private RosConnection ros;

    void Start()
    {
        // Initialize ROS connection
        InitializeROSConnection();

        // Setup environment
        SetupEnvironment();
    }

    void InitializeROSConnection()
    {
        ros = RosConnection.GetOrCreateInstance();
        ros.Initialize(rosIPAddress, rosPort);

        Debug.Log($"ROS Connection initialized to {rosIPAddress}:{rosPort}");
    }

    void SetupEnvironment()
    {
        // Apply materials to obstacles
        foreach (GameObject obstacle in obstacles)
        {
            if (obstacle.GetComponent<Renderer>() != null && environmentMaterials.Length > 0)
            {
                int materialIndex = Random.Range(0, environmentMaterials.Length);
                obstacle.GetComponent<Renderer>().material = environmentMaterials[materialIndex];
            }
        }

        Debug.Log("Environment setup completed");
    }

    void OnValidate()
    {
        // Validate settings in editor
        if (rosPort < 1024 || rosPort > 65535)
        {
            Debug.LogWarning("Port number should be between 1024 and 65535");
        }
    }
}