---
sidebar_position: 10
title: Chapter 8 | Simulating sensors LiDAR, Depth Cameras, and IMUs
---

# Chapter 8: Simulating Sensors: LiDAR, Depth Cameras, and IMUs

In this chapter, we'll explore the accurate simulation of key robotic sensors in both Gazebo and Unity environments. We'll examine how to model LiDAR, depth cameras, and IMUs with realistic noise characteristics and performance parameters that match real-world sensors.

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure and simulate LiDAR sensors with realistic parameters in both Gazebo and Unity
- Implement depth camera simulation with accurate depth perception in both environments
- Model IMU sensors with proper noise and bias characteristics in both Gazebo and Unity
- Validate sensor simulation accuracy against real-world specifications
- Integrate simulated sensors with ROS for realistic data streams

## Introduction to Sensor Simulation

Sensor simulation is a critical component of robotic simulation environments, enabling the development and testing of perception algorithms without requiring physical hardware.

### Importance of Accurate Sensor Simulation

Realistic sensor simulation is essential for:
- Developing robust perception algorithms
- Testing robot behavior in diverse conditions
- Training machine learning models with synthetic data
- Validating control systems before hardware deployment

### Sensor Simulation Challenges

- Modeling realistic noise and uncertainty
- Matching real sensor performance characteristics
- Maintaining real-time performance during simulation
- Bridging the reality gap between simulation and reality

## LiDAR Sensor Simulation

LiDAR (Light Detection and Ranging) sensors are crucial for robotics applications, providing accurate 3D environmental mapping.

### LiDAR Fundamentals

LiDAR sensors emit laser pulses and measure the time-of-flight to determine distances to objects in the environment.

#### Key Parameters
- **Range**: Maximum and minimum detection distances
- **Resolution**: Angular resolution of measurements
- **Field of View**: Horizontal and vertical coverage
- **Update Rate**: Frequency of complete scans
- **Accuracy**: Measurement precision and repeability

### LiDAR Simulation in Gazebo

#### SDF Configuration for LiDAR Sensors

```xml
<sensor name="lidar_sensor" type="ray">
  <pose>0 0 0.3 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

#### Multi-Beam LiDAR Configuration

For more advanced LiDAR sensors like Velodyne:

```xml
<sensor name="velodyne_sensor" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.2</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>true</always_on>
  <visualize>false</visualize>
</sensor>
```

### LiDAR Noise Modeling

Real LiDAR sensors have various sources of noise and error:

#### Distance Noise
- Additive Gaussian noise to distance measurements
- Range-dependent noise characteristics
- Angular-dependent noise patterns

#### Example Noise Configuration
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
</noise>
```

#### Spurious Measurements
- Modeling false returns from dust or rain
- Handling near-field limitations
- Managing multi-path interference

### LiDAR Performance Considerations

#### Computational Complexity
- Number of rays affects simulation performance
- Trade-off between accuracy and frame rate
- Optimization strategies for complex scenes

#### Occlusion Handling
- Proper handling of multiple reflections
- Near-field occlusion effects
- Transparent or semi-transparent objects

### LiDAR Simulation in Unity

#### Raycasting Implementation

```csharp
public class UnityLidarSimulation : MonoBehaviour
{
    [Header("Lidar Configuration")]
    public int horizontalRays = 360;
    public int verticalRays = 1;
    public float maxRange = 30.0f;
    public float minRange = 0.1f;
    public string rosTopic = "/scan";

    [Header("Noise Parameters")]
    public float noiseStdDev = 0.01f; // 1cm standard deviation

    private float[] ranges;
    private Vector3[] directions;

    void Start()
    {
        InitializeLidar();
    }

    void InitializeLidar()
    {
        int totalRays = horizontalRays * verticalRays;
        ranges = new float[totalRays];
        directions = new Vector3[totalRays];

        // Precompute ray directions for efficiency
        int index = 0;
        for (int v = 0; v < verticalRays; v++)
        {
            for (int h = 0; h < horizontalRays; h++)
            {
                float hAngle = (h / (float)horizontalRays) * 2 * Mathf.PI;
                float vAngle = 0; // For 2D LiDAR, or vary for 3D

                Vector3 direction = new Vector3(
                    Mathf.Cos(vAngle) * Mathf.Cos(hAngle),
                    Mathf.Cos(vAngle) * Mathf.Sin(hAngle),
                    Mathf.Sin(vAngle)
                );

                directions[index] = direction;
                index++;
            }
        }
    }

    void Update()
    {
        SimulateLidarScan();

        // Send data to ROS if connected
        if (Time.frameCount % 10 == 0) // Send every 10 frames
        {
            SendLidarDataToROS();
        }
    }

    void SimulateLidarScan()
    {
        for (int i = 0; i < ranges.Length; i++)
        {
            Vector3 worldDirection = transform.TransformDirection(directions[i]);

            RaycastHit hit;
            if (Physics.Raycast(transform.position, worldDirection, out hit, maxRange))
            {
                float distance = hit.distance;

                // Add noise to the measurement
                distance += UnityEngine.Random.Gaussian(0, noiseStdDev);

                // Ensure it's within valid range
                distance = Mathf.Clamp(distance, minRange, maxRange);

                ranges[i] = distance;
            }
            else
            {
                ranges[i] = maxRange;
            }
        }
    }

    void SendLidarDataToROS()
    {
        // Convert ranges array to ROS LaserScan message
        // Send via ROS-TCP-Connector
    }
}
```

## Depth Camera Simulation

Depth cameras provide 3D information about the environment by measuring the distance to objects in each pixel.

### Depth Camera Principles

Depth cameras can use various technologies:
- **Stereo vision**: Two cameras to compute depth via triangulation
- **Structured light**: Projecting known patterns and analyzing distortions
- **Time-of-flight**: Measuring light travel time to compute depth

### Depth Camera Simulation in Gazebo

#### RGB-D Camera Configuration

```xml
<sensor name="depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>10</updateRate>
    <cameraName>depth_camera</cameraName>
    <imageTopicName>rgb/image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>depth/points</pointCloudTopicName>
    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>depth_camera_frame</frameName>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <pointCloudCutoff>0.1</pointCloudCutoff>
    <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
    <CxPrime>0</CxPrime>
    <Cx>0</Cx>
    <Cy>0</Cy>
    <focalLength>0</focalLength>
    <hackBaseline>0</hackBaseline>
  </plugin>
</sensor>
```

### Depth Camera Noise Modeling

#### Depth Noise Characteristics
Depth cameras have specific noise patterns:

```xml
<camera>
  <depth_camera>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm depth accuracy -->
    </noise>
  </depth_camera>
</camera>
```

#### Depth-Dependent Noise
Real depth cameras often have noise that increases with distance:

- **Near range**: High accuracy, low noise
- **Mid range**: Moderate accuracy
- **Far range**: Lower accuracy, higher noise

### Point Cloud Generation

Depth cameras generate point clouds that are crucial for 3D perception:

#### Point Cloud Topics
- `/depth/points`: Unorganized point cloud
- `/depth/points_rect`: Rectified point cloud
- Data format: sensor_msgs/PointCloud2

#### Point Cloud Quality
- Resolution depends on camera resolution
- Accuracy depends on depth sensor quality
- Density varies with distance from camera

### Depth Camera Limitations Simulation

#### Near/Far Range Limitations
- Near range: Minimum distance for accurate depth
- Far range: Maximum distance with acceptable accuracy
- Handling out-of-range measurements

#### Reflective Surfaces
- Mirrors and glass can cause incorrect depth readings
- Transparent objects may not register correctly
- Highly reflective surfaces can cause artifacts

### Depth Camera Simulation in Unity

```csharp
public class UnityDepthCamera : MonoBehaviour
{
    [Header("Camera Configuration")]
    public int width = 640;
    public int height = 480;
    public float nearClip = 0.1f;
    public float farClip = 10.0f;
    public string imageTopic = "/camera/rgb/image_raw";
    public string depthTopic = "/camera/depth/image_raw";

    [Header("Noise Parameters")]
    public float depthNoiseStdDev = 0.01f;

    private Camera unityCamera;
    private RenderTexture depthTexture;
    private Texture2D depthTexture2D;

    void Start()
    {
        unityCamera = GetComponent<Camera>();

        // Create render texture for depth
        depthTexture = new RenderTexture(width, height, 24);
        unityCamera.targetTexture = depthTexture;

        // Create texture2D for reading
        depthTexture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        // Render depth information
        RenderDepthImage();

        if (Time.frameCount % 5 == 0) // Send every 5 frames
        {
            SendDepthDataToROS();
        }
    }

    void RenderDepthImage()
    {
        // Render to depth texture
        unityCamera.Render();

        // Read depth information
        RenderTexture.active = depthTexture;
        depthTexture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthTexture2D.Apply();

        // Process depth data (simplified)
        Color[] pixels = depthTexture2D.GetPixels();
        float[] depthValues = new float[pixels.Length];

        for (int i = 0; i < pixels.Length; i++)
        {
            // Extract depth information from pixel (this is a simplified approach)
            float depth = pixels[i].r; // In a real implementation, you'd use a depth shader
            depth = nearClip + depth * (farClip - nearClip);

            // Add noise
            depth += UnityEngine.Random.Gaussian(0, depthNoiseStdDev);

            depthValues[i] = Mathf.Clamp(depth, nearClip, farClip);
        }

        // Convert to point cloud or depth image as needed
    }

    void SendDepthDataToROS()
    {
        // Convert depth data to ROS format and send
    }
}
```

## IMU Sensor Simulation

Inertial Measurement Units (IMUs) provide crucial information about robot orientation, acceleration, and angular velocity.

### IMU Fundamentals

IMUs typically contain:
- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field (provides heading)

### IMU Simulation in Gazebo

#### Basic IMU Configuration

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00174533</stddev>  <!-- ~0.1 deg/s -->
          <bias_mean>0.000174533</bias_mean>  <!-- ~0.01 deg/s -->
          <bias_stddev>0.000174533</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00174533</stddev>
          <bias_mean>0.000174533</bias_mean>
          <bias_stddev>0.000174533</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00174533</stddev>
          <bias_mean>0.000174533</bias_mean>
          <bias_stddev>0.000174533</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- 1.7 mg -->
          <bias_mean>0.01</bias_mean>  <!-- 1 mg -->
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <body_name>imu_body</body_name>
    <update_rate>100</update_rate>
    <gaussian_noise>0.01</gaussian_noise>
  </plugin>
</sensor>
```

### IMU Noise Characteristics

#### Gyroscope Noise
- **White noise**: Random noise component
- **Bias instability**: Slowly varying bias
- **Scale factor error**: Scaling inaccuracies
- **Cross-axis sensitivity**: Coupling between axes

#### Accelerometer Noise
- **White noise**: Random measurement noise
- **Bias**: Constant offset in measurements
- **Scale factor**: Scaling errors
- **Non-linearity**: Non-linear response

#### Magnetometer Noise (if included)
- **Hard iron effects**: Constant magnetic field offsets
- **Soft iron effects**: Magnetic field distortions
- **Environmental interference**: External magnetic sources

### IMU Integration Challenges

#### Drift Compensation
IMUs suffer from drift over time:
- Gyroscope integration for orientation
- Accelerometer double integration for position
- Need for external reference systems

#### Calibration Simulation
- Initial bias and scale factor errors
- Temperature-dependent effects
- Mechanical mounting errors

### IMU Simulation in Unity

```csharp
public class UnityIMUSimulation : MonoBehaviour
{
    [Header("IMU Configuration")]
    public float updateRate = 100.0f; // Hz
    public string rosTopic = "/imu/data";

    [Header("Noise Parameters - Gyroscope")]
    public float gyroNoiseStdDev = 0.00174533f; // ~0.1 deg/s
    public float gyroBiasStdDev = 0.000174533f; // ~0.01 deg/s

    [Header("Noise Parameters - Accelerometer")]
    public float accelNoiseStdDev = 0.017f; // 1.7 mg
    public float accelBiasStdDev = 0.01f; // 1 mg

    private float updateInterval;
    private float lastUpdateTime;

    // IMU state with bias and noise
    private Vector3 trueAngularVelocity;
    private Vector3 trueLinearAcceleration;
    private Vector3 gyroBias;
    private Vector3 accelBias;

    void Start()
    {
        updateInterval = 1.0f / updateRate;
        lastUpdateTime = Time.time;

        // Initialize biases (these would normally drift over time)
        gyroBias = new Vector3(
            RandomGaussian(0, gyroBiasStdDev),
            RandomGaussian(0, gyroBiasStdDev),
            RandomGaussian(0, gyroBiasStdDev)
        );

        accelBias = new Vector3(
            RandomGaussian(0, accelBiasStdDev),
            RandomGaussian(0, accelBiasStdDev),
            RandomGaussian(0, accelBiasStdDev)
        );
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            SimulateIMUReading();
            SendIMUDataToROS();
            lastUpdateTime = Time.time;
        }
    }

    void SimulateIMUReading()
    {
        // Get true angular velocity from Unity's physics
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            trueAngularVelocity = rb.angularVelocity;
        }
        else
        {
            // If no rigidbody, estimate from rotation change
            trueAngularVelocity = GetEstimatedAngularVelocity();
        }

        // Get true linear acceleration
        trueLinearAcceleration = GetTrueLinearAcceleration();

        // Apply noise and bias
        Vector3 noisyAngularVelocity = trueAngularVelocity + gyroBias + new Vector3(
            RandomGaussian(0, gyroNoiseStdDev),
            RandomGaussian(0, gyroNoiseStdDev),
            RandomGaussian(0, gyroNoiseStdDev)
        );

        Vector3 noisyLinearAcceleration = trueLinearAcceleration + accelBias + new Vector3(
            RandomGaussian(0, accelNoiseStdDev),
            RandomGaussian(0, accelNoiseStdDev),
            RandomGaussian(0, accelNoiseStdDev)
        );

        // Convert to ROS IMU message format
        PublishIMUMessage(noisyAngularVelocity, noisyLinearAcceleration);
    }

    Vector3 GetEstimatedAngularVelocity()
    {
        // Estimate angular velocity from rotation changes
        // This is a simplified approach
        return Vector3.zero;
    }

    Vector3 GetTrueLinearAcceleration()
    {
        // Calculate true linear acceleration (excluding gravity)
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            // Remove gravity from the acceleration
            return rb.velocity - Physics.gravity;
        }

        // If no rigidbody, estimate from position changes
        return Vector3.zero;
    }

    float RandomGaussian(float mean, float stdDev)
    {
        // Box-Muller transform for Gaussian random numbers
        float u1 = Random.value;
        float u2 = Random.value;
        float normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        return mean + stdDev * normal;
    }

    void PublishIMUMessage(Vector3 angularVelocity, Vector3 linearAcceleration)
    {
        // Create and send ROS IMU message
        // This would use the ROS-TCP-Connector
    }

    void SendIMUDataToROS()
    {
        // Send the IMU data via ROS connection
    }
}
```

## Sensor Fusion in Simulation

Combining multiple sensor types provides more robust perception capabilities.

### Multi-Sensor Integration

#### Sensor Synchronization
- Time-stamping consistency
- Frame rate matching
- Data association challenges

#### Fusion Algorithms
- Kalman filters for state estimation
- Particle filters for non-linear systems
- Complementary filters for specific applications

### Example: Visual-Inertial Odometry

Combining camera and IMU data:

```xml
<!-- Synchronized camera and IMU for VIO in Gazebo -->
<sensor name="camera" type="camera">
  <!-- Camera configuration -->
</sensor>
<sensor name="imu" type="imu">
  <!-- IMU configuration -->
</sensor>
<!-- Fusion node configuration -->
```

## Realistic Environmental Effects

### Weather and Atmospheric Conditions

#### Rain and Snow Effects
- LiDAR attenuation in precipitation
- Camera visibility reduction
- IMU vibration effects

#### Dust and Particles
- Sensor contamination modeling
- Reduced visibility for cameras
- Potential LiDAR interference

### Dynamic Environmental Factors

#### Temperature Effects
- Sensor drift due to temperature changes
- Thermal noise variations
- Component performance changes

#### Vibration and Shock
- IMU noise increase during vibration
- Camera blur from vibrations
- Potential LiDAR measurement errors

## Sensor Validation and Calibration

### Performance Metrics

#### LiDAR Validation
- Range accuracy vs. ground truth
- Angular accuracy assessment
- Multi-target resolution capability

#### Depth Camera Validation
- Depth accuracy across range
- Field of view verification
- Point cloud density validation

#### IMU Validation
- Bias and noise characterization
- Scale factor accuracy
- Cross-axis sensitivity measurement

### Calibration Procedures

#### Intrinsic Calibration
- Camera internal parameters
- LiDAR geometric calibration
- IMU scale factor calibration

#### Extrinsic Calibration
- Sensor-to-sensor mounting relationships
- Sensor-to-robot coordinate transformations
- Temporal synchronization validation

## Performance Optimization

### Computational Efficiency

#### Sensor Update Rates
- Matching sensor rates to application needs
- Computational load balancing
- Real-time performance maintenance

#### Level of Detail
- Reducing sensor complexity when appropriate
- Adaptive sensor simulation
- Performance vs. accuracy trade-offs

### Memory Management

#### Large Data Streams
- Efficient point cloud processing
- Image buffer management
- Real-time data handling

## Integration with ROS

### Message Types and Standards

#### Standard ROS Messages
- sensor_msgs/LaserScan for LiDAR
- sensor_msgs/Image and sensor_msgs/PointCloud2 for cameras
- sensor_msgs/Imu for IMU data

#### TF Transformations
- Proper coordinate frame management
- Static and dynamic transforms
- Multi-sensor frame relationships

### Sensor Processing Pipelines

#### Standard ROS Packages
- robot_localization for state estimation
- pointcloud_to_laserscan for data conversion
- image_proc for camera processing

## Troubleshooting Common Issues

### LiDAR Issues
- Missing returns from transparent objects
- Multi-path interference artifacts
- Range accuracy problems

### Depth Camera Issues
- Incorrect depth values for reflective surfaces
- Point cloud density problems
- Calibration parameter errors

### IMU Issues
- Integration drift problems
- Coordinate frame confusion
- Noise parameter mismatches

## Chapter Summary

Simulating sensors with realistic characteristics is crucial for developing robust robotic systems. Accurate modeling of LiDAR, depth cameras, and IMUs with proper noise, bias, and environmental effects enables effective development and testing of perception and control algorithms.

The ability to simulate these sensors with realistic parameters allows developers to identify potential issues before deploying on physical hardware, saving time and resources while improving system reliability. Proper validation against real sensor characteristics ensures that algorithms developed in simulation will perform well when transferred to real robots.

Understanding the specific characteristics, limitations, and failure modes of each sensor type is essential for creating effective robotic systems. As you continue through this textbook, you'll learn how to integrate these simulated sensors with AI perception systems and use them for advanced robotics applications in the subsequent modules.

## Check Your Understanding

1. **Conceptual**: Explain the differences between simulating sensors in Gazebo versus Unity, and when you would use each approach.

2. **Application**: Design a sensor fusion system that combines data from LiDAR, depth camera, and IMU sensors. What challenges would you face in simulation?

3. **Analysis**: Why is it important to include realistic noise models in sensor simulation, and how does this affect algorithm development?

## Next Steps

In the next module, we'll explore the AI-Robot Brain using NVIDIA Isaac™, where we'll integrate the sensors we've simulated with advanced perception and control algorithms.

---

**Reflection Question**: Consider a robot operating in challenging environmental conditions (rain, fog, dust). How would you modify your sensor simulation to accurately represent the degradation in sensor performance under these conditions?