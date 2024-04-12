using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Gen3Moveit;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

public class TrajectoryPlanner : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // 硬编码变量 
    private int numRobotJoints = 7;
    private readonly float jointAssignmentWait = 0.06f;
    private readonly float poseAssignmentWait = 0.5f;
    private readonly float gripperAngle = 15f;
    // 偏移以确保夹具位于抓取点上方
    private readonly Vector3 pickPoseOffset = new Vector3(0f, 0.08f-0.77f, 0);//需要减去桌子高度
    private readonly Vector3 placePoseOffset = new Vector3(0f, 0.12f-0.77f, 0);
    // 乘数对应于每个关节的 URDF 模拟标签
    private float[] multipliers = new float[] { -1f, -1f, -1f, -1f, 1f, 1f, 1f };
    // 本示例的方向是硬编码的，因此夹具始终位于放置对象的正上方
    private readonly Quaternion pickOrientation = new Quaternion(-1.00f, 0f, 0f, 0f);

    // ROS通信所需的变量
    public string rosServiceName = "gen3_moveit";
    private const int isBigEndian = 0;
    private const int step = 4;

    public GameObject robot;
    public GameObject target;
    public Transform goal;

    // 关节机构
    private ArticulationBody[] jointArticulationBodies;
    ArticulationBody[] articulationChain;
    private List<ArticulationBody> gripperJoints;

    // 用户界面元素
    private Button InitializeButton;
    private Button RandomizeButton;
    private Button ServiceButton;
    private Text ActualPos;
    private Text ActualRot;
    private Text EstimatedPos;
    private Text EstimatedRot;

    private RenderTexture renderTexture;

    public PoseEstimationScenario scenario;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place,
        PostPlace
    };

    /// <summary>
    ///     根据抓取角度打开和关闭附加的抓取工具.
    /// </summary>
    /// <param name="toClose"></param>
    /// <returns></returns>
    public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle : 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            gripperJoints[i].xDrive = curXDrive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }

    /// <summary>
    ///     用于将机器人设置为默认位置的按钮回调
    /// </summary>
    public void Initialize()
    {
        StartCoroutine(MoveToInitialPosition());
    }

    /// <summary>
    ///     立方体随机化的按钮回调
    /// </summary>
    public void RandomizeCube()
    {
        scenario.Move();
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
    }

    /// <summary>
    ///     姿势估计的按钮回调
    /// </summary>
    public void PoseEstimation()
    {
        Debug.Log("Capturing screenshot...");

        InitializeButton.interactable = false;
        RandomizeButton.interactable = false;
        ServiceButton.interactable = false;
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
        EstimatedPos.text = "-";
        EstimatedRot.text = "-";

        // 捕获屏幕截图并将其传递给姿势估计服务
        byte[] rawImageData = CaptureScreenshot();
        InvokePoseEstimationService(rawImageData);
    }

    private IEnumerator MoveToInitialPosition()
    {
        bool isRotationFinished = false;
        while (!isRotationFinished)
        {
            isRotationFinished = ResetRobotToDefaultPosition();
            yield return new WaitForSeconds(jointAssignmentWait);
        }
        ServiceButton.interactable = true;
    }

    private bool ResetRobotToDefaultPosition()
    {
        bool isRotationFinished = true;
        var rotationSpeed = 180f;

        for (int i = 0; i < numRobotJoints; i++)
        {
            var tempXDrive = jointArticulationBodies[i].xDrive;
            float currentRotation = tempXDrive.target;

            float rotationChange = rotationSpeed * Time.fixedDeltaTime;

            if (currentRotation > 0f) rotationChange *= -1;

            if (Mathf.Abs(currentRotation) < rotationChange)
                rotationChange = 0;
            else
                isRotationFinished = false;

            // 新的 xDrive 目标是 currentRotation 与所需更改的总和
            float rotationGoal = currentRotation + rotationChange;
            tempXDrive.target = rotationGoal;
            jointArticulationBodies[i].xDrive = tempXDrive;
        }
        return isRotationFinished;
    }

    /// <summary>
    ///     使用捕获的屏幕截图作为字节创建一个新的 PoseEstimationServiceRequest 并实例化sensor_msgs/image.
    ///
    ///     使用 ROSConnection 调用 PoseEstimationService，并在 PoseEstimationServiceResponse 上调用 PoseEstimationCallback
    /// </summary>
    /// <param name="imageData"></param>
    private void InvokePoseEstimationService(byte[] imageData)
    {
        uint imageHeight = (uint)renderTexture.height;
        uint imageWidth = (uint)renderTexture.width;

        RosMessageTypes.Sensor.Image rosImage = new RosMessageTypes.Sensor.Image(new RosMessageTypes.Std.Header(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
        PoseEstimationServiceRequest poseServiceRequest = new PoseEstimationServiceRequest(rosImage);
        ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);
    }

    void PoseEstimationCallback(PoseEstimationServiceResponse response)
    {
        if (response != null)
        {
            // 模型输出的位置是立方体相对于相机的位置，因此我们需要提取其全局位置 
            var estimatedPosition = Camera.main.transform.TransformPoint(response.estimated_pose.position.From<RUF>());
            var estimatedRotation = Camera.main.transform.rotation * response.estimated_pose.orientation.From<RUF>();

            PublishJoints(estimatedPosition, estimatedRotation);

            EstimatedPos.text = estimatedPosition.ToString();
            EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
        }
        else
        {
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
        }
    }

    /// <summary>
    ///     捕获主相机的渲染纹理并转换为字节.
    /// </summary>
    /// <returns>imageBytes</returns>
    private byte[] CaptureScreenshot()
    {
        Camera.main.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        Camera.main.Render();
        Texture2D mainCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
        mainCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        mainCameraTexture.Apply();
        RenderTexture.active = currentRT;
        //从屏幕截图中获取原始字节信息
        byte[] imageBytes = mainCameraTexture.GetRawTextureData();
        Camera.main.targetTexture = null;
        return imageBytes;
    }

    /// <summary>
    ///     获取机器人关节角度的当前值.
    /// </summary>
    /// <returns>Gen3MoveitJoints</returns>
    Gen3MoveitJoints CurrentJointConfig()
    {
        Gen3MoveitJoints joints = new Gen3MoveitJoints();

        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;
        joints.joint_06 = jointArticulationBodies[6].xDrive.target;
        return joints;
    }

    public void PublishJoints(Vector3 targetPos, Quaternion targetRot)
    {
        MoverServiceRequest request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (targetPos + pickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(-180, targetRot.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (goal.position + placePoseOffset).To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };

        ros.SendServiceMessage<MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories != null && response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
            ServiceButton.interactable = true;
        }
    }

    /// <summary>
    ///     执行从MoverService返回的轨迹.
    ///
    ///     期望 MoverService 将返回四个轨迹计划：PreGrasp、Grasp、PickUp 和 Place,
    ///     其中每个计划都是机器人姿势的数组。机器人位姿是机器人六个关节的关节角度值.
    ///
    ///     执行单个轨迹将迭代阵列中的每个机器人姿势，同时更新机器人上的关节.
    /// 
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from gen3_moveit mover service running in ROS</param>
    /// <returns></returns>
    private IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // 对于每个返回的轨迹计划
            for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // 对于轨迹计划中的每个机器人姿势
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // 设置每个关节的关节值
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    // 等待机器人完成所有关节任务的姿势
                    yield return new WaitForSeconds(jointAssignmentWait);
                }

                // 如果完成执行抓取姿势的轨迹，则关闭夹具
                if (poseIndex == (int)Poses.Grasp)
                {
                    StartCoroutine(IterateToGrip(true));
                    yield return new WaitForSeconds(jointAssignmentWait);
                }
                else if (poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(poseAssignmentWait);
                    // 打开夹具以放置目标立方体
                    StartCoroutine(IterateToGrip(false));
                }
                // 等待机器人通过关节分配实现最终姿势
                yield return new WaitForSeconds(poseAssignmentWait);
            }

            // 重新启用按钮
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
            yield return new WaitForSeconds(jointAssignmentWait);
        }
    }

    /// <summary>
    ///     在Awake()中查找所有机器人关节并将其添加到jointArticulationBodies数组中.
    ///     找到所有夹具关节并将它们分配给各自的关节体对象.
    /// </summary>
    void Awake()
    {
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = robot.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string arm_link = shoulder_link + "/half_arm_1_link";
        jointArticulationBodies[1] = robot.transform.Find(arm_link).GetComponent<ArticulationBody>();

        string arm_link1 = arm_link + "/half_arm_2_link";
        jointArticulationBodies[2] = robot.transform.Find(arm_link1).GetComponent<ArticulationBody>();


        string elbow_link = arm_link1 + "/forearm_link";
        jointArticulationBodies[3] = robot.transform.Find(elbow_link).GetComponent<ArticulationBody>();

        string forearm_link = elbow_link + "/spherical_wrist_1_link";
        jointArticulationBodies[4] = robot.transform.Find(forearm_link).GetComponent<ArticulationBody>();

        string wrist_link = forearm_link + "/spherical_wrist_2_link";
        jointArticulationBodies[5] = robot.transform.Find(wrist_link).GetComponent<ArticulationBody>();

        string hand_link = wrist_link + "/bracelet_link";
        jointArticulationBodies[6] = robot.transform.Find(hand_link).GetComponent<ArticulationBody>();

        articulationChain = robot.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();

        var gripperJointNames = new string[] { "right_outer_knuckle", "right_inner_finger", "right_inner_knuckle", "left_outer_knuckle", "left_inner_finger", "left_inner_knuckle" };
        gripperJoints = new List<ArticulationBody>();

        foreach (ArticulationBody articulationBody in robot.GetComponentsInChildren<ArticulationBody>())
        {
            if (gripperJointNames.Contains(articulationBody.name))
            {
                gripperJoints.Add(articulationBody);
            }
        }
    }

    void Start()
    {
        // 获取ROS连接静态实例
        ros = ROSConnection.instance;

        // 分配 UI 元素
        InitializeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/DefaultButton").GetComponent<Button>();
        RandomizeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/RandomButton").GetComponent<Button>();
        ServiceButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/ServiceButton").GetComponent<Button>();

        ActualPos = GameObject.Find("ROSObjects/Canvas/PositionPanel/ActualPosField").GetComponent<Text>();
        ActualRot = GameObject.Find("ROSObjects/Canvas/PositionPanel/ActualRotField").GetComponent<Text>();
        EstimatedPos = GameObject.Find("ROSObjects/Canvas/PositionPanel/EstPosField").GetComponent<Text>();
        EstimatedRot = GameObject.Find("ROSObjects/Canvas/PositionPanel/EstRotField").GetComponent<Text>();

        // 初始化 UI 元素值
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
        EstimatedPos.text = "-";
        EstimatedRot.text = "-";

        // 渲染纹理 
        renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
    }
}