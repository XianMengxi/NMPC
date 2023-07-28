// pinocchio库，需要fwd在前（forward declaration）
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

// 标准库
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

// OCS2的pinocchio接口，用来加载Pinocchio解析的urdf模型，转化为拉格朗日问题
// #include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>

// OCS2的ROS接口，用来解析消息、自动更新observation
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

// ocs2相关模块，oc就是optimal control的缩写
// 非线性MPC需要：
// 0.加载参数
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
// 1.建立问题
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
// 2.损失函数
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
// 3.输入限制、软约束的惩罚
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/penalties/augmented/SlacknessSquaredHingePenalty.h>
// 4.滚动优化
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
// 5.求解器
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
// 在运行过程中，需要完成变量的更新，非常重要的是Observation的更新
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
// 在MRT功能中使用到了线程管理
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

// 自己定义的动力学模型的解析
#include "cartpole_dynamic.h"

// 使用命名空间
using namespace std;
using namespace ocs2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ocs2_cartpole_node");
    //  创建pinocchio模型
    //  loadurdf
    ros::NodeHandle nh;

    // 使用ros的参数服务器加载urdf，加载求解器的配置
    std::string urdfFile;
    std::string configFile;

    // 把参数服务器里面名为urdf_file的变量加载到urdfFile字符串内，配置文件同理
    nh.getParam("urdf_file", urdfFile);
    nh.getParam("config_file", configFile);

    // 使用config文件加载参数，loadDataType第一个参数是配置文件名，第二个是key，第三个是加载到的变量名称
    int stateDim, inputDim;
    double maxForce;
    loadData::loadCppDataType(configFile, "StateDim", stateDim);
    loadData::loadCppDataType(configFile, "InputDim", inputDim);
    loadData::loadCppDataType(configFile, "MaxForce", maxForce);

    // Q、R、finalQ矩阵的空间分配
    matrix_t Q = matrix_t::Identity(stateDim, stateDim);
    matrix_t R = matrix_t::Identity(inputDim, inputDim);
    matrix_t finalQ = matrix_t::Identity(stateDim, stateDim);

    // 系数矩阵加载
    loadData::loadEigenMatrix(configFile, "Q", Q);
    loadData::loadEigenMatrix(configFile, "R", R);
    loadData::loadEigenMatrix(configFile, "Q_final", finalQ);

    // pinocchio模型，把一个urdf文件内容以字符串形式给到Model变量，转换模型，生成它对应的Pinocchio接口
    pinocchio::Model cartPoleModel;
    pinocchio::urdf::buildModel(urdfFile, cartPoleModel);
    PinocchioInterface pinocchioInterface(cartPoleModel);

    // 优化控制问题
    OptimalControlProblem ocp;

    //  1.系统动力学模型
    CartpoleDynamics cartpoleDynamics(pinocchioInterface);
    ocp.dynamicsPtr = std::make_unique<CartpoleDynamics>(pinocchioInterface);

    //  2.优化目标函数的定义损失函数有三个模块：状态/输入的损失、最终状态损失
    // 添加状态/输入的损失
    ocp.costPtr->add("cartpoleStateInputCost", std::make_unique<QuadraticStateInputCost>(Q, R));
    // 添加终端损失
    ocp.finalCostPtr->add("cartpoleStateFinalCost", std::make_unique<QuadraticStateCost>(finalQ));

    auto getConstraint = [&]()
    {
        int constraintDim = 2;
        matrix_t C = matrix_t::Zero(constraintDim, stateDim);
        vector_t D = vector_t::Zero(constraintDim);
        D << 1, -1;
        vector_t e = vector_t::Zero(constraintDim);
        std::cout << "MaxForce:" << maxForce << "\n";
        e << maxForce, maxForce;
        return std::unique_ptr<StateInputConstraint>(new LinearStateInputConstraint(e, C, D));
    };

    // 添加输入损失，由于是软约束的，所以采用的是SlacknessSquaredHingePenalty
    auto getPenalty = [&]()
    {
        scalar_t scale = 0;
        scalar_t stepSize = 0;
        loadData::loadCppDataType(configFile, "MaxForceConstraintPenalty.scale", scale);
        loadData::loadCppDataType(configFile, "MaxForceConstraintPenalty.stepSize", stepSize);
        // 参数已经成功加载
        std::cout << "scale:" << scale << "\n";
        std::cout << "setpSize:" << stepSize << "\n";
        augmented::SlacknessSquaredHingePenalty::Config config(scale, stepSize);
        augmented::SlacknessSquaredHingePenalty penalty(config);
        return std::make_unique<augmented::SlacknessSquaredHingePenalty>(config);
    };

    ocp.inequalityLagrangianPtr->add("inputMaxConstraint", create(getConstraint(), getPenalty()));

    // 完成求解mpc的设置
    auto rolloutSettings = rollout::loadSettings(configFile, "rollout");
    TimeTriggeredRollout rollout(*ocp.dynamicsPtr, rolloutSettings);
    auto ddpSettings = ddp::loadSettings(configFile, "ddp");
    auto mpcSettings = mpc::loadSettings(configFile, "mpc");
    DefaultInitializer initializer(inputDim);
    GaussNewtonDDP_MPC mpc(mpcSettings, ddpSettings, rollout, ocp, initializer);

    vector_t initState = vector_t::Zero(stateDim);
    vector_t finalState = vector_t::Zero(stateDim);

    // 初始结尾状态成功加载
    loadData::loadEigenMatrix(configFile, "InitState", initState);
    loadData::loadEigenMatrix(configFile, "FinalState", finalState);

    vector_t initInput = vector_t::Zero(inputDim);

    TargetTrajectories targetTrajectories({0, 1},
    {initState, finalState},
    {initInput, initInput});

    auto referenceManager = std::make_shared<ReferenceManager>(targetTrajectories);

    auto rosReferenceManager = std::make_shared<RosReferenceManager>("cartpole", referenceManager);
    // mpc设置参考管理器为ros通信的
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManager);

    // 状态观测器，我们做的非常多的操作全都是为了它服务的
    SystemObservation observation;
    // 如果时间取得为0证明计时器初始化没开始，非0计时开始
    while (ros::Time::now().toSec() == 0)
        ;
    auto startTime = ros::Time::now();
    // 订阅来自频道"/cartpole/joint_states"的JointState消息，这个消息存在msg内，解析他，赋值给observation，已经赋值成功了，
    auto jointStateSub = nh.subscribe<sensor_msgs::JointState>("/cartpole/joint_states", 1, [&](const sensor_msgs::JointState::ConstPtr &msg)
    {
        observation.time = ros::Time::now().toSec() - startTime.toSec();
        observation.state = vector_t::Zero(stateDim);
        // observation.state << msg->position[0],
        //     msg->position[1],
        //     msg->position[2],
        //     msg->position[3],
        //     msg->velocity[0],
        //     msg->velocity[1],
        //     msg->velocity[2],
        //     msg->velocity[3];
        observation.input = vector_t::Zero(inputDim);
    });

    // 向仿真环境发布控制指令
    auto commandPub = nh.advertise<std_msgs::Float64>("/cartpole/cart_effort_controller/command", 1);
    
    // 向python命令发布节点发布当前的状态
    auto observationPub = nh.advertise<ocs2_msgs::mpc_observation>("/cartpole/mpc_observer", 1);
    rosReferenceManager->subscribe(nh);

    // 使用MRT管理:MRT首次出现
    MPC_MRT_Interface mpcMrtInterface(mpc);
    bool mpcRunning = false;
    bool controllerRunning = true;
    auto mpcThread = std::thread([&]()
    {
        while(ros::ok()&&controllerRunning){
            if(mpcRunning){
                executeAndSleep([&]()
                    { 
                        // cout<<"interface推mpc了"<<endl;
                        mpcMrtInterface.advanceMpc(); },
                        mpcSettings.mpcDesiredFrequency_);
                        }   
                    }
    });

    auto runningCheck = ros::Rate(100);
    while (observation.time == 0)
    {
        runningCheck.sleep();
        ros::spinOnce();
    }

    mpcMrtInterface.setCurrentObservation(observation);
    mpcRunning=true;

    // 等待第一次MPC轨迹的算出。
    while (!mpcMrtInterface.initialPolicyReceived())
    {
        cout<<"卡在第一次算mpc轨迹这里了"<<endl;
        runningCheck.sleep();
    }

    cout << "MPC initial policy recieved:\n"
        << observation << "\n"
        << endl;

    // 打印轨迹，只打印了第一次的轨迹，所以应该来说，是observation在主函数接到了第一个指令后算出轨迹的那么一下
    // 另外单独开一个线程，按照更高的频率发布运动指令command，command依序输出单次mpc的inputTrajectory
    // 在mpc完整地算完一次后，command再从inputTrajectory的第一位开始取。
    mpcMrtInterface.updatePolicy();
    auto currentPolicy = mpcMrtInterface.getPolicy();
    for (int i = 0; i < currentPolicy.stateTrajectory_.size(); i++)
    {
        cout << "时间序列:\n";
        cout << currentPolicy.timeTrajectory_[i] << endl;
        cout << "状态轨迹:\n";
        cout << currentPolicy.stateTrajectory_[i] << endl;
        cout << "输入轨迹:\n";
        cout << currentPolicy.inputTrajectory_[i] << endl;
    }

    ros::Rate rate(500);
    while (ros::ok())
    {
        ros::spinOnce();
        mpcMrtInterface.setCurrentObservation(observation);

        auto mpcMrtPolicy = mpcMrtInterface.updatePolicy();
        SystemObservation optimizedObservation;
        mpcMrtInterface.evaluatePolicy(observation.time,
            observation.state,
            optimizedObservation.state,
            optimizedObservation.input,
            optimizedObservation.mode);
        std_msgs::Float64 msg;

        auto input = optimizedObservation.input[0];
        msg.data = input;

        // 发布观测量
        commandPub.publish(msg);
        auto observation_msg = ros_msg_conversions::createObservationMsg(observation);
        observationPub.publish(observation_msg);
        rate.sleep();
    }
    return 0;
}