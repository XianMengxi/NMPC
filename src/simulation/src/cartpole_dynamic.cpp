#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "cartpole_dynamic.h" 
#include "ocs2_core/misc/LoadData.h"
#include "cartpole_dynamic.h"
#include "ros/ros.h"
#include "string"

ad_vector_t CartpoleDynamics::systemFlowMap(
    ad_scalar_t time,
    const ad_vector_t &state,
    const ad_vector_t &input,
    const ad_vector_t &parameters)
    const
{
    // 状态:【p \theta v[3] \omega】
    // 输入:
    // f：  【tau】
    // 输出：
    // q：  【p \theta】
    // dq： 【v[3] \omega】
    const auto &model = pinocchioInterfaceAdPtr_->getModel();
    auto &data = pinocchioInterfaceAdPtr_->getData();
    // nq是表示位置量的量的个数
    ad_vector_t q = state.head(model.nq);
    // nv是速度量的个数
    ad_vector_t dq = state.tail(model.nv);
    ad_vector_t tau = ad_vector_t::Zero(model.nv);
    tau[0] = input[0];

    ad_vector_t ddq = pinocchio::aba(model, data, q, dq, tau);
    ad_vector_t dx(state.size());
    dx << dq, ddq;

    return dx;
}

CartpoleDynamics::CartpoleDynamics(const PinocchioInterface &pinocchioInterface)
{
    ros::NodeHandle prenh;
    std::string configFile;
    prenh.getParam("config_file", configFile);
    int stateDim = 0;
    int inputDim = 0;
    std::string saveFolder;
    bool recompileFlag;

    loadData::loadCppDataType(configFile, "StateDim", stateDim);
    loadData::loadCppDataType(configFile, "InputDim", inputDim);
    loadData::loadCppDataType(configFile, "ModelCompileSettings.modelFolder", saveFolder);
    loadData::loadCppDataType(configFile, "ModelCompileSettings.recompile", recompileFlag);

    pinocchioInterfaceAdPtr_ =
        std::make_shared<ocs2::PinocchioInterfaceCppAd>(pinocchioInterface.toCppAd());
    initialize(stateDim, inputDim, "cartpole_dynamics", saveFolder, recompileFlag);
}

CartpoleDynamics *CartpoleDynamics::clone() const
{
    return new CartpoleDynamics(*this);
}
