#pragma once

#include "ocs2_core/dynamics/SystemDynamicsBaseAD.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"

using namespace ocs2;

class CartpoleDynamics : public ocs2::SystemDynamicsBaseAD
{
private:
    CartpoleDynamics(const CartpoleDynamics &rhs) : SystemDynamicsBaseAD(rhs){};
    std::shared_ptr<ocs2::PinocchioInterfaceCppAd> pinocchioInterfaceAdPtr_;

public:
    CartpoleDynamics(const PinocchioInterface &pinocchioInterface);

    CartpoleDynamics *clone() const override;

    // x_dot = Ax+Bu使用到
    virtual ad_vector_t systemFlowMap(
        ad_scalar_t time,
        const ad_vector_t &state,
        const ad_vector_t &input,
        const ad_vector_t &parameters) const override;

    ~CartpoleDynamics() = default;
};