#include "kin_wbc.h"

KinWBC::KinWBC(int dof) : dof_(dof) {}

void KinWBC::computeKinematicWBC(
    const std::vector<std::vector<TaskInfo>>& task_hierarchy,
    const std::map<std::string, Eigen::Vector3d>& x_desired, const std::map<std::string, Eigen::Vector3d>& dx_desired, const std::map<std::string, Eigen::Vector3d>& ddx_desired,
    const std::map<std::string, Eigen::Matrix3d>& R_desired, const std::map<std::string, Eigen::Vector3d>& w_desired, const std::map<std::string, Eigen::Vector3d>& dw_desired,
    const std::map<std::string, Eigen::Vector3d>& base_ee_pos, const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
    const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_v, const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_w,
    const Eigen::MatrixXd& base_contact_Jac,
    const Eigen::MatrixVVd& M,
    Eigen::VectorXd& dq_des, Eigen::VectorXd& qdot_des, Eigen::VectorXd& qddot_des)
{
    dq_des    = Eigen::VectorXd::Zero(dof_);
    qdot_des  = Eigen::VectorXd::Zero(dof_);
    qddot_des = Eigen::VectorXd::Zero(dof_);

    Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(dof_, dof_) - DyrosMath::pinv_SVD(base_contact_Jac) * base_contact_Jac;

    int i = 0;
    for (const auto& task_group : task_hierarchy)
    {
        int m = 3 * task_group.size();
        Eigen::MatrixXd J(m, dof_);
        Eigen::VectorXd e(m), de(m), dde(m);

        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [name, type] = task_group[i];
            if (type == TaskType::Position)
            {
                J.block(3 * i, 0, 3, dof_) = base_Jac_v.at(name);
                e.segment<3>(3 * i)   = x_desired.at(name) - base_ee_pos.at(name);
                de.segment<3>(3 * i)  = dx_desired.at(name);
                dde.segment<3>(3 * i) = ddx_desired.at(name);
            }
            else
            {
                J.block(3 * i, 0, 3, dof_) = base_Jac_w.at(name);  
                e.segment<3>(3 * i)   = orientationError(base_ee_rot.at(name), R_desired.at(name));
                de.segment<3>(3 * i)  = w_desired.at(name);
                dde.segment<3>(3 * i) = dw_desired.at(name);
            }
        }

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);
        Eigen::MatrixXd J_dyn_pinv = M.inverse() * J.transpose() * ((J * M.inverse() * J.transpose()).inverse());

        dq_des    += J_pinv     * (e   - J * dq_des);
        qdot_des  += J_pinv     * (de  - J * qdot_des);
        qddot_des += J_dyn_pinv * (dde - J * qddot_des);

        Ni *= (Eigen::MatrixXd::Identity(dof_, dof_) - J_pinv * J_pre);
        i++;
    }

}

//--- utils
Eigen::Vector3d KinWBC::orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& R_des)
{
    Eigen::Matrix3d R_err = R_des.transpose() * R;
    Eigen::Matrix3d logR = R_err.log();
    Eigen::Vector3d rotvec;
    rotvec(0) = logR(2,1);   // vee(logR)
    rotvec(1) = logR(0,2); 
    rotvec(2) = logR(1,0);  

    return rotvec;
}

Eigen::MatrixXd KinWBC::pinv_SVD(const Eigen::MatrixXd& A, double tolerance)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& S = svd.singularValues();
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    for (int i = 0; i < S.size(); ++i)
    {
        if (S(i) > tolerance)
        {
            S_inv(i, i) = 1.0 / S(i);
        }
    }

    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}