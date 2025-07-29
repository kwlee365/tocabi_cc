#include "kin_wbc.h"

KinWBC::KinWBC(int dof) : dof_(dof) {}

void KinWBC::computeTaskSpaceKinematicWBC(
    const std::vector<std::vector<TaskInfo>>& task_hierarchy,
    const std::map<std::string, Eigen::Vector3d>& x_desired, const std::map<std::string, Eigen::Vector3d>& dx_desired, const std::map<std::string, Eigen::Vector3d>& ddx_desired,
    const std::map<std::string, Eigen::Matrix3d>& R_desired, const std::map<std::string, Eigen::Vector3d>& w_desired, const std::map<std::string, Eigen::Vector3d>& dw_desired,
    const std::map<std::string, Eigen::Vector3d>& task_Kp, const std::map<std::string, Eigen::Vector3d>& task_Kv, 
    const std::map<std::string, Eigen::Vector3d>& base_ee_pos, const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
    const std::map<std::string, Eigen::Vector3d>& base_ee_v, const std::map<std::string, Eigen::Vector3d>& base_ee_w, 
    const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_v, const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_w,
    const Eigen::MatrixXd& base_contact_Jac,
    const Eigen::MatrixVVd& M_inv, 
    const Eigen::VectorVQd& qdot,
    Eigen::VectorVQd& dq_des, Eigen::VectorVQd& qdot_des, Eigen::VectorVQd& qddot_des)
{
    //--- Initialization
    dq_des    = Eigen::VectorVQd::Zero();
    qdot_des  = Eigen::VectorVQd::Zero();
    qddot_des = Eigen::VectorVQd::Zero();

    Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(dof_, dof_) - DyrosMath::pinv_SVD(base_contact_Jac) * base_contact_Jac;
    Eigen::MatrixXd Jc_dyn_pinv = M_inv * base_contact_Jac.transpose() * ((base_contact_Jac * M_inv * base_contact_Jac.transpose()).inverse());
    qddot_des = Jc_dyn_pinv * (-base_contact_Jac * qdot);  

    for (const auto& task_group : task_hierarchy)
    {
        int m = 3 * task_group.size();
        Eigen::MatrixXd J(m, dof_);
        Eigen::MatrixXd Jdot(m, dof_);
        Eigen::VectorXd e(m), de(m), dde(m);

        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [name, type] = task_group[i];
            if (type == TaskType::Position)
            {
                Eigen::Vector3d Kp_vec = task_Kp.at(name); 
                Eigen::Vector3d Kv_vec = task_Kv.at(name);

                J.block(3 * i, 0, 3, dof_) = base_Jac_v.at(name);
                Eigen::Vector3d pos_err = x_desired.at(name)  - base_ee_pos.at(name);
                Eigen::Vector3d vel_err = dx_desired.at(name) - base_ee_v.at(name); 

                e.segment<3>(3 * i)  = pos_err;
                // de.segment<3>(3 * i) = dx_desired.at(name);
                de.segment<3>(3 * i) = dx_desired.at(name) + pos_err;
                dde.segment<3>(3 * i) = ddx_desired.at(name) + Kp_vec.asDiagonal() * pos_err + Kv_vec.asDiagonal() * vel_err;
            }
            else if (type == TaskType::Orientation)
            {
                Eigen::Vector3d Kp_vec = task_Kp.at(name);
                Eigen::Vector3d Kv_vec = task_Kv.at(name);
                
                J.block(3 * i, 0, 3, dof_) = base_Jac_w.at(name);
                Eigen::Vector3d ori_err = -DyrosMath::getPhi(base_ee_rot.at(name), R_desired.at(name));
                Eigen::Vector3d vel_err = (w_desired.at(name) - base_ee_w.at(name)); 

                e.segment<3>(3 * i)   = ori_err;
                // de.segment<3>(3 * i)  = w_desired.at(name);
                de.segment<3>(3 * i)  = w_desired.at(name) + ori_err;
                dde.segment<3>(3 * i) = dw_desired.at(name) + Kp_vec.asDiagonal() * ori_err + Kv_vec.asDiagonal() * vel_err;
            }
            else
            {
                ROS_ERROR("Unknown TaskType");
                assert(type == TaskType::Position || type == TaskType::Orientation);
            }
        }

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);
        Eigen::MatrixXd J_dyn_pinv = M_inv * J.transpose() * ((J * M_inv * J.transpose()).inverse());

        dq_des    += J_pinv     * (e   - J * dq_des);
        qdot_des  += J_pinv     * (de  - J * qdot_des);
        qddot_des += J_dyn_pinv * (dde - J * qddot_des); 
        
        Ni *= (Eigen::MatrixXd::Identity(dof_, dof_) - J_pinv * J_pre);
    }
}