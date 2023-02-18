#include "Labs/4-Animation/tasks.h"
#include "CustomFunc.inl"
#include "IKSystem.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <spdlog/spdlog.h>

#define pi acos(-1)
namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) { // root
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }

        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics
            glm::vec4 pos             = glm::vec4(ik.JointLocalOffset[i], 1.0f);
            pos                       = glm::mat4_cast(ik.JointGlobalRotation[i - 1]) * pos;
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + glm::vec3(pos.x, pos.y, pos.z) / pos.w; //父关节的全局位置加上父关节方向上偏移量

            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        int CCDIKIteration = 0;
        for (CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int end_index = ik.JointLocalOffset.size() - 1;
            for (int i = end_index - 1; i >= 0; i--) {
                glm::vec3 orig           = ik.JointGlobalPosition[end_index] - ik.JointGlobalPosition[i]; //自身轴点到尾叶子节点的方向
                orig                     = glm::normalize(orig);
                glm::vec3 dest           = EndPosition - ik.JointGlobalPosition[i]; //自身轴点到目标点方向
                dest                     = glm::normalize(dest);
                glm::quat change         = glm::rotation(orig, dest);
                ik.JointLocalRotation[i] = change * ik.JointLocalRotation[i]; //转
                ForwardKinematics(ik, i);                                     //从i关节开始往后的关节调整一下
            }
        }
        // printf("maxCCDIKIteration is %d,CCDIKIteration is %d\n", maxCCDIKIteration, CCDIKIteration);
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int                    nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        int                    IKIteration = 0;
        for (IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 dir         = glm::normalize(ik.JointGlobalPosition[i] - backward_positions[i + 1]);
                backward_positions[i] = backward_positions[i + 1] + dir * ik.JointOffsetLength[i + 1];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0]   = ik.JointGlobalPosition[0];
            for (int i = 1; i < nJoints; i++) {
                // your code here
                glm::vec3 dir        = glm::normalize(backward_positions[i] - forward_positions[i - 1]);
                forward_positions[i] = forward_positions[i - 1] + dir * ik.JointOffsetLength[i];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }
        // printf("maxFABRIKIteration is %d,IKIteration is %d\n", maxFABRIKIteration, IKIteration);

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        /* int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        }
        custom->resize(index);
        return custom;*/
        int nums      = 100;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(9 * nums));
        int                      index = 0;
        // Q
        for (int i = 0; i < nums; ++i) {
            float x_val        = cos(2 * pi * i / nums);
            float y_val        = sin(2 * pi * i / nums);
            (*custom)[index++] = glm::vec3(0.4 + 0.4 * x_val, 0.0f, 0.4 + 0.4 * y_val);
        }

        for (int i = 0; i < nums; i++) {
            float x_val        = 0.4 * i / nums;
            float y_val        = 0.4 * i / nums;
            (*custom)[index++] = glm::vec3(0.4 + x_val, 0.0f, 0.4 + y_val);
        }
        // D
        for (int i = 0; i < nums; i++) {
            float x_val        = 1.0;
            float y_val        = 0.8 * i / nums;
            (*custom)[index++] = glm::vec3(x_val, 0.0f, y_val);
        }

        for (int i = 0; i < nums; ++i) {
            float x_val = sin(2 * pi * i / nums);
            float y_val = cos(2 * pi * i / nums);
            if (x_val < 0)
                continue;
            (*custom)[index++] = glm::vec3(1.0 + 0.4 * x_val, 0.0f, 0.4 + 0.4 * y_val);
        }

        custom->resize(index);
        return custom;
    }

    /*---------------------------------------------Task 2------------------------------------------------------------*/
    // VectorXf是任意维浮点向量
    float g(const MassSpringSystem & system, const Eigen::VectorXf & x, const Eigen::VectorXf & y, float h) {
        auto  x_y2  = (x - y).transpose() * system.Mass * (x - y); //|x-y|^2
        float g_val = x_y2[0] / (2.0f * h * h);
        //+1/2kx^2
        for (auto const spring : system.Springs) { //对每根弹簧
            auto const      p0   = spring.AdjIdx.first;
            auto const      p1   = spring.AdjIdx.second;
            glm::vec3       pos1 = glm::vec3(x[p1 * 3 + 0], x[p1 * 3 + 1], x[p1 * 3 + 2]);
            glm::vec3       pos0 = glm::vec3(x[p0 * 3 + 0], x[p0 * 3 + 1], x[p0 * 3 + 2]);
            glm::vec3 const x01  = pos1 - pos0;
            float           E    = 1.0f / 2.0f * system.Stiffness * (glm::length(x01) - spring.RestLength) * (glm::length(x01) - spring.RestLength); // E=1/2kx^2
            g_val += E;
        }
        return g_val;
    }

    Eigen::VectorXf nabla_g(const MassSpringSystem & system, const Eigen::VectorXf & x, const Eigen::VectorXf & y, float h) {
        Eigen::VectorXf result = (x - y) * system.Mass / (h * h); //(x-y)/h^2
        //+kx
        for (auto const spring : system.Springs) {
            auto const      p0   = spring.AdjIdx.first;
            auto const      p1   = spring.AdjIdx.second;
            glm::vec3       pos1 = glm::vec3(x[p1 * 3], x[p1 * 3 + 1], x[p1 * 3 + 2]);
            glm::vec3       pos0 = glm::vec3(x[p0 * 3], x[p0 * 3 + 1], x[p0 * 3 + 2]);
            glm::vec3 const x01  = pos1 - pos0;
            glm::vec3 const e01  = glm::normalize(x01);                                             //单位向量
            glm::vec3       f    = system.Stiffness * (glm::length(x01) - spring.RestLength) * e01; // f=kx

            for (int i = 0; i < 3; i++) {
                result[p0 * 3 + i] -= f[i];
                result[p1 * 3 + i] += f[i];
            }
        }
        return result;
    }

    //黑塞矩阵
    void nabla2_g(Eigen::SparseMatrix<float> & res, const MassSpringSystem & system, const Eigen::VectorXf & grad_g, const Eigen::VectorXf & x, const Eigen::VectorXf & y, float h) {
        std::vector<Eigen::Triplet<float>> coefficients;
        int                                n = system.Positions.size();

        for (int i = 0; i < 3 * n; i++)
            coefficients.push_back(Eigen::Triplet(i, i, system.Mass / (h * h)));

        for (auto const spring : system.Springs) {
            int p0 = spring.AdjIdx.first;
            int p1 = spring.AdjIdx.second;

            glm::vec3       pos1 = glm::vec3(x[p1 * 3], x[p1 * 3 + 1], x[p1 * 3 + 2]);
            glm::vec3       pos0 = glm::vec3(x[p0 * 3], x[p0 * 3 + 1], x[p0 * 3 + 2]);
            glm::vec3 const x01  = pos1 - pos0;

            float     dis = glm::length(x01);
            glm::mat3 H(0);
            for (int i = 0; i < 3; i++)
                H[i][i] = system.Stiffness * (1.0f - spring.RestLength / dis);
            float dis3 = dis * dis * dis;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    H[i][j] += system.Stiffness * spring.RestLength * x01[i] * x01[j] / dis3;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    if (glm::abs(H[i][j]) > 1e-5f) {
                        coefficients.push_back(Eigen::Triplet(p0 * 3 + i, p1 * 3 + j, -H[i][j]));
                        coefficients.push_back(Eigen::Triplet(p1 * 3 + j, p0 * 3 + i, -H[j][i]));
                        coefficients.push_back(Eigen::Triplet(p0 * 3 + i, p0 * 3 + j, H[i][j]));
                        coefficients.push_back(Eigen::Triplet(p1 * 3 + j, p1 * 3 + i, H[j][i]));
                    }
        }
        res.setFromTriplets(coefficients.begin(), coefficients.end());
    }

    //计算阻尼
    Eigen::VectorXf Damping_F(const MassSpringSystem & system, const Eigen::VectorXf & x, const Eigen::VectorXf & v) {
        Eigen::VectorXf result = Eigen::VectorXf::Zero(3 * system.Positions.size());
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;

            glm::vec3       pos1 = glm::vec3(x[p1 * 3 + 0], x[p1 * 3 + 1], x[p1 * 3 + 2]);
            glm::vec3       pos0 = glm::vec3(x[p0 * 3 + 0], x[p0 * 3 + 1], x[p0 * 3 + 2]);
            glm::vec3 const x01  = pos1 - pos0;

            glm::vec3       v1  = glm::vec3(v[p1 * 3 + 0], v[p1 * 3 + 1], v[p1 * 3 + 2]);
            glm::vec3       v0  = glm::vec3(v[p0 * 3 + 0], v[p0 * 3 + 1], v[p0 * 3 + 2]);
            glm::vec3 const v01 = v1 - v0;

            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3       f   = system.Damping * glm::dot(v01, e01) * e01;
            for (int i = 0; i < 3; i++) {
                result[p0 * 3 + i] += f[i];
                result[p1 * 3 + i] -= f[i];
            }
        }
        return result;
    }

    //计算逆矩阵
    Eigen::SparseMatrix<float> cal_inverse(const Eigen::SparseMatrix<float> & M, int n) {
        auto                       solver = Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>>(M);
        Eigen::SparseMatrix<float> I(3 * n, 3 * n);
        I.setIdentity();
        Eigen::SparseMatrix<float> inverse = solver.solve(I);
        return inverse;
    }
    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code

        //显式欧拉
        /* int const steps = 1000;
        float const ddt   = dt / steps;
        for (std::size_t s = 0; s < steps; s++) {
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const      p0  = spring.AdjIdx.first;
                auto const      p1  = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3       f   = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        }*/
        
        //隐式欧拉
        int   steps = 1;
        float h     = dt / steps; // delta t

        for (std::size_t s = 0; s < steps; s++) {
            int             n       = system.Positions.size();
            Eigen::VectorXf x0      = Eigen::VectorXf::Zero(3 * n);
            Eigen::VectorXf v0      = Eigen::VectorXf::Zero(3 * n);
            Eigen::VectorXf gravity = Eigen::VectorXf::Zero(3 * n);

            for (int i = 0; i < 3 * n; i++) {
                x0[i] = system.Positions[i / 3][i % 3];
                v0[i] = system.Velocities[i / 3][i % 3];
            }

            for (int i = 0; i < n; ++i) {
                gravity[3 * i + 1] = -system.Gravity;
            }
            Eigen::VectorXf y = x0 + v0 * h + (gravity + Damping_F(system, x0, v0) / system.Mass) * h * h;

            Eigen::VectorXf x1 = y;
            float           g1 = g(system, x1, y, h); //初始值

            for (int iter = 0; iter < 3; iter++) {
                Eigen::VectorXf            grad_g = nabla_g(system, x1, y, h);
                Eigen::SparseMatrix<float> grad2_g(3 * n, 3 * n);
                Eigen::VectorXf            dx;

                nabla2_g(grad2_g, system, grad_g, x1, y, h); //计算黑塞矩阵
                dx                 = -cal_inverse(grad2_g, n) * grad_g;
                Eigen::VectorXf x2 = Eigen::VectorXf::Zero(3 * n);

                float beta  = 0.95;
                float alpha = 1.0f / beta;
                float gamma = 1e-3;
                float g2    = 0;

                do {
                    alpha = beta * alpha;
                    x2    = x1 + dx * alpha;
                    g2    = g(system, x2, y, h);
                } while (g2 > g1 + gamma * alpha * (grad_g.transpose() * dx)[0] + 1e-4);

                x1 = x2;
                g1 = g2;
            }

            Eigen::VectorXf v1 = (x1 - x0) / h;
            for (int i = 0; i < n; i++) {
                if (system.Fixed[i]) continue;
                system.Positions[i]  = { x1[i * 3], x1[i * 3 + 1], x1[i * 3 + 2] };
                system.Velocities[i] = { v1[i * 3], v1[i * 3 + 1], v1[i * 3 + 2] };
            }
        }
    }
} // namespace VCX::Labs::Animation
