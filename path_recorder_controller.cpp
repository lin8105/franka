#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
// #include <franka/gripper.h>  // 注释掉夹爪头文件
#include <franka/model.h>
#include <franka/robot.h>
#include "common/examples_common.h"


#include <dqrobotics/DQ.h>
#include "FrankaRobot.h"


#include "cnpy.h" 

// --- 全局变量 ---
std::atomic<double> shared_gripper_width{0.08};
std::atomic<bool> shared_is_grasped{false}; 
std::atomic<bool> keep_running{true};
std::atomic<char> manual_cmd{' '}; 

// 定义内存缓冲结构体，用于 1000Hz 极速记录
struct RecordFrame {
    uint64_t timestamp;
    franka::RobotState robot_state;
    double gripper_width;
    bool is_grasped;
};

int main(int argc, char **argv)
{
    // 参数：程序名、机器人IP、CSV输出路径
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> <outfile.csv>" << std::endl;
        return -1;
    }

    // std::thread gripper_thread; // 注释掉夹爪线程变量

    try
    {
        franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
        robot.automaticErrorRecovery();
        setDefaultBehavior(robot);
        franka::Model model = robot.loadModel();
        
        // 初始化 DQ 运动学模型
        DQ_robotics::FrankaRobot custom_franka;
        DQ_robotics::DQ_SerialManipulatorMDH kin = custom_franka.kinematics();

        // ==========================================================
        // 【夹爪控制线程已完全注释】
        // ==========================================================
        /*
        franka::Gripper gripper(argv[1]);
        gripper_thread = std::thread([&gripper]() {
            while (keep_running) {
                try {
                    char cmd = manual_cmd.exchange(' '); 
                    if (cmd == 'g') {
                        gripper.grasp(0.0, 0.05, 30.0, 0.1, 0.1); 
                    } else if (cmd == 'o') {
                        gripper.move(0.08, 0.05); 
                    }
                    
                    franka::GripperState state = gripper.readOnce();
                    shared_gripper_width = state.width;
                    shared_is_grasped = state.is_grasped; 
                    
                } catch (...) {
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        */

        // 屏蔽碰撞检测，保证拖动丝滑
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, 
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, 
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        std::cout << "Ready. Press 'q' to quit recording" << std::endl; // 打印给 Python 监听的统一就绪标志
        
        std::cin.clear();
        std::cin.ignore(10000, '\n');

        // 2. 键盘监听线程
        std::thread keyboard_thread([]() {
            char c;
            while (keep_running) {
                if (std::cin >> c) {
                    if (c == 'g' || c == 'o') {
                        manual_cmd = c; 
                    } else if (c == 'q') {
                        keep_running = false;
                    }
                }
            }
        });
        keyboard_thread.detach(); 

 
        // 预分配内存
        std::vector<RecordFrame> trajectory_buffer;
        trajectory_buffer.reserve(150000); // 预留约 150 秒

        // 3. 1000Hz 实时控制循环
        auto zero_force_control_callback = [&](const franka::RobotState &robot_state, franka::Duration duration) -> franka::Torques {
            
            auto now = std::chrono::high_resolution_clock::now();
            uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
            
            RecordFrame frame;
            frame.timestamp = timestamp;
            frame.robot_state = robot_state;
            
            // 夹爪控制移出后，直接在此处写死默认状态值，防止指针或逻辑空挂
            frame.gripper_width = 0.08; 
            frame.is_grasped = false;   
            
            trajectory_buffer.push_back(frame);

            std::array<double, 7> zero_tau = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (!keep_running) {
                return franka::MotionFinished(franka::Torques(zero_tau));
            }
            return zero_tau;
        };

        robot.control(zero_force_control_callback);

        // 准备 CSV
        std::ofstream csv_file(argv[2], std::ios::out);
        csv_file << "timestamp_us,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,"
                 << "T01,T02,T03,T04,T05,T06,T07,T08,T09,T10,T11,T12,T13,T14,T15,T16,"
                 << "F_x,F_y,F_z,tau_x,tau_y,tau_z,tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,tau_J7,"
                 << "v_x,v_y,v_z,w_x,w_y,w_z,gripper_width,is_grasped,"
                 << "qr_w,qr_x,qr_y,qr_z,qd_w,qd_x,qd_y,qd_z,"
                 << "citr_ff,citr_ftau,citr_tautau,citr_fv,citr_tauv,citr_vv,citr_fw,citr_tauw,citr_vw,citr_ww\n";

        // 准备 NPY 缓冲 (每个时间步存 4x4=16 个元素)
        std::vector<double> citr_matrix_buffer;
        citr_matrix_buffer.reserve(trajectory_buffer.size() * 16); 

        for (const auto& frame : trajectory_buffer) {
            const auto& state = frame.robot_state;

            // --- A. 统一计算耗时的数学量 ---
            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
            Eigen::Matrix<double, 6, 1> twist = jacobian * dq;

            Eigen::VectorXd q_vec = Eigen::VectorXd::Map(state.q.data(), 7);
            DQ_robotics::DQ x = kin.fkm(q_vec); 
            Eigen::Matrix<double, 8, 1> x_vec8 = x.vec8();

            Eigen::Vector3d f_vec(state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1], state.O_F_ext_hat_K[2]);
            Eigen::Vector3d tau_vec(state.O_F_ext_hat_K[3], state.O_F_ext_hat_K[4], state.O_F_ext_hat_K[5]);
            Eigen::Vector3d v_vec = twist.head(3);
            Eigen::Vector3d w_vec = twist.tail(3);

            // 预先算好 10 个独立内积特征值
            double ff = f_vec.dot(f_vec);
            double ftau = f_vec.dot(tau_vec);
            double tautau = tau_vec.dot(tau_vec);
            double fv = f_vec.dot(v_vec);
            double tauv = tau_vec.dot(v_vec);
            double vv = v_vec.dot(v_vec);
            double fw = f_vec.dot(w_vec);
            double tauw = tau_vec.dot(w_vec);
            double vw = v_vec.dot(w_vec);
            double ww = w_vec.dot(w_vec);

            // --- B. CSV: 存入所有数据 + 10个标量特征 ---
            csv_file << frame.timestamp << ",";
            for(double q : state.q) csv_file << q << ",";
            for(double dq_val : state.dq) csv_file << dq_val << ",";
            for(double t : state.O_T_EE) csv_file << t << ",";
            for(double f : state.O_F_ext_hat_K) csv_file << f << ",";
            for(double tau : state.tau_ext_hat_filtered) csv_file << tau << ",";
            for(int i=0; i<6; i++) csv_file << twist(i) << ",";
            csv_file << frame.gripper_width << "," << (frame.is_grasped ? 1 : 0) << ",";
            for(int i=0; i<8; i++) csv_file << x_vec8(i) << ",";
            
            // 写入 10 个特征值到 CSV
            csv_file << ff << "," << ftau << "," << tautau << "," << fv << "," 
                     << tauv << "," << vv << "," << fw << "," << tauw << "," 
                     << vw << "," << ww << "\n";

            // --- C. NPY: 存入完整的 4x4 对称矩阵 (按行铺平为 16 个元素) ---
            citr_matrix_buffer.push_back(ff);
            citr_matrix_buffer.push_back(ftau);
            citr_matrix_buffer.push_back(fv);
            citr_matrix_buffer.push_back(fw);
            
            citr_matrix_buffer.push_back(ftau);   
            citr_matrix_buffer.push_back(tautau);
            citr_matrix_buffer.push_back(tauv);
            citr_matrix_buffer.push_back(tauw);
            
            citr_matrix_buffer.push_back(fv);     
            citr_matrix_buffer.push_back(tauv);   
            citr_matrix_buffer.push_back(vv);
            citr_matrix_buffer.push_back(vw);
            
            citr_matrix_buffer.push_back(fw);     
            citr_matrix_buffer.push_back(tauw);   
            citr_matrix_buffer.push_back(vw);     
            citr_matrix_buffer.push_back(ww);
        }

        csv_file.close();

        std::string csv_path = argv[2];
        size_t last_slash = csv_path.find_last_of("/\\");
        std::string npy_path = (last_slash == std::string::npos) ? "citr_matrices.npy" : csv_path.substr(0, last_slash + 1) + "citr_matrices.npy";

        cnpy::npy_save("citr_matrices.npy", &citr_matrix_buffer[0], {trajectory_buffer.size(), 4, 4}, "w");
    }
    catch (const franka::Exception &ex)
    {
        std::cout << "Exception: " << ex.what() << std::endl;
    }

    keep_running = false;
    // if (gripper_thread.joinable()) gripper_thread.join(); // 注释掉夹爪线程回收

    return 0;
}