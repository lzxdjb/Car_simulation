

#define NSTATES 3
#define NINPUTS 2
#define Ttime 0.2
#define NHORIZON 10
#define NTOTAL 301
#include <Eigen/Dense>
// #include <Eigen.h>
// #include "problem_data/quadrotor_20hz_params.hpp"
// #include "trajectory_data/quadrotor_20hz_y_axis_line.hpp"
#include <tinympc/admm.hpp>
#include<iostream>
using namespace std;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
using Eigen::Matrix;

TinyCache cache;
TinyWorkspace work;
TinySettings settings;
TinySolver solver{&settings, &cache, &work};

typedef Matrix<tinytype, NINPUTS, NHORIZON - 1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;




int main()
{
    cache.rho = 1;

    tinyMatrix work_Adyn(3 , 3);
    work_Adyn.setIdentity();
    work.Adyn = work_Adyn;

    tinyMatrix Q(3, 3); // Define a 3x3 matrix Q

    // Method 1: Using comma initialization
    Q << 1.0, 0.0, 0.0,
         0.0, 5.0, 0.0,
         0.0, 0.0, 0.1;
    tinyMatrix R(2 , 2) ;
    R<< 0.5, 0.0, 0.0, 0.05;

    tiny_VectorNx x0(3 , 1), x1(3 , 1); // current and next simulation states
    x0 <<0 , 0 , 0;
    tinyMatrix work_Bdyn(3, 2); // Example: Bdyn is a 3x2 matrix

    // Assign values to Bdyn based on x0
    work_Bdyn << std::cos(x0(2, 0)), 0,
                 std::sin(x0(2, 0)), 0,
                 0, 1;
    work.Bdyn = work_Bdyn;


 // Update by adding rho * identity matrix to Q, R
    tinyMatrix Q1 =Q +  cache.rho* tinyMatrix::Identity(3, 3);
    tinyMatrix R1 =R +  cache.rho* tinyMatrix::Identity(2, 2);

    tinyMatrix Ktp1 = tinyMatrix::Zero(2, 3);
    tinyMatrix Ptp1 = 1 * tinyMatrix::Ones(3, 1).array().matrix().asDiagonal();
    tinyMatrix Kinf = tinyMatrix::Zero(2, 3);
    tinyMatrix Pinf = tinyMatrix::Zero(3, 3);

    for (int i = 0; i < 1000; i++)
    {

        tinyMatrix temp = R1 + work.Bdyn.transpose() * Ptp1 * work.Bdyn;
        if (temp.determinant() != 0) {  // Ensure the matrix is invertible
            Kinf = temp.inverse() * work.Bdyn.transpose() * Ptp1 * work.Adyn;
        } else {
            std::cerr << "Matrix is not invertible at iteration " << i << std::endl;
            break;
        }
     
        Pinf = Q1+ work.Adyn.transpose() * Ptp1 * (work.Adyn - work.Bdyn * Kinf);
        // if Kinf converges, break
        if ((Kinf - Ktp1).cwiseAbs().maxCoeff() < 1e-5)
        {
            break;
        }
        Ktp1 = Kinf;
        Ptp1 = Pinf;

    }
    tinyMatrix q(3 , 1);
    q <<1 , 5 , 0.1;
    work.Q = q;
    tinyMatrix r(2 , 1);
    r << 0 , 0;
    work.R = r;

    // std::cout<<"Kinf = " << Kinf;
    // exit(0);

    // Compute cached matrices
    tinyMatrix Quu_inv = (R1 + work.Bdyn.transpose() * Pinf * work.Bdyn).inverse();
    tinyMatrix AmBKt = (work.Adyn -work.Bdyn * Kinf).transpose();

    cache.rho = 1;
    cache.Kinf = Kinf;
    cache.Pinf = Pinf;
    cache.Quu_inv = Quu_inv;
    cache.AmBKt = AmBKt;


    work.nx = NSTATES;
    work.nu = NINPUTS;
    work.N = NHORIZON;

    work.u_min = tiny_MatrixNuNhm1::Constant(-50);
    work.u_max = tiny_MatrixNuNhm1::Constant(50);
    work.x_min = tiny_MatrixNxNh::Constant(-50);
    work.x_max = tiny_MatrixNxNh::Constant(50);

    work.Xref = tiny_MatrixNxNh::Zero();
    work.Uref = tiny_MatrixNuNhm1::Zero();

    work.x = tiny_MatrixNxNh::Zero();
    work.q = tiny_MatrixNxNh::Zero();
    work.p = tiny_MatrixNxNh::Zero();
    work.v = tiny_MatrixNxNh::Zero();
    work.vnew = tiny_MatrixNxNh::Zero();
    work.g = tiny_MatrixNxNh::Zero();

    work.u = tiny_MatrixNuNhm1::Zero();
    work.r = tiny_MatrixNuNhm1::Zero();
    work.d = tiny_MatrixNuNhm1::Zero();
    work.z = tiny_MatrixNuNhm1::Zero();
    work.znew = tiny_MatrixNuNhm1::Zero();
    work.y = tiny_MatrixNuNhm1::Zero();

    work.primal_residual_state = 0;
    work.primal_residual_input = 0;
    work.dual_residual_state = 0;
    work.dual_residual_input = 0;
    work.status = 0;
    work.iter = 0;

    settings.abs_pri_tol = 0.001;
    settings.abs_dua_tol = 0.001;
    settings.max_iter = 100;
    settings.check_termination = 1;
    settings.en_input_bound = 1;
    settings.en_state_bound = 1;


    Eigen::Vector3d columnVector(3.5, 2.7, 0.5);
    Matrix<tinytype, NSTATES, NTOTAL> Xref_total;
    // Fill each column of the matrix with the column vector
    for (int i = 0; i < NTOTAL; ++i)
    {
        Xref_total.col(i) = columnVector;
    }
    work.Xref = Xref_total.block<NSTATES, NHORIZON>(0, 0);

    // Initial state

    for (int k = 0; k < 1000; ++k)
    {
        std::cout << "tracking error: " << (x0 - work.Xref.col(1)).norm() << std::endl;

        // 1. Update measurement
        work.x.col(0) = x0;

        // // 2. Update reference
        work.Xref = Xref_total.block<NSTATES, NHORIZON>(0, 1);

        // // 3. Reset dual variables if needed
        tinyMatrix work_Bdyn(3, 2); // Example: Bdyn is a 3x2 matrix

    // Assign values to Bdyn based on x0
        work_Bdyn << std::cos(x0(2, 0)), 0,
                 std::sin(x0(2, 0)), 0,
                 0, 1;
        work.Bdyn = work_Bdyn;

        // std::cout << work.Bdyn << std::endl;


        work.y = tiny_MatrixNuNhm1::Zero();
        work.g = tiny_MatrixNxNh::Zero();

        // // 4. Solve MPC problem
        tiny_solve(&solver);

        std::cout << work.iter << std::endl;
        std::cout << work.u.col(0).transpose().format(CleanFmt) << std::endl;

        exit(0);
        // 5. Simulate forward
        x1 = work.Adyn * x0 + work.Bdyn * work.u.col(0) * Ttime;
        x0 = x1;


        // std::cout << x0.transpose().format(CleanFmt) << std::endl;
        // std::cout<<x0<<endl;

        // int start_index = k * tiny_VectorNx::RowsAtCompileTime;
        // for (int j = 0; j < tiny_VectorNx::RowsAtCompileTime; ++j) {
        //     data[start_index + j] = x0[j];
    }
}

