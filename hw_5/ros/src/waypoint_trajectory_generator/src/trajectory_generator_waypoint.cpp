#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

// ooqp
#include "ooqp/QpGenData.h"
#include "ooqp/QpGenVars.h"
#include "ooqp/QpGenResiduals.h"
#include "ooqp/GondzioSolver.h"
#include "ooqp/QpGenSparseMa27.h"

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

// define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(int n_seg, int d_order, const Eigen::VectorXd &Time){
    /*
     * get Q matrix
     * args:
     *      n_seg: the number of segments
     *      d_order: the deferential order: if minimal snap. it is 4
     *      Time: time allocation as a column vector, size: n_seg x 1,
     * return:
     *      Q: a matrix, size: n_seg * p_num1d x n_seg * p_num1d
     *      Note: p = [p0, p1, p2,...pn-1]'
     */
    int p_num1d = d_order * 2;
    MatrixXd Q = MatrixXd::Zero(n_seg * p_num1d, n_seg * p_num1d);
    for(int n = 0; n < n_seg; n++){
        for(int i = 0; i < p_num1d; i++){
            for(int l = 0; l < p_num1d; l++){
                if(i < d_order || l < d_order) continue;
                else{
                    Q(n*p_num1d+i, n*p_num1d+l) = Factorial(i) / Factorial(i-d_order) * Factorial(l) / 
                                                Factorial(l-d_order) / (i+l-2*d_order+1) * 
                                                pow(Time(n), i+l-2*d_order+1);
                }
            }
        }
    } 
    return Q;
} 

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getM(int n_seg, int d_order, const Eigen::VectorXd &Time){
     /*
     * get Mapping Matrix
     * args:
     *      n_seg: the number of segments
     *      d_order: the deferential order: if minimal snap. it is 4
     *      Time: time allocation as a column vector, size: n_seg x 1,
     * return:
     *      M: a matrix, size: n_seg * p_num1d x n_seg * p_num1d
     */
    int p_num1d = d_order * 2;
    MatrixXd M = MatrixXd::Zero(n_seg * p_num1d, n_seg * p_num1d);
    MatrixXd coeff(d_order, p_num1d);
    // 已经默认是4次，8个未知数，默认是minimal snap了，懒得写jerk之类其他的了
    coeff << 1,  1,  1,  1,  1,  1,  1,  1,
             0,  1,  2,  3,  4,  5,  6,  7,
             0,  0,  2,  6,  12, 20, 30, 42,
             0,  0,  0,  6,  24, 60, 120, 210;
    for(int n = 0; n < n_seg; n++){
        for(int i = 0; i < d_order; i++){ // 起点映射
            M(n*p_num1d+i, n*p_num1d+i) = coeff(i, i); 
        }
        for(int i = 0; i < d_order; i++){ // 末端映射
            for(int j = i; j < p_num1d; j++){
                if(i == j){
                    M(n*p_num1d+i+d_order, n*p_num1d+j) = coeff(i, j);
                }
                else{
                    M(n*p_num1d+i+d_order, n*p_num1d+j) = coeff(i, j) * pow(Time(n), j-i);
                }
            }
        }
    }
    return M;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getCt(int n_seg, int d_order){
    /*
     * get Selection Matrix
     * args:
     *      n_seg: the number of segments
     *      d_order: the deferential order: if minimal jerk. it is 3,
     * return:
     *      Ct: a matrix,
     *      row corresponds to original variable vector d ( 2 * d_order * n_seg )
     *      column corresponds to [ dF, dP ]' ( d_order*2*n_seg - (n_seg-1)*d_order )
     *      Note: the variables implicitly eliminates same variables
     */
    int ct_rows = d_order * 2 * n_seg;
    int ct_cols = d_order * 2 * n_seg - (n_seg-1) * d_order;
    MatrixXd Ct = MatrixXd::Zero(ct_rows, ct_cols);
    // 和matlab里的实现方式一样，直接用的别人的代码
    // 构建哈希表
    vector<int> d_vector;
    for(int k = 0; k < n_seg; k ++){
        for(int t = 0; t < 2; t++){
            for(int d = 0; d < d_order; d++){
                d_vector.push_back(k*100+t*10+d);
            }
        }
    }
    int val, row;
    int col = 0; 
    // 固定起点状态
    int k = 0;
    int t = 0;
    int d = 0;
    for(d = 0; d < d_order; d++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }
    // 固定中间节点位置
    t = 1;
    d = 0;
    for(k = 0; k < n_seg - 1; k++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        val = (k + 1) * 100 + (t - 1) * 10 + d;
        it= std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }
    // 固定终点状态
    k = n_seg - 1;
    t = 1;
    d = 0;
    for(d = 0; d < d_order; d++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }
    // 保证连续性约束
    k = 0;
    t = 1;
    d = 1;
    for(k = 0; k < n_seg - 1; k++){
        for(d = 1; d < d_order; d++){
            val = k * 100 + t * 10 + d;
            auto it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;
            val = (k + 1) * 100 + (t - 1) * 10 + d;
            it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;
            col += 1;
        }
    }
    return Ct;
}

/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

// closed-form
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients

    // 实现逻辑参照matlab的代码
    // get Q
    MatrixXd Q = getQ(m, d_order, Time);

    // compute M 将多项式系数p映射到始末两个端点的状态变量及其导数
    MatrixXd M = getM(m, d_order, Time);

    // compute Ct 选择矩阵将[d_F d_p]映射到[d_1 ... d_M]，其中d_p为要被优化的自由变量
    MatrixXd Ct = getCt(m, d_order);
    
    // 中间变量计算
    MatrixXd C = Ct.transpose();
    MatrixXd M_inv = M.inverse();
    MatrixXd M_inv_t = M_inv.transpose();
    MatrixXd R = C * M_inv_t * Q * M_inv * Ct; // 算出R
    int num_d_F = 2 * d_order + m - 1;
    int num_d_P = (m - 1) * (d_order - 1);
    MatrixXd R_pp = R.bottomRightCorner(num_d_P, num_d_P); // 得到R_pp
    MatrixXd R_fp = R.topRightCorner(num_d_F, num_d_P); // 得到R_fp

    // 计算d_F，进而得到d_P(x，y，z三维)，转为多项式系数存入PolyCoeff中
    for(int dim = 0; dim < 3; dim++){
        VectorXd wayPoints = Path.col(dim);
        
        VectorXd d_F = VectorXd::Zero(num_d_F); // 计算d_F，开始/结束的v，a，j = 0
        d_F(0) = wayPoints(0); 
        for(int i = 0; i < m - 1; i++){
            d_F(d_order + i) = wayPoints(i + 1);
        }
        d_F(d_order + m - 1) = wayPoints(m);

        VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F; // 计算d_P，即使得J取最优时的d_P值
        VectorXd d_total(d_F.rows() + d_P.rows());
        d_total << d_F, d_P;
        VectorXd poly_coef_1d = M_inv * Ct * d_total; // 将所得的d_P复原为多项式系数
        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose(); // 取转置

        for(int k = 0; k < m; k++){
            PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0, k*p_num1d, 1, p_num1d);
            // block(起始行索引，起始列索引，行数，列数)
        }
    }
    return PolyCoeff;
}


/************************************************这是一条分割线****************************************************/

// 在线性约束矩阵的指定位置插入系数
void TrajectoryGeneratorWaypoint::InsertCoff(const int row, 
                const int col, 
                Eigen::SparseMatrix<double, Eigen::RowMajor> & linearMatrix, 
                const double t, 
                const int d_order,
                bool one_line,
                bool reverse)
{
    int p_num1d = 2 * d_order;
    int flag = d_order;
    if(one_line){
        flag = 1;
    }
    Eigen::MatrixXd coff(d_order, p_num1d);
    // 目前只支持minimum snap
    coff << 1.0, 1.0*t, 1.0*pow(t,2), 1.0*pow(t,3), 1.0*pow(t,4), 1.0*pow(t,5), 1.0*pow(t,6), 1.0*pow(t,7),
            0.0, 1.0, 2.0*t, 3.0*pow(t,2), 4.0*pow(t,3), 5.0*pow(t,4), 6.0*pow(t,5), 7.0*pow(t,6),
            0.0, 0.0, 2.0, 6.0*t, 12.0*pow(t,2), 20.0*pow(t,3), 30.0*pow(t,4), 42.0*pow(t,5),
            0.0, 0.0, 0.0, 6.0, 24.0*t, 60.0*pow(t,2), 120.0*pow(t,3), 210.0*pow(t,4);
    if(reverse){
        coff = coff * (-1.0);
    }
    for (int i = 0; i < d_order && i < flag; ++i){
        for(int j = 0; j < p_num1d; ++j){
            linearMatrix.insert(row+i, col+j) = coff(i, j);
        }
    }
}

// 获取等式约束矩阵，也就是矩阵Aeq
void TrajectoryGeneratorWaypoint::GetLinearConstraintsMatrix(const int n_seg,
                const int d_order,
                const Eigen::VectorXd &Time,
                Eigen::SparseMatrix<double, Eigen::RowMajor> & linearMatrix)
{
    int p_order = 2 * d_order - 1;
    int p_num1d = p_order + 1;
    linearMatrix.resize(2*d_order + (n_seg-1)*(d_order + 1), p_num1d * n_seg);
    // 起点和终点限制约束
    int row = 0;
    int col = 0;
    InsertCoff(row, col, linearMatrix, 0, d_order, false, false);
    row += d_order;
    col = (n_seg - 1) * p_num1d;
    InsertCoff(row, col, linearMatrix, Time(n_seg-1), d_order, false, false);
    // 中间节点的位置约束
    row += d_order;
    for(int k = 0; k < n_seg -1; ++k){
        InsertCoff(row + k , k*p_num1d , linearMatrix, Time(k), d_order, true, false);
    }
    // 连续性约束
    row += n_seg - 1;
    for(int k = 0; k < n_seg - 1; ++k){
        InsertCoff(row, k*p_num1d , linearMatrix, Time(k), d_order, false, false);
        InsertCoff(row, (k + 1)*p_num1d , linearMatrix, 0, d_order, false, true);
        row += d_order;
    }
}

// 使用OOQP求解器
// referance: https://blog.csdn.net/a456459/article/details/120010550
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time,           // time allocation in each segment
            int shabi)                            // 多个参数，函数重载用
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    
    const int nx = m * p_num1d; // 向量x的元素个数
    double c[nx] = {0.0}; // 目标函数中的线性项，在本作业中为0
    double xupp[nx] = {0.0}; // 这两项表示x的上界，本作业p系数没上界，故这两项都设置成零
    char ixupp[nx] = {0};
    double xlow[nx] = {0.0}; // 这两项表示x的下界，本作业p系数没下界，故这两项都设置成零
    char ixlow[nx] = {0};

    const int nnzQ = m * (p_num1d + 1) * p_num1d / 2; // 目标函数Q矩阵下三角元素的个数
    int irowQ[nnzQ]; // 这三项表示Q矩阵的nnzQ个下三角元素
    int jcolQ[nnzQ]; // 
    double dQ[nnzQ]; // 

    int my = d_order*2 + (m-1) + (m-1)*d_order; // 线性等式约束的个数=始末+位置+连续性
    double b[my] = {0.0}; // 线性等式约束的右侧向量
    // int nnzA = bzd; // 线性等式约束矩阵A的非零元素个数
    // int irowA[nnzA]; // 这三项表示A的nnzA个非零元素
    // int jcolA[nnzA]; // 
    // double dA[nnzA]; //

    const int mz = 0; // 不等式约束的个数，本作业没有不等式约束项，赋零即可
    double *cupp = 0;
    char *icupp = 0;
    double *clow = 0;
    char *iclow = 0;
    const int nnzC = 0; // 不等式约束矩阵C的元素个数？，本作业没有不等式约束项，赋零即可
    int *irowC = 0;
    int *jcolC = 0;
    double *dC = 0;

    // 给Q赋值
    int Q_idx = 0;
    for(int n = 0; n < m; n++){
        for(int i = 0; i < p_num1d; i++){
            for(int l = 0; l < p_num1d; l++){
                if(i >= l){ // 下三角
                    irowQ[Q_idx] = n * p_num1d + i;
                    jcolQ[Q_idx] = n * p_num1d + l;
                    if(i < d_order || l < d_order){
                        dQ[Q_idx] = 0;
                    }else{
                        dQ[Q_idx] = Factorial(i) / Factorial(i-d_order) * Factorial(l) / 
                                    Factorial(l-d_order) / (i+l-2*d_order+1) * 
                                    pow(Time(n), i+l-2*d_order+1);
                    }
                    Q_idx++;
                }
            }
        }
    }

    // 给A赋值
    Eigen::SparseMatrix<double, Eigen::RowMajor> A;
    GetLinearConstraintsMatrix(m, d_order, Time, A);
    A.prune(0.0); // 删除稀疏矩阵的所有零元素
    int nnzA = A.nonZeros();
    std::vector<int> irowA_vec(nnzA);
    std::vector<int> jcolA_vec(nnzA);
    std::vector<double> dA_vec(nnzA);
    // 取出非零元素
    int A_idx = 0;
    for (int j = 0; j < A.outerSize(); ++j) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A, j); it; ++it) {
            irowA_vec[A_idx] = it.row();
            jcolA_vec[A_idx] = it.col();
            dA_vec[A_idx] = it.value();
            A_idx++;
        }
    }
    // 将vector转换成正常数组
    int n = irowA_vec.size();
    int *irowA = new int[n];
    std::copy(ir_vec.begin(), irowA_vec.end(), irowA); // irowA
    n = jcolA_vec.size();
    int *jcolA = new int[n];
    std::copy(jcolA_vec.begin(), jcolA_vec.end(), jcolA); // jcolA
    n = dA_vec.size();
    double *dA = new double[n];
    std::copy(dA_vec.begin(), dA_vec.end(), dA); // dA

    for(int dim = 0; dim < 3; dim++){ // 分别求解x，y，z三轴
        // 给b赋值
        VectorXd wayPoints = Path.col(dim);
        b[0] = wayPoints(0); // 起始位置约束
        b[d_order] = wayPoints(m); // 结束位置约束
        for(int i = 0; i < m - 1; i++){ // 中间位置约束
            b[2*d_order + i] = wayPoints(i + 1);
        }

        // 官方求解步骤
        QpGenSparseMa27 * qp = new QpGenSparseMa27(nx,my,mz,nnzQ,nnzA,nnzC);
        QpGenData * prob = (QpGenData *)qp -> copyDataFromSparseTriple(
                c,irowQ,nnzQ,jcolQ,dQ,
                xlow,ixlow,xupp,ixupp,
                irowA,nnzA,jcolA,dA,b,
                irowC,nnzC,jcolC,dC,
                clow,iclow,cupp,icupp);
        QpGenVars * vars = (QpGenVars *)qp -> makeVariables(prob);
        QpGenResiduals *resid = (QpGenResiduals *)qp -> makeResiduals(prob);
        GondzioSolver *s = new GondzioSolver(qp,prob);
        s->monitorSelf();
        int isErr = s->solve(prob,vars,resid);
        
        // 取出解
        double d_var[nx]; // 将解存放在该数组中
        if(isErr == 0){ // 如果解成功
            cout << "解成功了捏!" << endl;
            vars->x->copyIntoArray(d_var);
            VectorXd poly_coef_1d = Eigen::Map<Eigen::VectorXd>(d_var, nx); // 转为VectorXd类型
            MatrixXd poly_coef_1d_t = poly_coef_1d.transpose(); // 取转置
            for(int k = 0; k < m; k++){
                PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0, k*p_num1d, 1, p_num1d);
                // block(起始行索引，起始列索引，行数，列数)
            }
        }else{
            cout << "解失败了捏!" << endl;
        }
    }
    return PolyCoeff;
}