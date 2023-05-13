#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;
    public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time,
            int shabi); // 重载

        void GetLinearConstraintsMatrix(
            const int n_seg,
            const int d_order,
            const Eigen::VectorXd &Time,
            Eigen::SparseMatrix<double, Eigen::RowMajor> & linearMatrix);

        void InsertCoff(const int row, 
            const int col, 
            Eigen::SparseMatrix<double, Eigen::RowMajor> & linearMatri, 
            const double t, 
            const int d_order,
            bool one_line,
            bool reverse);
        
        Eigen::MatrixXd getCt(int n_seg, int d_order);
        Eigen::MatrixXd getQ(int n_seg, int d_order, const Eigen::VectorXd &Time); 
        Eigen::MatrixXd getM(int n_seg, int d_order, const Eigen::VectorXd &Time); 
        
        int Factorial(int x);
};
        

#endif
