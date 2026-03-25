#ifndef NUBS_TRAJECTORY_HPP
#define NUBS_TRAJECTORY_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace nubs
{

class BandedSystem 
{
private:
    int N, lowerBw, upperBw;
    std::vector<double> data; 
public:
    inline void create(const int &n, const int &p, const int &q) {
        N = n; lowerBw = p; upperBw = q;
        data.assign(N * (lowerBw + upperBw + 1), 0.0);
    }
    inline void reset(void) { std::fill(data.begin(), data.end(), 0.0); }
    
    inline const double &operator()(const int &i, const int &j) const { return data[(i - j + upperBw) * N + j]; }
    inline double &operator()(const int &i, const int &j) { return data[(i - j + upperBw) * N + j]; }
    
    template <typename EIGENMAT>
    inline EIGENMAT multiply(const EIGENMAT &x) const 
    {
        EIGENMAT res = EIGENMAT::Zero(N, x.cols());
        for (int i = 0; i < N; i++) {
            int j_start = std::max(0, i - lowerBw);
            int j_end = std::min(N - 1, i + upperBw);
            for (int j = j_start; j <= j_end; j++) {
                res.row(i) += operator()(i, j) * x.row(j);
            }
        }
        return res;
    }

    inline void factorizeLU() {
        int iM, jM; double cVl;
        for (int k = 0; k <= N - 2; k++) {
            iM = std::min(k + lowerBw, N - 1);
            cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; i++) { if (operator()(i, k) != 0.0) operator()(i, k) /= cVl; }
            jM = std::min(k + upperBw, N - 1);
            for (int j = k + 1; j <= jM; j++) {
                cVl = operator()(k, j);
                if (cVl != 0.0) {
                    for (int i = k + 1; i <= iM; i++) {
                        if (operator()(i, k) != 0.0) operator()(i, j) -= operator()(i, k) * cVl;
                    }
                }
            }
        }
    }

    template <typename EIGENMAT>
    inline void solve(EIGENMAT &b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            iM = std::min(j + lowerBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(i, j) != 0.0) b.row(i) -= operator()(i, j) * b.row(j);
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            b.row(j) /= operator()(j, j);
            iM = std::max(0, j - upperBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(i, j) != 0.0) b.row(i) -= operator()(i, j) * b.row(j);
            }
        }
    }

    template <typename EIGENMAT>
    inline void solveAdj(EIGENMAT &b) const 
    {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(j, i) != 0.0) b.row(i) -= operator()(j, i) * b.row(j);
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            iM = std::max(0, j - lowerBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(j, i) != 0.0) b.row(i) -= operator()(j, i) * b.row(j);
            }
        }
    }
};

template<int Dim, int MaxP = 7>
class NUBSTrajectory 
{
private:
    int s;         
    int p;         
    int order;
    int N_c;
    
    Eigen::VectorXd knots;
    Eigen::Matrix<double, Eigen::Dynamic, Dim> control_points;
    Eigen::Matrix<double, Eigen::Dynamic, Dim> B_matrix_rhs; 

    inline static const std::vector<std::vector<double>> gauss_nodes = {
        {}, {0.0}, 
        {-0.5773502691896, 0.5773502691896}, 
        {-0.7745966692415, 0.0, 0.7745966692415}, 
        {-0.8611363115941, -0.3399810435849, 0.3399810435849, 0.8611363115941},
        {-0.9061798459387, -0.5384693101057, 0.0, 0.5384693101057, 0.9061798459387}
    };
    inline static const std::vector<std::vector<double>> gauss_weights = {
        {}, {2.0}, 
        {1.0, 1.0}, 
        {0.5555555555556, 0.8888888888889, 0.5555555555556}, 
        {0.3478548451375, 0.6521451548625, 0.6521451548625, 0.3478548451375},
        {0.2369268850562, 0.4786286704994, 0.5688888888889, 0.4786286704994, 0.2369268850562}
    };

    inline void buildSystemMatrixA(int M, const Eigen::VectorXd& u_vec, BandedSystem& A_out) const {
        A_out.create(N_c, p, p);
        int row = 0;
        Eigen::Matrix<double, MaxP+1, MaxP+1> ders;
        for (int d = 0; d < s; d++) {
            dersBasisFuns(d, p, u_vec(p), u_vec, ders);
            for (int j = 0; j <= p; j++) A_out(row, j) = ders(d, j);
            row++;
        }
        for (int i = 1; i < M; i++) {
            double t = u_vec(p + i); 
            int span = p + i;
            dersBasisFuns(0, span, t, u_vec, ders);
            for (int j = 0; j <= p; j++) A_out(row, span - p + j) = ders(0, j);
            row++;
        }
        double t_end = u_vec(N_c);
        for (int d = s - 1; d >= 0; d--) {
            dersBasisFuns(d, N_c - 1, t_end, u_vec, ders);
            for (int j = 0; j <= p; j++) A_out(row, N_c - 1 - p + j) = ders(d, j);
            row++;
        }
    }

public:
    BandedSystem A;  

    NUBSTrajectory(int sys_order = 3) : s(sys_order), p(2 * sys_order - 1), order(2 * sys_order) {}

    inline int getS() const { return s; }
    inline int getP() const { return p; }
    inline int getCtrlPtNum(int M) const { return M + 2 * s - 1; }
    inline double getTotalDuration() const { return knots(knots.size() - 1); }
    const Eigen::VectorXd& getKnots() const { return knots; }
    const Eigen::Matrix<double, Eigen::Dynamic, Dim>& getControlPoints() const { return control_points; }
    inline int getPieceNum() const 
    {
        if (knots.size() == 0) return 0;
        return knots.size() - 2 * p - 1;
    }

    inline int findSpan(double t, int num_ctrl_pts, const Eigen::VectorXd& u) const {
        if (t >= u(num_ctrl_pts)) return num_ctrl_pts - 1;
        if (t <= u(p)) return p;
        int low = p, high = num_ctrl_pts, mid;
        while (low < high) {
            mid = (low + high) / 2;
            if (t < u(mid)) high = mid;
            else low = mid + 1;
        }
        return low - 1;
    }

    inline void dersBasisFuns(int n, int span, double t, const Eigen::VectorXd& u, Eigen::Matrix<double, MaxP+1, MaxP+1>& ders) const {
        n = std::min(n, p); 
        ders.setZero();
        double ndu[MaxP + 1][MaxP + 1] = {{0}}; 
        ndu[0][0] = 1.0;
        double left[MaxP + 1], right[MaxP + 1];

        for (int j = 1; j <= p; j++) {
            left[j] = t - u(span + 1 - j);
            right[j] = u(span + j) - t;
            double saved = 0.0;
            for (int r = 0; r < j; r++) {
                ndu[j][r] = right[r + 1] + left[j - r];
                double temp = (ndu[j][r] == 0.0) ? 0.0 : ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }
        for (int j = 0; j <= p; j++) ders(0, j) = ndu[j][p];
        if (n == 0) return;

        double a[2][MaxP + 1] = {{0}};
        for (int r = 0; r <= p; r++) {
            int s1 = 0, s2 = 1;
            a[0][0] = 1.0;
            for (int k = 1; k <= n; k++) {
                double d = 0.0;
                int rk = r - k, pk = p - k;
                if (r >= k) {
                    double den = ndu[pk + 1][rk];
                    a[s2][0] = (den == 0.0) ? 0.0 : a[s1][0] / den;
                    d = a[s2][0] * ndu[rk][pk];
                }
                int j1 = (rk >= -1) ? 1 : -rk;
                int j2 = (r - 1 <= pk) ? k - 1 : p - r;
                for (int j = j1; j <= j2; j++) {
                    double den = ndu[pk + 1][rk + j];
                    a[s2][j] = (den == 0.0) ? 0.0 : (a[s1][j] - a[s1][j - 1]) / den;
                    d += a[s2][j] * ndu[rk + j][pk];
                }
                if (r <= pk) {
                    double den = ndu[pk + 1][r];
                    a[s2][k] = (den == 0.0) ? 0.0 : -a[s1][k - 1] / den;
                    d += a[s2][k] * ndu[r][pk];
                }
                ders(k, r) = d;
                std::swap(s1, s2);
            }
        }
        double fac = p;
        for (int k = 1; k <= n; k++) 
        {
            for (int j = 0; j <= p; j++) ders(k, j) *= fac;
            fac *= (p - k);
        }
    }

    inline Eigen::VectorXd generateKnots(const Eigen::VectorXd& T, int nc) const {
        int num_knots = nc + p + 1;
        Eigen::VectorXd u = Eigen::VectorXd::Zero(num_knots);
        for(int i = 0; i <= p; i++) u(i) = 0.0;
        double current_t = 0.0;
        for(int i = 0; i < T.size(); i++) {
            current_t += T(i);
            u(p + 1 + i) = current_t;
        }
        for(int i = p + 1 + T.size(); i < num_knots; i++) u(i) = current_t;
        return u;
    }

    inline void generate(const Eigen::MatrixXd& P_inner, 
                         const Eigen::MatrixXd& headState, 
                         const Eigen::MatrixXd& tailState, 
                         const Eigen::VectorXd& T, 
                         Eigen::MatrixXd& P_full)
    {
        int M = T.size();
        N_c = getCtrlPtNum(M);
        P_full.resize(N_c, Dim);
        knots = generateKnots(T, N_c);
        
        buildSystemMatrixA(M, knots, A);
        
        Eigen::Matrix<double, Eigen::Dynamic, Dim> b = Eigen::Matrix<double, Eigen::Dynamic, Dim>::Zero(N_c, Dim);
        
        int row = 0;
        for (int d = 0; d < s; d++) {
            b.row(row++) = headState.col(d).transpose();
        }
        for (int i = 1; i < M; i++) {
            b.row(row++) = P_inner.row(i - 1); 
        }
        for (int d = s - 1; d >= 0; d--) {
            b.row(row++) = tailState.col(d).transpose();
        }
        
        B_matrix_rhs = b; 

        A.factorizeLU();
        A.solve(b);
        P_full = b;
        control_points = P_full; 
    }

    Eigen::Matrix<double, Dim, 1> evaluate(double t, int d_ord = 0) const 
    {
        if (t <= knots(p)) t = knots(p);
        if (t >= knots(knots.size() - p - 1)) t = knots(knots.size() - p - 1) - 1e-9;
        
        int span = findSpan(t, N_c, knots);
        Eigen::Matrix<double, MaxP+1, MaxP+1> ders;
        dersBasisFuns(d_ord, span, t, knots, ders);
        
        Eigen::Matrix<double, Dim, 1> res = Eigen::Matrix<double, Dim, 1>::Zero();
        for (int j = 0; j <= p; j++) {
            res += ders(d_ord, j) * control_points.row(span - p + j).transpose();
        }
        return res;
    }


    inline double getEnergy() const 
    {
        double cost = 0.0;
        int N_pts = std::min(s, 5); 
        Eigen::Matrix<double, MaxP+1, MaxP+1> ders;
        
        for(int i = p; i < knots.size() - p - 1; i++) {
            double t_start = knots(i), t_end = knots(i+1);
            if(t_end - t_start < 1e-9) continue;
            
            double len = t_end - t_start;
            double mid = (t_end + t_start) / 2.0;

            for (int k = 0; k < N_pts; k++) {
                double t = mid + (len / 2.0) * gauss_nodes[N_pts][k];
                double w = gauss_weights[N_pts][k] * (len / 2.0);

                dersBasisFuns(s, i, t, knots, ders); 
                
                Eigen::Matrix<double, Dim, 1> val = Eigen::Matrix<double, Dim, 1>::Zero();
                for (int j = 0; j <= p; j++) {
                    val += ders(s, j) * control_points.row(i - p + j).transpose(); 
                }
                cost += w * val.squaredNorm();
            }
        }
        return cost;
    }

    inline void getEnergyPartialGradByCoeffs(double &cost, Eigen::MatrixXd &gdC) const 
    {
        cost = 0.0;
        gdC.setZero(N_c, Dim);
        int N_pts = std::min(s, 5); 
        Eigen::Matrix<double, MaxP+1, MaxP+1> ders;

        for(int i = p; i < knots.size() - p - 1; i++) {
            double t_start = knots(i), t_end = knots(i+1);
            if(t_end - t_start < 1e-9) continue;
            
            double len = t_end - t_start;
            double mid = (t_end + t_start) / 2.0;

            for (int k = 0; k < N_pts; k++) {
                double t = mid + (len / 2.0) * gauss_nodes[N_pts][k];
                double w = gauss_weights[N_pts][k] * (len / 2.0);

                dersBasisFuns(s, i, t, knots, ders);

                Eigen::Matrix<double, Dim, 1> val = Eigen::Matrix<double, Dim, 1>::Zero();
                for (int j = 0; j <= p; j++) {
                    val += ders(s, j) * control_points.row(i - p + j).transpose(); 
                }
                cost += w * val.squaredNorm();

                for (int j = 0; j <= p; j++) 
                {
                    gdC.row(i - p + j) += 2.0 * w * ders(s, j) * val.transpose();
                }
            }
        }
    }


    inline void getEnergyPartialGradByTimes(const Eigen::VectorXd& T, Eigen::VectorXd &gdT_direct) 
    {
        double eps = 1e-5; 
        gdT_direct.resize(T.size());
        
        Eigen::VectorXd orig_knots = knots; 

        for (int i = 0; i < T.size(); i++) 
        {
            Eigen::VectorXd T_p = T, T_m = T;
            T_p(i) += eps;
            T_m(i) -= eps;
            
            knots = generateKnots(T_p, N_c);
            double cost_p = getEnergy();
            
            knots = generateKnots(T_m, N_c);
            double cost_m = getEnergy();

            gdT_direct(i) = (cost_p - cost_m) / (2.0 * eps);
        }
        knots = orig_knots; 
    }

    inline void propagateGrad(const Eigen::MatrixXd &gdC,
                              const Eigen::VectorXd &gdT_direct,
                              const Eigen::VectorXd &T,
                              Eigen::MatrixXd &gradByPoints,
                              Eigen::VectorXd &gradByTimes) 
    {
        int M = T.size();
        gradByPoints.resize(M - 1, Dim);
        gradByTimes.resize(M);

        Eigen::MatrixXd adjGrad = gdC;
        A.solveAdj(adjGrad); 

        for (int i = 0; i < M - 1; i++) {
            gradByPoints.row(i) = adjGrad.row(s + i);
        }

        double eps = 1e-5;
        
        for (int i = 0; i < M; i++) 
        {
            Eigen::VectorXd T_p = T, T_m = T;
            T_p(i) += eps;
            T_m(i) -= eps;

            Eigen::VectorXd u_plus = generateKnots(T_p, N_c);
            BandedSystem A_plus;
            buildSystemMatrixA(M, u_plus, A_plus);
            Eigen::MatrixXd Ap_C = A_plus.multiply(control_points);

            Eigen::VectorXd u_minus = generateKnots(T_m, N_c);
            BandedSystem A_minus;
            buildSystemMatrixA(M, u_minus, A_minus);
            Eigen::MatrixXd Am_C = A_minus.multiply(control_points);

            Eigen::MatrixXd dAdT_C = (Ap_C - Am_C) / (2.0 * eps);

            double adj_correction = -(adjGrad.cwiseProduct(dAdT_C)).sum();
            gradByTimes(i) = gdT_direct(i) + adj_correction;
        }
    }
};

} 
#endif