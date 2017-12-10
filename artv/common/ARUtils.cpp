#include <omp.h>
#include <common/ARUtils.h>

using namespace std;
using namespace cv;

namespace ar {
    using cv::Mat;

    /// Plot matched points between two images.
    void PlotMatches(const Mat &img1,
                     const Mat &img2,
                     const vector<cv::Point2f> &pts1,
                     const vector<cv::Point2f> &pts2,
                     Mat &out) {
        vconcat(img1, img2, out);
        auto num_points = pts1.size();
        for (int i = 0; i < num_points; ++i) {
            circle(out, pts1[i], 4, Scalar(255, 0, 0));
            auto pt2 = pts2[i];
            pt2.y += img1.rows;
            circle(out, pt2, 4, Scalar(255, 0, 0));
            line(out, pts1[i], pt2, Scalar(0, 255, 0), 2);
        }
        resize(out, out, Size(out.rows / 2, out.cols / 2));
    }

    cv::Mat COMMON_API CombineExtrinsics(const cv::Mat &base, const cv::Mat &rel) {
        Mat res(3, 4, CV_32F);
        Mat(rel.col(3) + rel.colRange(0, 3) * base.col(3)).copyTo(res.col(3));
        Mat(rel.colRange(0, 3) * base.colRange(0, 3)).copyTo(res.colRange(0, 3));
        return res;
    }

    /// Recover rotation and translation from an essential matrix.
    ///	This operation produces four posible results, each is a 3x4 matrix that combines
    ///	rotation and translation.
    vector<Mat> RecoverRotAndTranslation(const Mat &essential_matrix) {
        Mat U, W, Vt;
        auto svd = SVD();
        svd.compute(essential_matrix, W, U, Vt);
        W = Mat::diag(W);

        float m = (W.at<float>(1, 1) + W.at<float>(2, 2)) / 2;
        W.at<float>(0, 0) = W.at<float>(1, 1) = m;
        Mat regularized_E = U * W * Vt;
        svd.compute(regularized_E, W, U, Vt);

        W = Mat::zeros(3, 3, CV_32F);
        W.at<float>(0, 1) = -1;
        W.at<float>(1, 0) = W.at<float>(2, 2) = 1;
        if (determinant(U * W * Vt) < 0)
            W = -W;
        assert(abs(determinant(W) - 1) < 0.01 || abs(determinant(W) + 1) < 0.01);

        vector<Mat> res;
        res.reserve(4);
        Mat R1 = U * W * Vt;
        Mat R2 = U * W.t() * Vt;
        Mat t = U.col(2) / max({abs(U.at<float>(2, 0)), abs(U.at<float>(2, 1)), abs(U.at<float>(2, 2))});
        assert(abs(determinant(R1) - 1) < 0.001);
        assert(abs(determinant(R2) - 1) < 0.001);

        Mat candidate;
        hconcat(R1, t, candidate);
        assert(abs(determinant(candidate.colRange(0, 3)) - 1) < 0.001);
        res.push_back(candidate.clone());
        hconcat(R1, -t, candidate);
        assert(abs(determinant(candidate.colRange(0, 3)) - 1) < 0.001);
        res.push_back(candidate.clone());
        hconcat(R2, t, candidate);
        assert(abs(determinant(candidate.colRange(0, 3)) - 1) < 0.001);
        res.push_back(candidate.clone());
        hconcat(R2, -t, candidate);
        assert(abs(determinant(candidate.colRange(0, 3)) - 1) < 0.001);
        res.push_back(candidate);
        return res;
    }

    /// Calculate the relative rotation and translation from camera 1 to camera 2,
    //	given their own rotations and translations with respect to the world coordinate.
    pair<Mat, Mat> CalRelRotAndTranslation(Mat R1, Mat t1, Mat R2, Mat t2) {
        return {R1.t() * R2, R1.t() * (t2 - t1)};
    }

    /// Input a series of camera matrices and 2D points. The 2D points are all matched in order to relate to some 3D points.
    ///	Output the estimation of 3D points and estimation error.
    ERROR_CODE Triangulate(const vector<pair<Mat, Mat>> &pts,
                           Mat &points3d,
                           double *error) {
        int num_pts = -1;
        for (auto &p : pts) {
            if (num_pts < 0)
                num_pts = p.second.rows;
            else if (p.second.rows != num_pts)
                return AR_INVALID_INPUT;
        }

        // Estimate the 3D points with all the information.
        int num_points = pts[0].second.rows;
        auto num_cameras = pts.size();
        points3d = Mat(num_points, 3, CV_32F);

        for (int i = 0; i < num_points; ++i) {
            Mat A = Mat(static_cast<int>(num_cameras << 1), 4, CV_32F);
            for (int j = 0; j < num_cameras; ++j) {
                ((Mat) (pts[j].first.row(0) - pts[j].second.at<float>(i, 0) * pts[j].first.row(2))).copyTo(
                        A.row(2 * j));
                ((Mat) (pts[j].first.row(1) - pts[j].second.at<float>(i, 1) * pts[j].first.row(2))).copyTo(
                        A.row(2 * j + 1));
            }
            Mat U, W, VT;
            auto svd = SVD();
            svd.compute(A, W, U, VT);
            Mat p = VT.row(VT.rows - 1);
            p = p / p.at<float>(0, 3);
            for (int k = 0; k < num_cameras; ++k) {
                Mat proj = p * pts[k].first.t();
                proj = proj.colRange(0, 2) / proj.at<float>(0, 2);
                Mat diff = proj - pts[k].second.row(i);
                double err = diff.at<float>(0, 0) * diff.at<float>(0, 0) + diff.at<float>(0, 1) * diff.at<float>(0, 1);
                *error += err;
            }
            p.colRange(0, 3).copyTo(points3d.row(i));
        }
        return AR_SUCCESS;
    }

    struct BALResidual_1 {
        BALResidual_1(double pts1[], int N, double K1[], double M1[]) : N_(N) {
            pts1_ = new double[N];
            memcpy(pts1_, pts1, sizeof(double) * N);
            memcpy(K1_, K1, sizeof(double) * 9);
            memcpy(M1_, M1, sizeof(double) * 9);
        }

        ~BALResidual_1() {
            delete[] pts1_;
        }

        template<typename T>
        bool operator()(const T *const points, //3*N
                        T *residuals) const {
            T C1[3][4];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    C1[i][j] = T(0);
                    for (int k = 0; k < 3; k++) {
                        C1[i][j] += K1_[i * 3 + k] * M1_[k * 4 + j];
                    }
                }
            }
            T p1_proj[3][N_];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < N_; j++) {
                    p1_proj[i][j] = T(0);
                    for (int k = 0; k < 3; k++) {
                        p1_proj[i][j] += C1[i][k] * points[j * 3 + k];
                    }
                    p1_proj[i][j] += C1[i][3];
                }
            }
            residuals[0] = T(0);
            residuals[1] = T(0);
            for (int i = 0; i < N_; i++) {
                residuals[0] += pow(p1_proj[0][i] / p1_proj[2][i] - T(pts1_[i * 2]), 2.);
                residuals[1] += pow(p1_proj[1][i] / p1_proj[2][i] - T(pts1_[i * 2 + 1]), 2.);
            }
            residuals[0] = sqrt(residuals[0]);
            residuals[1] = sqrt(residuals[1]);
            return true;
        }

    private:
        // Observations for a sample.
        double *pts1_;
        const int N_;
        double K1_[9]{};
        double M1_[9]{};
    };

    struct BALResidual_2 {
        BALResidual_2(double pts2[], int N, double K2[]) : N_(N) {
            pts2_ = new double[N];
            memcpy(pts2_, pts2, sizeof(double) * N);
            memcpy(K2_, K2, sizeof(double) * 9);
        }

        ~BALResidual_2() {
            delete[] pts2_;
        }

        template<typename T>
        vector<T> rawRodrigues(const T *const r) const {
            vector<T> R(9);
            T theta = T(0);
            for (int i = 0; i < 3; i++) {
                theta += r[i] * r[i];
            }
            theta = sqrt(theta);
            if (theta < T(DBL_EPSILON)) {
                R[0] = R[4] = R[8] = T(1);
                R[1] = R[2] = R[3] = R[5] = R[6] = R[7] = T(0);
            } else {
                vector<T> u(3);
                for (int i = 0; i < 3; i++) {
                    u[i] = r[i] / theta;
                }
                T costheta = cos(theta);
                T sintheta = sin(theta);
                R[0] = costheta + u[0] * u[0] * (T(1) - costheta);
                R[1] = u[0] * u[1] * (T(1) - costheta) - u[2] * sintheta;
                R[2] = u[0] * u[2] * (T(1) - costheta) + u[1] * sintheta;
                R[3] = u[1] * u[0] * (T(1) - costheta) + u[2] * sintheta;
                R[4] = costheta + u[1] * u[1] * (T(1) - costheta);
                R[5] = u[1] * u[2] * (T(1) - costheta) - u[0] * sintheta;
                R[6] = u[2] * u[0] * (T(1) - costheta) - u[1] * sintheta;
                R[7] = u[2] * u[1] * (T(1) - costheta) + u[0] * sintheta;
                R[8] = costheta + u[2] * u[2] * (T(1) - costheta);
            }
            return R;
        }

        template<typename T>
        bool operator()(const T *const r2, //3
                        const T *const t2, //3
                        const T *const points, //3*N
                        T *residuals) const {
            vector<T> R2 = rawRodrigues(r2);
            //debug
            bool DEBUG_FLAG = false;
            T C2[3][4];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    C2[i][j] = T(0);
                    for (int k = 0; k < 3; k++) {
                        C2[i][j] += K2_[i * 3 + k] * R2[k * 3 + j];
                    }
                }
                C2[i][3] = T(0);
                for (int k = 0; k < 3; k++) {
                    C2[i][3] += K2_[i * 3 + k] * t2[k];
                }
            }
            T p2_proj[3][N_];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < N_; j++) {
                    p2_proj[i][j] = T(0);
                    for (int k = 0; k < 3; k++) {
                        p2_proj[i][j] += C2[i][k] * points[j * 3 + k];
                    }
                    p2_proj[i][j] += C2[i][3];
                }
            }
            residuals[0] = T(0);
            residuals[1] = T(0);
            for (int i = 0; i < N_; i++) {
                residuals[0] += pow(p2_proj[0][i] / p2_proj[2][i] - T(pts2_[i * 2]), 2.);
                residuals[1] += pow(p2_proj[1][i] / p2_proj[2][i] - T(pts2_[i * 2 + 1]), 2.);
            }
            residuals[0] = sqrt(residuals[0]);
            residuals[1] = sqrt(residuals[1]);
            return true;
        }

    private:
        // Observations for a sample.
        double *pts2_;
        const int N_;
        double K2_[9]{};
    };

    void BundleAdjustment(Mat K1, Mat M1, Mat p1,
                          Mat K2, Mat &M2, Mat p2,
                          Mat K3, Mat &M3, Mat p3,
                          Mat &pts3d) {
        double start_time = omp_get_wtime();

        K1.convertTo(K1, CV_64F);
        M1.convertTo(M1, CV_64F);
        p1.convertTo(p1, CV_64F);
        K2.convertTo(K2, CV_64F);
        M2.convertTo(M2, CV_64F);
        p2.convertTo(p2, CV_64F);
        K3.convertTo(K3, CV_64F);
        M3.convertTo(M2, CV_64F);
        p3.convertTo(p3, CV_64F);
        pts3d.convertTo(pts3d, CV_64F);

        cout << "Finished converting!" << endl;

        Mat t2 = M2.col(3);
        Mat r2;
        Rodrigues(M2.colRange(0, 3), r2);

        Mat t3 = M3.col(3);
        Mat r3;
        Rodrigues(M3.colRange(0, 3), r3);

        cout << "Finished rodrigues!" << endl;

        int num_points = p1.rows;
        const int MAX_NUM_POINT = 150;
        //using ceres for nonlinear optimization
        {
            ceres::Problem problem;
            ceres::CostFunction *cost_function_1 =
                    new ceres::AutoDiffCostFunction<BALResidual_1, 2, MAX_NUM_POINT * 3>(
                            new BALResidual_1((double *) p1.data, num_points, (double *) K1.data,
                                              (double *) M1.data));
            problem.AddResidualBlock(cost_function_1, nullptr, (double *) pts3d.data);

            ceres::CostFunction *cost_function_2 =
                    new ceres::AutoDiffCostFunction<BALResidual_2, 2, 3, 3, MAX_NUM_POINT * 3>(
                            new BALResidual_2((double *) p2.data, num_points, (double *) K2.data));
            problem.AddResidualBlock(cost_function_2, nullptr, (double *) r2.data, (double *) t2.data,
                                     (double *) pts3d.data);

            ceres::CostFunction *cost_function_3 =
                    new ceres::AutoDiffCostFunction<BALResidual_2, 2, 3, 3, MAX_NUM_POINT * 3>(
                            new BALResidual_2((double *) p3.data, num_points, (double *) K3.data));
            problem.AddResidualBlock(cost_function_3, nullptr, (double *) r3.data, (double *) t3.data,
                                     (double *) pts3d.data);

            cout << "Start solving..." << endl;

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.linear_solver_type = ceres::SPARSE_SCHUR;
            options.max_num_iterations = 1000;
            //options.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.FullReport() << endl;
        }
        Mat R2;
        Rodrigues(r2, R2);
        hconcat(R2, t2, M2);
        M2.convertTo(M2, CV_32F);

        Mat R3;
        Rodrigues(r3, R3);
        hconcat(R3, t3, M3);
        M3.convertTo(M3, CV_32F);

        pts3d.convertTo(pts3d, CV_32F);

        double end_time = omp_get_wtime();
        cout << "Finished BA in " << int(end_time - start_time) << "ms." << endl;
    }
}
