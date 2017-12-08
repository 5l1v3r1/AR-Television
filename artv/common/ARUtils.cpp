#include <common/ARUtils.h>

using namespace std;
using namespace cv;

namespace ar {
    using cv::Mat;

    /// Plot matched points between two images.
    void PlotMatches(const Mat& img1,
                     const Mat& img2,
                     const vector<cv::Point2f>& pts1,
                     const vector<cv::Point2f>& pts2,
                     Mat& out) {
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

        vector<Mat> res;
        res.reserve(4);
        Mat R1 = U * W * Vt;
        Mat R2 = U * W.t() * Vt;
        Mat t = U.col(2) / max({abs(U.at<float>(2, 0)), abs(U.at<float>(2, 1)), abs(U.at<float>(2, 2))});

        Mat candidate;
        hconcat(R1, t, candidate);
        res.push_back(candidate.clone());
        hconcat(R1, -t, candidate);
        res.push_back(candidate.clone());
        hconcat(R2, t, candidate);
        res.push_back(candidate.clone());
        hconcat(R2, -t, candidate);
        res.push_back(candidate);
        return res;
    }

    /// Calculate the relative rotation and translation from camera 1 to camera 2,
    //	given their own rotations and translations with respect to the world coordinate.
    pair<Mat, Mat> CalRelRotAndTranslation(Mat R1, Mat t1, Mat R2, Mat t2) {
        return {R1.t() * R2, R1.t() * (t2 - t1)};
    }

    Mat ComputeCameraMatrix(cv::Mat intrinsics, cv::Mat R, cv::Mat t) {
        Mat extrinsics;
        hconcat(R, t, extrinsics);
        return intrinsics * extrinsics;
    }

    /// Input a series of camera matrices and 2D points. The 2D points are all matched in order to relate to some 3D points.
    //	Output the estimation of 3D points and estimation error.
    ERROR_CODE Triangulate(const std::vector<std::pair<cv::Mat, cv::Mat>> &pts,
                           cv::Mat &points3d,
                           double *error) {
        int num_pts = -1;
        for (auto &p : pts) {
            if (num_pts < 0)
                num_pts = p.second.rows;
            else if (p.second.rows != num_pts)
                return AR_INVALID_INPUT;
        }

        // TODO: Estimate the 3D points with all the information.
        int N = pts[0].second.rows;
        auto n = pts.size();
        points3d = Mat(N, 3, CV_32F);

        for (int i = 0; i < N; ++i) {
            Mat A = Mat(static_cast<int>(n << 1), 4, CV_32F);
            for (int j = 0; j < n; ++j) {
                ((Mat) (pts[j].first.row(0) - pts[j].second.at<float>(i, 0) * pts[j].first.row(2))).copyTo(
                        A.row(2 * j));
                ((Mat) (pts[j].first.row(1) - pts[j].second.at<float>(i, 1) * pts[j].first.row(2))).copyTo(
                        A.row(2 * j + 1));
            }
            Mat U, W, VT, V;
            SVD svd;
            svd.compute(A, W, U, VT);
            Mat p = VT.row(V.cols);
            p = p / p.at<float>(0, 3);
            for (int k = 0; k < n; ++k) {
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

    struct BALResidual {
        BALResidual(double x2, double y2)
                : x2_(x2), y2_(y2) {}

        template<typename T>
        bool operator()(const T *const r2, //3
                        const T *const t2, //3
                        const T *const point, //3
                        const T *const K2, //9
                        T *residuals) const {
            Mat r2Mat(3, 1, CV_32F, (void *) r2);
            Mat R2;
            Rodrigues(r2Mat, R2);
            Mat M2;
            Mat t2Mat(3, 1, CV_32F, (void *) t2);
            hconcat(R2, t2Mat, M2);
            Mat C2 = Mat(3, 3, CV_32F, (void *) K2) * M2; //3*4
            Mat P_h;
            hconcat(Mat(3, 1, CV_32F, (void *) point), Mat(1, 1, CV_32F, 1.), P_h); //4*1
            Mat p2_proj = C2 * P_h; //3*1
            double p2_hat_x = p2_proj.at<double>(0, 0) / p2_proj.at<double>(2, 0);
            double p2_hat_y = p2_proj.at<double>(1, 0) / p2_proj.at<double>(2, 0);
            residuals[0] = T(p2_hat_x - x2_);
            residuals[1] = T(p2_hat_y - y2_);
            return true;
        }

    private:
        // Observations for a sample.
        const double x2_;
        const double y2_;
    };

    void BundleAdjustment(Mat K1, Mat M1, Mat pts1,
                          Mat K2, Mat &M2, Mat pts2,
                          Mat &Points3d) {
        Mat R2_init = M2.rowRange(0, 3).clone();
        Mat t2_init = M2.rowRange(3, 4).clone();
        Mat r2_init;
        Rodrigues(R2_init, r2_init);

        int N = pts1.rows;
        double r2[3], t2[3];
        double K2array[9];
        for (int i = 0; i < 3; i++) {
            r2[i] = r2_init.at<double>(i, 0);
            t2[i] = t2_init.at<double>(i, 0);
        }
        for (int i = 0; i < 9; i++) {
            K2array[i] = K2.at<double>(i / 3, i % 3);
        }
        //using ceres to nonlinear optimization
        {
            ceres::Problem problem;
            for (int i = 0; i < N; ++i) {
                double pt3d[3];
                for (int j = 0; j < 3; j++)
                    pt3d[j] = Points3d.at<double>(i, j);
                ceres::CostFunction *cost_function =
                        new ceres::AutoDiffCostFunction<BALResidual, 2, 3, 3, 3, 9>(
                                new BALResidual(pts2.at<double>(i, 0), pts2.at<double>(i, 1)));
                problem.AddResidualBlock(cost_function, NULL, r2, t2, pt3d, K2array);
            }
            ceres::Solver::Options options;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
        }
        Mat R2;
        Rodrigues(r2_init, R2);
        return;
    }
}
