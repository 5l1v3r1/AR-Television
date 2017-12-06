#include <common/ARUtils.h>

using namespace std;
using namespace cv;

namespace ar {
	Mat RefineFundamentalMatrix(const Mat& fundamental_matrix,
		const vector<Point2d>& point1,
		const vector<Point2d>& point2) {
		// TODO: Locally minimize a geometric cost function.
		

		// Enforce singularity.
		auto svd = SVD(fundamental_matrix);
		auto Sigma = svd.w;
		Sigma.at<double>(Sigma.cols - 1, Sigma.rows - 1) = 0;
		return svd.u * Sigma * svd.vt;
	}

	//! Recover rotation and translation from an essential matrix.
	//	This operation produces four posible results, each is a 3x4 matrix that combines
	//	rotation and translation.
	vector<Mat> RecoverRotAndTranslation(const Mat& essential_matrix) {
		Mat U, W, Vt;
		auto svd = SVD();
		svd.compute(essential_matrix, W, U, Vt);
		W = Mat::diag(W);

		float m = (W.at<float>(1, 1) + W.at<float>(2, 2)) / 2;
		W.at<float>(0, 0) = W.at<float>(1, 1) = m;
		Mat regularized_E = U * W * Vt;
		svd.compute(regularized_E, W, U, Vt);
		W = Mat::diag(W);

		W.at<float>(0, 0) = W.at<float>(1, 1) = 0;
		W.at<float>(0, 1) = -1;
		W.at<float>(1, 0) = W.at<float>(2, 2) = 1;
		if (determinant(U * W * Vt) < 0)
			W = -W;

		vector<Mat> res;
		res.reserve(4);
		Mat R1 = U * W * Vt;
		Mat R2 = U * W.t() * Vt;
		Mat t = U.col(2) / max({ abs(U.at<float>(2, 0)), abs(U.at<float>(2, 1)), abs(U.at<float>(2, 2)) });

		Mat candidate;
		hconcat(R1, t, candidate);
		res.push_back(candidate);
		hconcat(R1, -t, candidate);
		res.push_back(candidate);
		hconcat(R2, t, candidate);
		res.push_back(candidate);
		hconcat(R2, -t, candidate);
		res.push_back(candidate);
		return res;
	}

	//! Calculate the relative rotation and translation from camera 1 to camera 2,
	//	given their own rotations and translations with respect to the world coordinate.
	pair<Mat, Mat> CalRelRotAndTranslation(Mat R1, Mat t1, Mat R2, Mat t2) {
		return { R1.t() * R2, R1.t() * (t2 - t1) };
	}

	//! Input a series of camera matrices and 2D points. The 2D points are all matched in order to relate to some 3D points.
	//	Output the estimation of 3D points and estimation error.
	ERROR_CODE triangulate(const std::vector<std::pair<cv::Mat, cv::Mat>>& pts,
						   cv::Mat& points3d,
						   double* error) {
		int num_pts = -1;
		for (auto& p : pts) {
			if (num_pts < 0)
				num_pts = p.second.rows;
			else if (p.second.rows != num_pts)
				return AR_INVALID_INPUT;
		}
        
        // TODO: Estimate the 3D points with all the information.
        int N = pts[0].second.rows;
        int n = pts.size();
        points3d = Mat(N, 3, CV_32F);
        *error = 0;
        cout << "Estimating 3D Points" << endl;
        
        for (int i = 0; i < N; ++i) {
            Mat A = Mat(2*n, 4, CV_32F);
//            cout << pts[0].first << endl;
            for (int j = 0; j < n; ++j) {
                Mat(pts[j].first.row(0) - pts[j].second.at<float>(i, 0) * pts[j].first.row(2)).copyTo(A.row(2 * j));
                Mat(pts[j].first.row(1) - pts[j].second.at<float>(i, 1) * pts[j].first.row(2)).copyTo(A.row(2 * j + 1));
            }
//            cout << A << endl;
            Mat U, W, VT, V;
            SVD svd;
            svd.compute(A, W, U, VT);
            Mat p = VT.row(V.cols);
            p = p / p.at<float>(0, 3);
            for (int k = 0; k < n; ++k) {
                Mat proj = p * pts[k].first.t();
                proj = proj.colRange(0, 2) / proj.colRange(2, 2);
                Mat diff = proj - pts[k].second.row(i);
                double err = diff.at<float>(0, 0) * diff.at<float>(0, 0) + diff.at<float>(0, 1) * diff.at<float>(0, 1);
                *error += err;
            }
            p.colRange(0, 3).copyTo(points3d.row(i));
        }
        return AR_SUCCESS;
	}
}
