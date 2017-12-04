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
		svd.compute(essential_matrix, U, W, Vt);

		double m = (W.at<double>(1, 1) + W.at<double>(2, 2)) / 2;
		W.at<double>(0, 0) = W.at<double>(1, 1) = m;
		Mat regularized_E = U * W * Vt;
		svd.compute(regularized_E, U, W, Vt);

		W.at<double>(0, 0) = W.at<double>(1, 1) = 0;
		W.at<double>(0, 1) = -1;
		W.at<double>(1, 0) = W.at<double>(2, 2) = 1;
		if (determinant(U * W * Vt) < 0)
			W = -W;

		vector<Mat> res;
		res.reserve(4);
		Mat R1 = U * W * Vt;
		Mat R2 = U * W.t() * Vt;
		Mat t = U.col(2) / max({ abs(U.at<double>(2, 0)), abs(U.at<double>(2, 1)), abs(U.at<double>(2, 2)) });
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
	ERROR_CODE triangulate(const std::vector<std::pair<cv::Mat, cv::Mat>>& camera_matrices_and_2d_points,
						   cv::Mat& points3d,
						   int* error) {
		int num_pts = -1;
		for (auto& p : camera_matrices_and_2d_points) {
			if (num_pts < 0)
				num_pts = p.second.rows;
			else if (p.second.rows != num_pts)
				return AR_INVALID_INPUT;
		}

		// TODO: Estimate the 3D points with all the information.
	}
}