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
	vector<pair<Mat, Mat>> RecoverRotationAndTranslation(const Mat& essential_matrix) {
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

		vector<pair<Mat, Mat>> res;
		res.reserve(4);
		Mat R1 = U * W * Vt;
		Mat R2 = U * W.t() * Vt;
		Mat t = t / max({ abs(U.at<double>(2, 0)), abs(U.at<double>(2, 1)), abs(U.at<double>(2, 2)) });
		res.push_back({ R1, t });
		res.push_back({ R1, -t });
		res.push_back({ R2, t });
		res.push_back({ R2, -t });
		return res;
	}

	//! Calculate the relative rotation and translation from camera 1 to camera 2,
	//	given their own rotations and translations with respect to the world coordinate.
	pair<Mat, Mat> CalculateRelativeRotationAndTranslation(Mat R1, Mat t1, Mat R2, Mat t2) {
		return { R1.t() * R2, R1.t() * (t2 - t1) };
	}
}