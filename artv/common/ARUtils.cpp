#include <common/ARUtils.hpp>

using namespace std;
using namespace cv;

namespace ar {
	Mat RefineFundamentalMatrix(const Mat& fundamentalMatrix,
		const vector<Point2d>& point1,
		const vector<Point2d>& point2) {
		// TODO: Locally minimize a geometric cost function.
		

		// Enforce singularity.
		auto svd = SVD(fundamentalMatrix);
		auto Sigma = svd.w;
		Sigma.at<double>(Sigma.cols - 1, Sigma.rows - 1) = 0;
		return svd.u * Sigma * svd.vt;
	}
}