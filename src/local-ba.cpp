#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
//#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
//#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "local-ba.h"
#include <opencv2/core/core.hpp>
#include <Eigen/StdVector>

using namespace std;

// column size for keyframe
typedef Eigen::Matrix<float, 1, 12> KeyFrameMatrix;
typedef Eigen::Matrix<float, 1, 3> MapPointMatrix;
typedef Eigen::Matrix<float, 1, 4> KeyFrameMapPointMatrix;

//index - key
typedef std::pair<  std::pair<int, int>, KeyFrameMatrix> KeyFrame;
typedef std::pair< std::pair<int, int>, MapPointMatrix> MapPoint;


///new URB
//keyframes : All keyframes
//worldMapPoints : All the mappoint in worldcoordinates
//keyframeWorldMapPoint: Keyframes and relation with
int LocalBundleAdjustment(Eigen::MatrixXf keyframes, Eigen::MatrixXf worldMapPoints, Eigen::MatrixXf keyframeWorldMapPoint) {

	//primary keyframe
	KeyFrame primaryKeyframe;
	Eigen::MatrixXf primKeyFrame(1, keyframes.cols());
	primKeyFrame << keyframes.row(primIndex);
	cout << primKeyFrame << std::endl;
	primaryKeyframe = std::make_pair(std::make_pair(0, 10), primKeyFrame);


	//Prep Data
	//step 1    //Set relationships with keyframe that share the same points
	std::vector<int> keyMapPointsIndex;

	//frames with same mappoints as the keyframe
	std::vector<KeyFrame> lLocalKeyFrames;

	for (int n = 1; n < keyframes.rows(); n++) {
		Eigen::MatrixXf currentFrame(1, keyframes.cols());
		currentFrame << keyframes.row(n);
		cout << currentFrame << std::endl;
		KeyFrame frame = std::make_pair(std::make_pair(n, primaryKeyframe.first.second), currentFrame);
		lLocalKeyFrames.push_back(frame);
	}


	//step 2 Local MapPoints seen in Local KeyFrames
	std::vector<MapPoint> lLocalMapPoints;
	std::vector<int> indexUsed;
	for (KeyFrame frameMatrix : lLocalKeyFrames)
	{
		vector<MapPoint> vpMPs;
		for (int k = 0; k < keyframeWorldMapPoint.size(); k++) {
			Eigen::MatrixXf currentFrame(1, keyframes.cols());
			currentFrame << keyframeWorldMapPoint.row(k);
			if (currentFrame(0) == frameMatrix.first.first) {
				int currentIndex = currentFrame(1);  //index map

				bool found = false;
				for (int indexInArray : indexUsed) {
					if (indexInArray == currentIndex) {
						found = true;
						break;
					}
				}

				if (found == false) {
					Eigen::MatrixXf currentPoint(1, keyframes.cols());
					currentPoint << worldMapPoints.row(currentIndex);
					MapPoint point = std::make_pair(std::make_pair(currentIndex, primaryKeyframe.first.second), currentPoint);
					lLocalMapPoints.push_back(point);
				}
			}
		}


		//TODO
		// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
		//    vector<KeyFrameMatrix> lFixedCameras;
		//    for( MapPointMatrix localMapPoint : lLocalMapPoints ) {
		//        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
		//        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
		//        {
		//            KeyFrame* pKFi = mit->first;
		//
		//            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
		//            {
		//                pKFi->mnBAFixedForKF=pKF->mnId;
		//                if(!pKFi->isBad())
		//                    lFixedCameras.push_back(pKFi);
		//            }
		//        }
		//    }


		///Optimizer

		// Setup optimizer
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);



		unsigned long maxKFid = 0;

		// Set Local KeyFrame vertices
		for (KeyFrame frame : lLocalKeyFrames)
		{
			KeyFrame pKFi = frame;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(pKFi.second);
			vSE3->setId(pKFi.first.second);
			if (pKFi == primaryKeyframe) {
				vSE3->setFixed(true);
			}
			else {
				vSE3->setFixed(false);
			}
			optimizer.addVertex(vSE3);
			if (pKFi->mnId>maxKFid)
				maxKFid = pKFi->mnId;
		}



		return 0;
	}

