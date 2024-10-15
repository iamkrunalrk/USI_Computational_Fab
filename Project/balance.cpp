#include "balance.h"
#include "make_voxels.h"

void make_it_stand(
	const Eigen::MatrixXd &V, 
	const Eigen::MatrixXi &F, 
	Eigen::MatrixXd &MiV, 
	Eigen::MatrixXi &MiF)
{
	MiV.resize(V.rows(), V.cols());
	MiF.resize(F.rows(), F.cols());

	MiV = V;
	MiF = F;

}