#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// We can also use "igl/centroid.h" for the same task
void center_of_mass(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    double mass,
    Eigen::Vector3d &center
)

void alignVerticesToAxis(
    const Eigen::MatrixXd &V, 
    Eigen::MatrixXd &alignedV
) 

void createVoxelGrid(
	const Eigen::MatrixXd& V,
	Eigen::MatrixXd& centers,
	Eigen::MatrixXd& corners,
	Eigen::Vector3i& dimensions,
	Eigen::Vector3d& voxelSize
);

void transformVertices(
	const Eigen::MatrixXd& V, 
	const Eigen::AlignedBox3d &newOrientation,  
	bool keepSize, 
	Eigen::MatrixXd& Vout
);

void transformVertices(
	const Eigen::MatrixXd& V, 
	const Eigen::AlignedBox3d &newOrientation, 
	Eigen::MatrixXd& Vout
);

void createAlignedBox(
	const Eigen::MatrixXd& V, 
	Eigen::AlignedBox3d& box
);
