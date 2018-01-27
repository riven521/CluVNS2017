#pragma once
#include "Solution.h"
//该类比较简单,一个总体solution包含ClusterSolution和NodeSolution两个单独的solution构成
struct CluVRPsol
{
	ClusterSolution* sCluster_;
	NodeSolution* sNode_;

	CluVRPsol(ClusterSolution*, NodeSolution*);
	~CluVRPsol();
};

