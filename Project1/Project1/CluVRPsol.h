//��:��Ϊ��,���շ��ؽ����ʽ,����������solution;clustersolution+nodeSolution
#pragma once
#include "Solution.h"
//����Ƚϼ�,һ������solution����ClusterSolution��NodeSolution����������solution����
struct CluVRPsol
{
	ClusterSolution* sCluster_;
	NodeSolution* sNode_;

	CluVRPsol(ClusterSolution*, NodeSolution*);
	~CluVRPsol();
};

