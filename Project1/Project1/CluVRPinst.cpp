#include "CluVRPinst.h"
#include <iostream>

//**********************************************************************************************//

CluVRPinst::CluVRPinst(std::vector<Cluster> vClusters, int nVehicles)
{
	this->nVehicles_ = nVehicles;							//输入的车辆数为可用车辆数
	this->vehicleCapacity_ = Globals::vehicleCapacity;		//车辆容量是相同的

	this->vClusters_ = vClusters;
	this->nClusters_ = vClusters.size();
}

CluVRPinst::~CluVRPinst(){}

std::vector<Cluster*> CluVRPinst::getClientClusters(void)
{
	std::vector<Cluster*> v;
	for (int i = 0; i < nClusters_; i++)
	{
		if(!vClusters_.at(i).isDepot) v.push_back(&vClusters_.at(i));
	}
	return v;
}

Cluster* CluVRPinst::getRandomDepot(void)
{
	std::vector<Cluster*> v;

	for (int i = 0; i < nClusters_; i++)
	{
		if (vClusters_.at(i).isDepot) v.push_back(&vClusters_.at(i));
	}

	return v.at(rand() % v.size());
}

bool CluVRPinst::isFeasible(void)
{
	int totalDemand = 0;
	
	for (int i = 0; i < nClusters_; i++)
	{
		totalDemand += vClusters_.at(i).getDemand();
	}

	return totalDemand <= (nVehicles_ * vehicleCapacity_);
}