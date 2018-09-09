#include "CluVRPinst.h"
#include <iostream>



//**********************************************************************************************//

CluVRPinst::CluVRPinst(std::vector<Cluster> vClusters, int nVehicles)
{
	this->nVehicles_ = nVehicles;							//����ĳ�����Ϊ���ó�����
	this->vehicleCapacity_ = Globals::vehicleCapacity;		//������������ͬ��

	this->vClusters_ = vClusters;
	this->nClusters_ = vClusters.size();
}

CluVRPinst::~CluVRPinst(){}

// ��ȡ�ų�Depot��ľ���
std::vector<Cluster*> CluVRPinst::getClientClusters(void)
{
	std::vector<Cluster*> v;
	for (int i = 0; i < nClusters_; i++)
	{
		if(!vClusters_.at(i).isDepot) v.push_back(&vClusters_.at(i));
	}
	return v;
}
// ���ж��depot,���ѡ��һ��
Cluster* CluVRPinst::getRandomDepot(void)
{
	std::vector<Cluster*> v;

	for (int i = 0; i < nClusters_; i++)
	{
		if (vClusters_.at(i).isDepot) v.push_back(&vClusters_.at(i));
	}

	return v.at(rand() % v.size());//���ѡ��һ��
}

// ���ж����������Ƿ���������������
bool CluVRPinst::isFeasible(void)
{
	int totalDemand = 0;
	
	for (int i = 0; i < nClusters_; i++)
	{
		totalDemand += vClusters_.at(i).getDemand();
	}

	return totalDemand <= (nVehicles_ * vehicleCapacity_);
}


// ��������<<������
template <typename TElem>
ostream& operator<<(ostream& os, const vector<TElem>& vec) {
	typedef vector<TElem>::const_iterator iter_t;
	const iter_t iter_begin = vec.begin();
	const iter_t iter_end = vec.end();
	os << "[";
	for (iter_t iter = iter_begin; iter != iter_end; ++iter) {
		cout << ((iter != iter_begin) ? "," : "") << *iter;
	}
	os << "]" << endl;
	return os;
}

ostream& operator<<(ostream& os, const CluVRPinst* cluVRPinst)
{
	cout << "�����������£�" << endl;
	cout << "��������" << cluVRPinst->nClusters_ << " ";
	cout << "��������:" << cluVRPinst->vehicleCapacity_ << " ";
	//cout << "�����㹻?:" << cluVRPinst->isFeasible << " ";
	cout << "������:" << cluVRPinst->nClusters_ << endl << endl;
	

	cout << "�ڵ�����Ϊ:" << endl;
	cout << cluVRPinst->distNodes_ << endl;

	cout << "�����Handoff����Ϊ:" << endl;
	cout << cluVRPinst->distClusters_ << endl;

	cout << "ʵ����:�������ÿ������:" << endl;
	//cout << (cluVRPinst->vClusters_) << endl;
	vector<Cluster>::const_iterator it = cluVRPinst->vClusters_.begin();
	for (; it != cluVRPinst->vClusters_.end(); ++it)
	{
		//cout << "����ID��" << it->show() << endl;
		cout << "����ID��" << it->getId() << " ";
		cout << "�����ڽڵ���:" << it->getnNodes() << " ";
		cout << "������Demand:" << it->getDemand() << " ";
		cout << "�����Ƿ�depot:" << it->isDepot <<  " ";
		cout << "�����ڽڵ���Ϣδ���" << endl;		
	}
	return os;
}

