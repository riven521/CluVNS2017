#include "Cluster.h"

using namespace std;

Cluster::Cluster(int id)
{
	this->id_ = id;
	this->demand_ = 0;
	this->nNodes_ = 0;
	this->demand_ = 0;

	isDepot = false;
}

Cluster::~Cluster(){}

bool Cluster::sortByDemandOperator(Cluster*& a, Cluster*& b)
{
	return a->demand_ > b->demand_;
}

void Cluster::addNode(Node n)
{
	vNodes_.push_back(n);
	nNodes_++;
}



//template <typename TElem>
//ostream& operator<<(ostream& os, const vector<TElem>& vec) {
//	typedef vector<TElem>::const_iterator iter_t;
//	const iter_t iter_begin = vec.begin();
//	const iter_t iter_end = vec.end();
//	os << "[";
//	for (iter_t iter = iter_begin; iter != iter_end; ++iter) {
//		cout << ((iter != iter_begin) ? "," : "") << *iter;
//	}
//	os << "]" << endl;
//	return os;
//}
//ostream& operator<<(ostream& os, const Cluster* cluster)
//{
//	cout << "����ID��" << cluster->getId() << endl;
//	cout << "�����ڽڵ���:" << cluster->getnNodes() << endl;
//	cout << "������Demand:" << cluster->getDemand() << endl;
//	cout << "�����Ƿ�depot:" << cluster->isDepot << endl;
//
//
//	cout << "�ڵ�����Ϊ:" << endl;
//	//cout << cluster->vNodes_ << endl;
//
//	return os;
//}