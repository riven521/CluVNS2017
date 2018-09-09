#include "Heuristic.h"
#include "Move.h"

Heuristic::Heuristic(CluVRPinst* cluVRPinst)
{
	this->cluVRPinst_ = cluVRPinst;
}

Heuristic::~Heuristic(){}

/******************************** BIN PACKING ****************************************/

//�̳���Heuristic;����:run;˽��;Redistribution��Diversification
BinPacking::BinPacking(CluVRPinst* cluVRPinst) : Heuristic(cluVRPinst)
{
	redistributionOperator_ = new Redistribution(cluVRPinst);
	diversOperator_ = new Diversification(cluVRPinst);
}

BinPacking::~BinPacking()
{
	delete redistributionOperator_;
	delete diversOperator_;
}

//Ӧ����װ���㷨����BF�ȣ���������bool��,ClusterSolution(��s)���ں����и���
bool BinPacking::run(ClusterSolution*& s)
{
	//��ȡinst�����е�Clusters����,������depot��cluster
	std::vector<Cluster*> vClientClusters = cluVRPinst_->getClientClusters();

	//sort clusters from by demand (large to small)
	// TODO �޸ĵݼ�����Ϊ���˳��
	sort(vClientClusters.begin(), vClientClusters.end(), Cluster::sortByDemandOperator);

	//create trip for every vehicle,ÿ��vehicle��Ӧһ��trip,�öν�������ʼdepot��trip�������trip��CluSolution
	for (int i = 0; i < this->cluVRPinst_->getnVehicles(); i++)
	{
		CluTrip* t = new CluTrip(cluVRPinst_);	//ÿ��Trip��������;dist;totalDemand;size
		
		//select random depot
		//ÿ������inst����vClusters_,���ܺ��ж��depot����,���ѡ��һ����Ϊ��ʼTrip�����
		t->addStop(cluVRPinst_->getRandomDepot());	//t��һ��CluTrip;addStop:����Cluster����CluTrip
		//add trip to the solution
		s->addTrip(t);	//s��һ��ClusterSolution,����Trip t
	}

	//add the clusters one by one
	int nRedistributionIterations = 0;

	//ѭ��ѡ��cluster���벻ͬtrip,�ŵ���s��,ֱ������
	//�̶�����c,ѡ��vehicle
	while (vClientClusters.size() > 0)
	{
		//get cluster to add
		//c��������ĳ��trip/veh��
		Cluster* c = vClientClusters.front();

		//get strategy
		double r = (double)rand() / RAND_MAX;
		bool success = false;	
		int veh;

		//���ݸ���,���ѡ��veh(Ҫ��ʣ�������㹻)��ѡ��൱ǰ��c�����veh(Ҫ��ʣ�������㹻,�Ҿ��������veh/vtrips)
		// (��������ͬ��?) -> ����ͬ,ѡ���һ��; �����Ͳ�ͬ, ��ѡ�����Գ���- ���ܲ�ͬ���ʹ�С��һ
		if (r < Params::RANDOM_CONSTRUCTION) veh = s->getRandomFeasibleVehicle(c,success);	
		else veh = s->getClosestFeasibleVehicle(c,success);	//RANDOM_CONSTRUCTIONĬ��Ϊ0,���ȫ��ѡ�������

		//perform move
		//�ж��Ƿ�����ҵ������㹻�ĳ�,����ҵ�,��c�����veh��Ӧ��trip;ɾ��c
		if (success)
		{
			s->getTrip(veh)->addStop(c);
			vClientClusters.erase(vClientClusters.begin());
		}
		else	//���δ�ҵ�,����Sec3.3��redistribution algorithm(SwapCluster),�����ƽ��޳���������(������ɹ�,���·Ÿþ���c;�粻�ɹ�����5��,ִ��diversOperator_;�粻�ɹ�����100��,����)
		{			
			redistributionOperator_->run(s);
			nRedistributionIterations++;
			
			if (nRedistributionIterations > 5)	//���redistribution��������5��,ִ��diversOperator(perturbationCluster + repairCluster)
			{
				if (nRedistributionIterations > 100) return false;	//���redistribution��������100��,����BF�㷨ִ��ʧ��
				diversOperator_->run(s);	//ִ��Sec3.5��
			}
		}
	}

	//return to the initial depot
	//����size�����ص�depot�ľ��뵽ClusterSolution��s��
	for (int i = 0; i < this->cluVRPinst_->getnVehicles(); i++)
	{
		s->getTrip(i)->returnToDepot();
	}

	//calc objective value of this solution
	//����ClusterSolution��s��Ŀ��ֵ,�ܾ���totalDist_,ÿ��trip�ľ���֮��
	s->calcTotalDist();

	return true;
}

/******************************* DIVERSIFICATION **************************************/
//��toAdd���Ƴ����������ӵ����s��
void Diversification::repairCluster(ClusterSolution*& s, std::vector<Cluster*> toAdd)
{
	//add clusters in random order
	random_shuffle(toAdd.begin(), toAdd.end());
	int nTry = 0;
	
	while (toAdd.size() > 0)
	{
		std::vector<int> feasVeh;

		//look for all feasible vehicles (capacity)
		do {
			//�ҳ��ܷ���toAdd���һ��node�����г�;����ʣ������>toAdd��back������
			for (int v = 0; v < s->getnTrips(); v++)
			{
				if (toAdd.back()->getDemand() <= s->getTrip(v)->getSpareCapacity())
				{
					//std::cout << toAdd.back()->getDemand() << " ";
					//std::cout << s->getTrip(v)->getSpareCapacity() << " ";
					feasVeh.push_back(v);
				}
			}
			//��û�г�:1�ٴ�perturbationCluster��ȡ�����toAdd;2:���Ӷ����trip/veh
			if (feasVeh.size() == 0)	//no feasible vehicles found
			{
				redistribution_->run(s);
				nTry++;
				if (nTry > 100)
				{
					//add an additional trip
					CluTrip* t = new CluTrip(cluVRPinst_);
					//add random depot
					t->addStop(cluVRPinst_->getRandomDepot());
					s->addTrip(t);
					cluVRPinst_->increasenVeh();

					continue;
				}
				else if (nTry > 5)
				{
					std::vector<Cluster*> removed;
					removed = perturbationCluster(s);
					toAdd.insert(toAdd.end(), removed.begin(), removed.end());
					continue;
				}
			}
			else break;

		} while (true);

		//choose one feasible vehicle at random
		//���ܷ����node�ĳ������ѡȡһ������
		int selection = rand() % feasVeh.size();
		s->getTrip(feasVeh.at(selection))->addStop(toAdd.back());
		toAdd.pop_back();
	}
	

	//return to the depot for every trip
	//����perturbation�Ƴ���depot,�˴�����Ϊÿ��trip����depot
	for (int v = 0; v < cluVRPinst_->getnVehicles(); v++)
	{
		s->getTrip(v)->addStop(s->getTrip(v)->getvClusters().front());
	}

	s->calcTotalDist();
}

//�Ƴ���clustersȫ������removedClusters(ÿ����·���ѡȡ����cluster�Ƴ�)
std::vector<Cluster*> Diversification::perturbationCluster(ClusterSolution*& s)
{
	std::vector<Cluster*> removedClusters;
	//ѭ��;ÿ����·�ֱ�Ѱ���Ƴ�����
	for (int v = 0; v < cluVRPinst_->getnVehicles(); v++)
	{
		//����������·,���Ƴ���·��β��depot
		if (s->getTrip(v)->getvClusters().front() == s->getTrip(v)->getvClusters().back())
		{
			s->getTrip(v)->removeStop(s->getTrip(v)->getSize() - 1);		//remove depot at the end of the trip
		}

		for (int i = 1; i < s->getTrip(v)->getSize(); i++)
		{
			//�����ǰ��s����·vֻ��0-2��,��ִ��perturbation.
			if (s->getTrip(v)->getSize() < 3)
			{
				break;
			}
			//������㼫С����,�Ƴ���ǰcluster-i,������removedClusters
			double r = (double)rand() / (RAND_MAX);
			if (r < Params::PERT_RATE)
			{
				removedClusters.push_back(s->getTrip(v)->getCluster(i));
				s->getTrip(v)->removeStop(i);
			}
		}
	}
	return removedClusters;
}

Diversification::Diversification(CluVRPinst* cluVRPinst) : Heuristic (cluVRPinst)
{
	redistribution_ = new Redistribution(cluVRPinst_);
}

Diversification::~Diversification() 
{
	delete redistribution_;
}

bool Diversification::run(ClusterSolution*& s)
{
	std::vector<Cluster*> toAdd;

	//�ƻ����޸�cluSolution;
	toAdd = perturbationCluster(s);
	repairCluster(s, toAdd);

	double r = (double)rand() / RAND_MAX;
	//�ж��Ƿ���Ҫִ��VNS1�㷨,false��ʾ��Ҫִ��
	if (r < Params::DIVERSIFICATION_1)
		return false;		//go to cluVNS
	else
		return true;		//go to conversion
}

/******************************* REDISTRIBUTION **************************************/
//3.3���е�cluster����
bool Redistribution::swapClusters(ClusterSolution*& cluSol)
{
	CluTrip* cluT1 = nullptr;
	CluTrip* cluT2 = nullptr;

	int minDif = BIG_M;
	bool success = false;
	bool flag = false;

	for (int v1 = 0; v1 < cluVRPinst_->getnVehicles(); v1++)
	{
		cluT1 = cluSol->getTrip(v1); //cluT1������v1��trips

		int spareCapacity = cluT1->getSpareCapacity();
		if (spareCapacity == 0) continue;

		for (int v2 = 0; v2 < cluVRPinst_->getnVehicles(); v2++)
		{
			if (v1 == v2) continue;

			cluT2 = cluSol->getTrip(v2);

			for (int i = 1; i < cluT1->getSize() - 1; i++)	//i������v1��Ӧ�ľ������
			{
				for (int j = 1; j < cluT2->getSize() - 1; j++)
				{
					if (cluT1->getCluster(i)->getDemand() == cluT2->getCluster(j)->getDemand()) continue;

					//diff:����swap���������Ƿ�����
					int diff = \
						+ spareCapacity\
						+ cluT1->getCluster(i)->getDemand()\
						- cluT2->getCluster(j)->getDemand();

					//��swap�����������ұ�minDifС,��ɽ���swap
					if (diff >= 0 && diff < minDif)
					{
						//�����������,��¼�¶�Ӧ�ľ���λ��p1��p2
						Pos p1(v1, i); //v1��veh���;i��ind���
						Pos p2(v2, j);
						if ((p1 == p1_ && p2 == p2_) || (p1 == p2_ && p2 == p1_)) continue;
						
						success = true;
						minDif = diff;
						p1_ = p1;
						p2_ = p2;
						if (minDif == 0)
						{
							flag = true;
							break;
						}
					}
				}
				if (flag) break;
			}
			if (flag) break;
		}
		if (flag) break;
	}
	//�������swap,ִ������code
	if (success)
	{
		//1 ����ÿ��vehӦ���ӻ���ٵ���c
		int c = \
			+ cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_)->getDemand()\
			- cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_)->getDemand();

		//2 ÿ��veh(trip)���ӻ���ٶ�Ӧ����c
		cluSol->getTrip(p1_.veh_)->altTotalDemand(-c);
		cluSol->getTrip(p2_.veh_)->altTotalDemand(c);

		//3 ����veh��dist�������¼���
		cluSol->getTrip(p1_.veh_)->altDist(\
			- cluVRPinst_->getDistClusters(cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_ - 1), cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_))\
			- cluVRPinst_->getDistClusters(cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_), cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_ + 1))\
			+ cluVRPinst_->getDistClusters(cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_ - 1), cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_))\
			+ cluVRPinst_->getDistClusters(cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_), cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_ + 1)));
		cluSol->getTrip(p2_.veh_)->altDist(\
			- cluVRPinst_->getDistClusters(cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_ - 1), cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_))\
			- cluVRPinst_->getDistClusters(cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_), cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_ + 1))\
			+ cluVRPinst_->getDistClusters(cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_ - 1), cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_))\
			+ cluVRPinst_->getDistClusters(cluSol->getTrip(p1_.veh_)->getCluster(p1_.ind_), cluSol->getTrip(p2_.veh_)->getCluster(p2_.ind_ + 1)));

		//4 ��ǰ��������ɺ�,ִ��Move�е�swap(λ�ý���)
		Move::swap(cluSol, p1_, p2_);
		return true;
	}
	else
	{
		return false;
	}
}

Redistribution::Redistribution(CluVRPinst* cluVRPinst) : Heuristic(cluVRPinst)
{
	p1_ = Pos(-1, -1);
	p2_ = Pos(-1, -1);
}

Redistribution::~Redistribution() {}

bool Redistribution::run(ClusterSolution*& cluSol)
{
	return swapClusters(cluSol);
}