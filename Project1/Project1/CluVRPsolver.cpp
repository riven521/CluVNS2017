#include "CluVRPsolver.h"

#include <time.h>

CluVRPsolver::CluVRPsolver()
{
	params_ = new Params();	//Ĭ��ȫ�ֲ�����ֵ

	sCluCurrent_ = nullptr;
	sCluBest_ = nullptr;
	sNodCurrent_ = nullptr;
	sNodBest_ = nullptr;
}

CluVRPsolver::~CluVRPsolver()
{
	delete params_;
}

CluVRPsol* CluVRPsolver::solve(CluVRPinst* cluVRPinst, Timer* timer)
{
	//�������:�������������<����������,��������������,��instance��infeasible
	if (!cluVRPinst->isFeasible()) {
		std::cout << "instance infeasible" << std::endl; return nullptr;
	}

	//��ʼ��:��ֵ��ָ���cluVRPsol
	CluVRPsol* cluVRPsol = nullptr;

	//create methods
	BPheuristic_ = new BinPacking(cluVRPinst);

	//keep track of the best solutions
	//���屾���˽��,����Best����������Ҫ��
	sCluBest_ = nullptr;
	sNodBest_ = new NodeSolution(cluVRPinst, BIG_M);//��ȡ����NodeSolution:��������+totalDist_=BIG_M����ֵ; 
	
	sCluCurrent_ = new ClusterSolution(cluVRPinst);	//��ǰ��ClusterSolution,Ӧ����ʱΪ��
	
	//try to construct feasible solution at cluster level
	//3.2;��������+�Դ����,���ӳ�����,ֱ��BPheuristic_.run���м���󷵻�Ture
	while (!BPheuristic_->run(sCluCurrent_)) 
	{
		delete sCluCurrent_; sCluCurrent_ = nullptr;
		cluVRPinst->increasenVeh();
		sCluCurrent_ = new ClusterSolution(cluVRPinst);
	}

	//create methods
	cluVNS_ = new CluVNS(cluVRPinst);	//��ʼ��cluVNS->nbhSequenceΪ����,0-6
	nodVNS_ = new NodVNS(cluVRPinst, false);
	diversOperator_ = new Diversification(cluVRPinst);

	//start local search
	bool goToNodeVNS = false;
	bool stoppingCriterion = false;
	int nIterationsWithoutImprovement = 0;

	do
	{
		//VNS1:��sCluCurrent_ת��ΪVNS��sCluCurrent_,�����¼���Ŀ�����ֵ
		cluVNS_->run(sCluCurrent_);

		//Convert;��sCluCurrent_��ȡ��Ӧ��NodeSolution;sNodCurrent_
		sNodCurrent_ = sCluCurrent_->convert();

		do
		{			
			//VNS2:��sNodCurrent_ת��ΪVNS��sNodCurrent_
			nodVNS_->run(sNodCurrent_);

			//evaluate node solution ������ǰsNodCurrent_�Ƿ�С��sNodBest_,������ͬʱ����sCluBest_+nIterationsWithoutImprovement
			if (sNodCurrent_->evaluate(sNodBest_))	//555 ��ȡbest:sNodBest_��sCluBest_
			{
				timer->setTimeBest(timer->getIntervalTime());
				//keep also the cluster variant of the best node solution
				if (sCluBest_ != nullptr) delete sCluBest_;
				sCluBest_ = sNodBest_->convert();
				//reset counter
				nIterationsWithoutImprovement = 0;

				nodVNS_->disableAdditionalNBHs();
			}
			else  //�޸Ľ�������nIterationsWithoutImprovement;������VNS1ѭ��
			{
				delete sNodCurrent_; sNodCurrent_ = nullptr;

				nIterationsWithoutImprovement++;
				
				if (nIterationsWithoutImprovement % 100 == 0)	//every 100th iteration
				//if (nIterationsWithoutImprovement + 1 == params_->N_NO_IMPROVEMENT)	//the last iteration
				{
					//use all the power we have to improve the current best solution
					sNodCurrent_ = new NodeSolution(sNodBest_);
					nodVNS_->enableAdditionalNBHs();	//try complex neighbourhoods
					goToNodeVNS = true;
					continue;
				}				
				else if (nIterationsWithoutImprovement >= params_->N_NO_IMPROVEMENT) //check stopping criterion
				{
					stoppingCriterion = true;
					break;
				}
				else
					nodVNS_->disableAdditionalNBHs();
			}
			
			//diversification before new round of improvement
			//Diversification�׶�:�����Ƿ��иĽ�,��ִ�����²���.
			goToNodeVNS = diversOperator_->run(sCluCurrent_);	//DIVERSIFICATION_1=1�����ж�goToNodeVNS������,ȫ��Ϊfalse:go to cluVNS
			if(goToNodeVNS) sNodCurrent_ = sCluCurrent_->convert();	//���Ϊtrue,��Cluת��ΪNodes,ѭ��VNS2;Ϊfalse:��ѭ��VNS1;

		} while (goToNodeVNS);

	//} while (timer->getIntervalTime() < CALC_TIME);
	} while (!stoppingCriterion);

	//free memory
	delete BPheuristic_;
	delete cluVNS_;
	delete nodVNS_;
	delete diversOperator_;
	delete sCluCurrent_;

	//create solution object
	//��ȡ���յĽ�:cluVRPsol:���������ӽ�
	cluVRPsol = new CluVRPsol(sCluBest_, sNodBest_);
	return cluVRPsol;	
}