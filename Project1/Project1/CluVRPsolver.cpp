#include "CluVRPsolver.h"

#include <time.h>

CluVRPsolver::CluVRPsolver()
{
	params_ = new Params();	//默认全局参数赋值

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
	//如果车辆总容量<聚类总需求,即给定车辆不够,则instance是infeasible
	if (!cluVRPinst->isFeasible()) {
		std::cout << "instance infeasible" << std::endl; return nullptr;
	}

	//赋值空指针给cluVRPsol
	CluVRPsol* cluVRPsol = nullptr;

	//create methods
	BPheuristic_ = new BinPacking(cluVRPinst);

	//keep track of the best solutions
	//定义本类的私有,两个Best是我们最需要的
	sCluBest_ = nullptr;
	sNodBest_ = new NodeSolution(cluVRPinst, BIG_M);//获取最优NodeSolution:包含算例+totalDist_=BIG_M极大值; 
	
	sCluCurrent_ = new ClusterSolution(cluVRPinst);	//当前的ClusterSolution,应该暂时为空
	
	//try to construct feasible solution at cluster level
	//试错过程,增加车辆数,直到BPheuristic_.run进行计算后返回Ture
	while (!BPheuristic_->run(sCluCurrent_)) 
	{
		delete sCluCurrent_; sCluCurrent_ = nullptr;
		cluVRPinst->increasenVeh();
		sCluCurrent_ = new ClusterSolution(cluVRPinst);
	}

	//create methods
	cluVNS_ = new CluVNS(cluVRPinst);	//初始化cluVNS->nbhSequence为数组,0-6
	nodVNS_ = new NodVNS(cluVRPinst, false);
	diversOperator_ = new Diversification(cluVRPinst);

	//start local search
	bool goToNodeVNS = false;
	bool stoppingCriterion = false;
	int nIterationsWithoutImprovement = 0;

	do
	{
		//将sCluCurrent_进步一转换为VNS的sCluCurrent_,并重新计算目标距离值
		cluVNS_->run(sCluCurrent_);
		//由sCluCurrent_获取对应的NodeSolution;sNodCurrent_
		sNodCurrent_ = sCluCurrent_->convert();

		do
		{			
			//将sNodCurrent_进步一转换为VNS的sNodCurrent_
			nodVNS_->run(sNodCurrent_);

			//evaluate node solution 评估当前sNodCurrent_是否小于sNodBest_,如是则同时更新sCluBest_;nIterationsWithoutImprovement
			if (sNodCurrent_->evaluate(sNodBest_))	//555 获取best:sNodBest_和sCluBest_
			{
				timer->setTimeBest(timer->getIntervalTime());
				//keep also the cluster variant of the best node solution
				if (sCluBest_ != nullptr) delete sCluBest_;
				sCluBest_ = sNodBest_->convert();
				//reset counter
				nIterationsWithoutImprovement = 0;

				nodVNS_->disableAdditionalNBHs();
			}
			else
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
			goToNodeVNS = diversOperator_->run(sCluCurrent_);	//DIVERSIFICATION_1=1参数判断goToNodeVNS的类型,全部为false:go to cluVNS
			if(goToNodeVNS) sNodCurrent_ = sCluCurrent_->convert();	//如果未true,则go to conversion;不通过步骤cluVNS_->run(sCluCurrent_);

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
	//获取最终的解:cluVRPsol:包含两个子解
	cluVRPsol = new CluVRPsol(sCluBest_, sNodBest_);
	return cluVRPsol;	
}