//  Portions Copyright 2013 Lingyun meng (lymeng@bjtu.edu.cn) and Xuesong Zhou (xzhou99@gmail.com)

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of FastTrain.

//    FastTrain is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    FastTrain is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with FastTrain.  If not, see <http://www.gnu.org/licenses/>.

// FastTrain.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "Network.h"
#include "FastTrain.h"
using namespace std;


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern CString g_GetAppRunningTime();


extern int g_NumberOfIterationsWithMemory;
extern int g_MaxLinkSize;
extern int g_MaxNodeSize;

extern double globalupperbound;
extern float g_MinimumStepSize;

extern CTime g_AppStartTime;
extern CTime g_SolutionStartTime;


bool g_UpdateLinkResourcePriceByForbiddenLinkIDs(int *pFLinkIDs,int direction)
{
	ASSERT(pFLinkIDs!=NULL);
	ASSERT(direction==0||direction==1);
	int linksize = g_LinkSet.size();
	CLink* pl = NULL;
	for (int i=0;linksize;i++)
	{
		if (pFLinkIDs[i]==-1)
		{
			break;
		}
		pl = g_LinkIDMap[pFLinkIDs[i]];
		ASSERT(pl!=NULL);
		
		for (int t=0;t<g_OptimizationHorizon;t+=g_OptimizationInterval)
		{
			if (direction==0)
			{			
				pl->m_ResourceAry[t].tempprice = pl->m_ResourceAry[t].Price;
				pl->m_ResourceAry[t].Price = MAX_SPLABEL;
			} 
			if (direction==1)
			{
				pl->m_ResourceAry[t].Price = pl->m_ResourceAry[t].tempprice;
			}
			 
		}
	}
	return true;
	
}

bool g_UpdateResourcePrices(int StartTime,int EndTime,double StepSize,int CurrentIterationNumber,int NumberOfIterationsWithMemory)
{
	ASSERT(StartTime>=0&&EndTime<=g_OptimizationHorizon);
	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		for(int t=StartTime; t< EndTime; t++)
		{				
			// if the total usage (i.e. resource consumption) > capacity constraint
			// then the resource price increases, otherwise decreases

			int capexce = (*iLink)->m_ResourceAry[t].UsageCount - (*iLink)->m_LinkCapacity[t];
			if (capexce>0)
			{
				(*iLink)->m_ResourceAry[t].Price  += StepSize*capexce*1;
			}
			else
			{
				(*iLink)->m_ResourceAry[t].Price  += StepSize*capexce*1;
			}

			//keep the resource price>=0. if the resource is not used within the past 5 iterations, then set the price to 0
			int lastiternum =  (*iLink)->m_ResourceAry[t].LastUseIterationNo;
			double pr = (*iLink)->m_ResourceAry[t].Price;
			if(pr < 0)//ensure the price is larger than 0
			{
				(*iLink)->m_ResourceAry[t].Price = 0;
			}
			if ((CurrentIterationNumber -lastiternum) > NumberOfIterationsWithMemory&&(*iLink)->m_LinkCapacity[t]>0)
			{
				(*iLink)->m_ResourceAry[t].Price = 0;
			}
		}
	}
	return true;
}


bool g_UpdateMaxNodeandLinkSizebyTrain(CTrain* pTrain)
{

	std::list<CLink*>::iterator iterLink;
	std::list<TrainPath*>::iterator iterPath;
	int locallinkcounter = 0;
	int localnodecounter = 0;


	//update m_MPLinkID and m_MPNodeID
	for (iterPath = pTrain->m_TrainPathList.begin();iterPath!=pTrain->m_TrainPathList.end();iterPath++)
	{
		for(iterLink = (*iterPath)->m_LinkList.begin(); iterLink != (*iterPath)->m_LinkList.end(); iterLink++)
		{				
			//update link counter
			
			locallinkcounter++;

			//update node counter
			if (iterLink == (*iterPath)->m_LinkList.begin())
			{
			
				localnodecounter++;		
				localnodecounter++;
			}
			else
			{
				localnodecounter++;
			}
		}

	}


	//update m_linksize and m_nodesize
	int LinkSize = locallinkcounter+pTrain->m_TrainPathList.size()*2;
	int NodeSize = localnodecounter+2;

	g_MaxLinkSize = max(LinkSize,g_MaxLinkSize);
	g_MaxNodeSize = max(NodeSize,g_MaxNodeSize);

	return true;
}


bool g_UpdateMaximumNodeandLinkSize(std::vector<CTrain*> TrainVector)
{
	for(int v = 0; v<g_TrainVector.size(); v++)
	{
		CTrain* pTrain = TrainVector[v];
		g_UpdateMaxNodeandLinkSizebyTrain(pTrain);
	}
	return true;
}


bool g_Timetable_Optimization_Lagrangian_Method()
{
	cout << "Preparation......"<< endl;

	NetworkForSP* m_pNetwork =NULL;

	int NumberOfIterationsWithMemory = g_NumberOfIterationsWithMemory; // this is much greater than -100 when  LR_Iteration = 0, because  LastUseIterationNo is initialized as -100;

	int OptimizationHorizon = g_OptimizationHorizon;

	std::set<CLink*>::iterator iLink;
	std::list<CLink*>::iterator iLink2;

	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		// reset resource usage counter for each time stamp
		(*iLink)->ResetResourceAry(g_OptimizationHorizon);//price of link is set to be 0
	}

	float **templinkprice =AllocateDynamicArray<float>(g_LinkSet.size(),g_OptimizationHorizon); //temporary storage of link prices. for path_based scheduling usage only
	CLink*pLink = NULL;                                                                                     //because m_LinkTDCostAry is changed by virtual links and nodes
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)//initialization: all temporary link prices to 0
	{
		for (int t=0;t<g_OptimizationHorizon;t++)
		{
			if ((*iLink)->m_LinkCapacity[t]==0)
			{
				templinkprice[(*iLink)->m_LinkID][t]= MAX_SPLABEL;
				(*iLink)->m_ResourceAry[t].Price = MAX_SPLABEL;
				pLink = (*iLink);
			}
			else
			{
				templinkprice[(*iLink)->m_LinkID][t] = 0;
				(*iLink)->m_ResourceAry[t].Price = 0;
			}			
		}
	}	

	cout << "Start scheduling trains by Lagrangian Relaxation method"<< endl;
	cout << "Running Time:" << g_GetAppRunningTime()  << endl;
	g_SolutionStartTime = CTime::GetCurrentTime();
	if (g_MaxNumberOfLRIterations>0)//need iteration
	{
		//build time-dependent network with resource price, here we allocate OptimizationHorizon time and cost labels for each node
			int linksize = 0;
			int nodesize = 0;
		    if (g_PathBasedScheduling)//update g_MaxNodeSize and g_MaxLinkSize by the number of nodes/links in the super network
				                      //which is generated by the physical selected paths of some train
			{
				g_UpdateMaximumNodeandLinkSize(g_TrainVector);
				nodesize = g_MaxNodeSize;
				linksize = g_MaxLinkSize;
			}
			else
			{
				nodesize = g_NodeSet.size();
				linksize = g_LinkSet.size();
			}
					
			m_pNetwork = new NetworkForSP(nodesize, linksize, OptimizationHorizon, g_OptimizationInterval);  		
				
	} 
	else
	{
		if (templinkprice!=NULL)
		{
			DeallocateDynamicArray<float>(templinkprice,g_LinkSet.size(),g_OptimizationHorizon);
			templinkprice = NULL;
		}
		return true;
	}
	
    double globallowerbound=-999999;

	//loop for each LR iteration
	int StartTime=0;
	int EndTime=0;
	int PreStartTime=0;
	int PreEndTime=0;
	int localcounter=0;

	for(int LR_Iteration = 0; LR_Iteration< g_MaxNumberOfLRIterations; LR_Iteration++)
	{	
		//note that for link prices in each iteration. it (1) is first restored from temp link prices, then (2) it is then updated 
		//based on link usage status, then the current links prices are saved to temp link prices (3) then for each train it is updated again in terms of forbidden links;at the same time,save the current prices to clink:tempprices before
		//updating (4)now according to these link prices, schedule the train (5)update link prices by clink:tempprices for the current train
		//(6) schedule all trains by repeating (3),(4),(5)
		cout << "Lagrangian Iteration " << LR_Iteration+1<<"/"<<g_MaxNumberOfLRIterations << endl;
		g_CurrentLRIterationNumber=LR_Iteration+1;		
	
		 //1 update link prices (i.e. lagrangian multipliers)
		 //1.1 restore link prices from last iteration.in case link.price is changed by deducing algorithm
		if (LR_Iteration>0)
		{
			 for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			 {
				 for(int t=0; t< OptimizationHorizon; t++)
				 {
					 (*iLink)->m_ResourceAry[t].Price = templinkprice[(*iLink)->m_LinkID][t];					
				 }
			 }
		}			 

		 //1.2 update resource usage status by the trains in g_TrainVector
		 g_UpdateResourceUsageStatus(g_CurrentLRIterationNumber,g_TrainVector,true);

		//1.3 update step size using subgradient
		 float StepSize=0;
		 
		StepSize = 1.0f/(LR_Iteration+1.0f);
		 		

		if(StepSize< g_MinimumStepSize)  //1.3.1 keep the minimum step size
		{
			StepSize = g_MinimumStepSize;   
		}
		
		//1.4 update resource usage price for each time stamp			
			StartTime=0;
			EndTime=g_OptimizationHorizon;
		//1.4 update link prices
		g_UpdateResourcePrices(StartTime,EndTime,StepSize,g_CurrentLRIterationNumber,NumberOfIterationsWithMemory);
		//g_ExportResourcePriceToTXTFile();
		//save the current link prices in this iteration into temporary storage
		if (LR_Iteration>0)
		{
			for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			{
				for(int t=0; t< OptimizationHorizon; t++)
				{
					templinkprice[(*iLink)->m_LinkID][t] = (*iLink)->m_ResourceAry[t].Price;					
				}
			}
		}
				
		//2 find the subproblem solution ( by time-dependent shortest path) for each train, so we know its path and timetable
		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		float TotalRASTripCost = 0;
		double TotalWaitingTimeCost = 0;
		double TotalFreeRunningTimeCost = 0;
		int totaldelaytime=0;
		for(int v = 0; v<g_TrainVector.size(); v++)//note that the train scheduling sequence does not matter  here
		{
			CTrain* pTrain = g_TrainVector[v];			
			float TripPrice  = 0;		
			m_pNetwork->pT = pTrain;			
			//2.1 update link usage price according to forbidden links for each train. since different train may have different forbidden links
			g_UpdateLinkResourcePriceByForbiddenLinkIDs(pTrain->ForbiddenLinkIDs,0);
			
			//2.2 update the time-space network and m_LinkTDCost in the m_pNetwork according to link price
			m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain);

			//2.3 perform shortest path algorithm
			if (!g_PathBasedScheduling)
			{
				m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_EntryTime,pTrain->m_DestinationNodeID , pTrain->m_AllowableSlackAtDeparture, pTrain->m_CostPerUnitTimeStopped,pTrain);
			}
			else
			{
				m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginMPNodeID , pTrain->m_EntryTime,pTrain->m_DestinationMPNodeID , pTrain->m_AllowableSlackAtDeparture, pTrain->m_CostPerUnitTimeStopped,pTrain);
			}
			
			//2.4 fetch the train path solution
			TripPrice  = 0;		
			if (!g_PathBasedScheduling)
			{
				pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_EntryTime, pTrain->m_DestinationNodeID,pTrain,TripPrice);
			}
			else
			{
				pTrain->m_NodeSize = m_pNetwork->FindOptimalSolutionMP(pTrain->m_OriginMPNodeID , pTrain->m_EntryTime, pTrain->m_DestinationMPNodeID,pTrain,TripPrice);
				ASSERT(pTrain->m_NodeSize!=-1);
			}			
			
			TotalTripPrice += TripPrice;//2.4.1 update variables for computing lower bound
			TotalTravelTime += pTrain->m_ActualTravelTime;
			TotalWaitingTimeCost += pTrain->m_WaitingTime*pTrain->m_CostPerUnitTimeStopped;
			TotalFreeRunningTimeCost += pTrain->m_FreeRunningTime*pTrain->m_CostPerUnitTimeRunning;
			totaldelaytime+=abs(pTrain->m_RealizedCompletionTime-pTrain->m_PlannedCompletionTime);
			for (int i=1; i< pTrain->m_NodeSize; i++)//2.4.2 find the link number along the path
			{
				CLink* pLink = NULL;
				if (pTrain->m_Direction==1)
				{
					pLink=g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
				} 
				else
				{
					pLink=g_FindLinkWithNodeIDs(pTrain->m_aryTN[i].NodeID , pTrain->m_aryTN[i-1].NodeID  );
				}
				ASSERT(pLink!=NULL);
				pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID;
			}		
			
			//restore link resource prices that are updated according to forbidden linkids at the beginning of this loop
			g_UpdateLinkResourcePriceByForbiddenLinkIDs(pTrain->ForbiddenLinkIDs,1);
		}
	
		float TotalResourcePrice = 0;//2.4.3 update total resource price for computing lower bound , make sure that the below total resource price calculation is before deducing
		double lrp=0;
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{
			int t;
			for(t=0; t< OptimizationHorizon; t++)
			{
				lrp= (*iLink)->m_ResourceAry[t].Price;
				if (lrp>10000)
				{
					continue;
				}	
			    ASSERT(lrp>=0);
				TotalResourcePrice+=lrp;
			}
			
		}
		
		g_UpdateResourceUsageStatus(g_CurrentLRIterationNumber,g_TrainVector,true);//2.4.4 check whether the feasible solution has been found by LR
		bool bFeasibleFlag = true;
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{	
			for(int t=0; t< OptimizationHorizon; t++)
			{			
				if((*iLink)->m_ResourceAry[t].UsageCount >(*iLink)->m_LinkCapacity[t])
				{
					bFeasibleFlag = false;		
				}
			}
		}
		
		if (bFeasibleFlag==true)
		{
			//g_ExportTimetableDataToXMLFile(LR_Iteration+1,g_TrainVector);
			//g_ExportTimetableDataToCSVFile(LR_Iteration+1,g_TrainVector);
		}

		//3 deduce solutions obtained by LR into problem feasible solutions
		bool result = true;
		
		result = g_DeduceToProblemFeasibleSolution(2);

		if (result)
		{
			//g_ExportTimetableDataToXMLFile(1000+LR_Iteration+1,g_TrainVector2);//corresponding problem feasible solution
			//g_ExportTimetableDataToCSVFile(1000+LR_Iteration+1,g_TrainVector2);
		}		
		else
		{
			cout << "Unable to deduce at LR_Iteration:"<<LR_Iteration+1  << endl;
		}
	
		
		//4 update lowerbound and upperbound values
		double localupperbound=0;
		double locallowerbound=0;
		
		//4.1 update upper bound by deduced solutions
				
		g_UpdateUpperBoundByTrainVector(g_TrainVector2);
		

		if (bFeasibleFlag&& LR_Iteration >0)
		{				
			localupperbound = totaldelaytime;
			globalupperbound=min(globalupperbound,localupperbound);
		}
		//4.2 update upper bound if feasible solution is found by LR
		
		//4.3 update lower bound
		locallowerbound = TotalTripPrice-TotalResourcePrice;
		globallowerbound = max(globallowerbound,locallowerbound);
				 
		if (abs(globallowerbound-globalupperbound)<=0.05)
		{
			ASSERT(abs(globallowerbound-globalupperbound)<=0.05);
		}
		else
		{
			ASSERT(globallowerbound<=globalupperbound);
		}

		CTimeSpan ctime = CTime::GetCurrentTime()-g_SolutionStartTime;
		g_LogFile<<"Computational time:,"<< ctime.GetTotalSeconds() << ",Iteration:, " << LR_Iteration+1 <<",Lower bound in Minute:,"<<globallowerbound/g_MinuteDivision<<",Upper bound in Minute:,"<<globalupperbound/g_MinuteDivision<<",Lower bound:,"<<globallowerbound<<",Upper bound:,"<<globalupperbound<< ",Total Trip Price:,"<< TotalTripPrice << ",Total Resource Price (Resource Cost):," << TotalResourcePrice << ",Total Travel Time:," << TotalTravelTime<<",Optimality gap:,"<<(globalupperbound-globallowerbound)/globalupperbound  << endl;
		
			
		if (bFeasibleFlag)// feasible solution found, exit Lagrangian iteration
		{
			cout << "Feasible solution is found by LR." << endl;
			break;   
		}

	}//for each lagrangian relaxation iteration
	
	cout << "End of Lagrangian Iteration Process " << endl;
	cout << "Running Time:" << g_GetAppRunningTime()  << endl;
	

	if(m_pNetwork !=NULL)     // m_pNetwork is used to calculate time-dependent generalized least cost path 
	{
		delete m_pNetwork;
		m_pNetwork = NULL;
	}

	if (templinkprice!=NULL)
	{
		DeallocateDynamicArray<float>(templinkprice,g_LinkSet.size(),g_OptimizationHorizon);
		templinkprice = NULL;
	}

	return true;
}
bool g_FindPriorityRanking_FIFO(int* pTrainIndexInTrainVector)
{

	int number =0;
	int newindex=-1;

	CTrain* pT= NULL;	

	int rsize = g_TrainVector.size();	

	while (number<rsize)
	{
		float initvalue=999999;		
		for (int v=0;v<rsize;v++)
		{
			bool flag=false;
			pT = g_TrainVector[v];
			for (int j=0;j<number;j++)
			{
				if (pTrainIndexInTrainVector[j]==v)
				{
					flag=true;
				}
			}
			if (flag==true)
			{
				continue;
			}

			if (pT->m_EntryTime<=initvalue)
			{
				initvalue=pT->m_EntryTime;
				newindex = v;
			}
		}
		if (newindex!=-1)
		{
			pTrainIndexInTrainVector[number]=newindex;
			number++;
		}
	}
	pTrainIndexInTrainVector[number]=-1;
	return true;
}

bool g_FindPriorityRanking(int* pTrainIndexInTrainVector)
{

	int number =0;
	CTrain* pT= NULL;

	for (int v=0;v<g_TrainVector.size();v++)
	{
		pT=g_TrainVector[v];
		float actualtriptime = pT->m_ActualTravelTime;		
		if (g_CurrentLRIterationNumber==1)
		{
			pT->m_ActualTravelTimeFreeStatus = pT->m_ActualTravelTime;
		}
		float freetriptime  = pT->m_ActualTravelTimeFreeStatus;
		ASSERT(freetriptime!=0);
		pT->m_DeviationRatio = (actualtriptime-freetriptime)/freetriptime;

	}

	int rsize = g_TrainVector.size();

	while (number<rsize)
	{
		float initvalue=999999;
		int newindex=-1;
		for (int v=0;v<rsize;v++)
		{
			bool flag=false;
			pT = g_TrainVector[v];
			for (int j=0;j<number;j++)
			{
				if (pTrainIndexInTrainVector[j]==v)
				{
					flag=true;
				}
			}
			if (flag==true)
			{
				continue;
			}
			
			if (pT->m_DeviationRatio<=initvalue)
			{
				initvalue=pT->m_DeviationRatio;
				newindex = v;
			}
			
			

		}
		if (newindex!=-1)
		{
			pTrainIndexInTrainVector[number]=newindex;
			number++;
		}
	}
	pTrainIndexInTrainVector[number]=-1;
	return true;
}



bool g_Timetable_Optimization_Priority_Rule_Method(int *pTrainsToBeScheduled)
{
	
	if(pTrainsToBeScheduled==NULL)//use the order in which trains are stored in g_trainvector2
	{
		pTrainsToBeScheduled = new int[MAX_TRAIN_NUMBER];
		CTrain *pTrain = NULL;
		for (int i=0;i<MAX_TRAIN_NUMBER;i++)
		{
			pTrainsToBeScheduled[i]=MAX_TRAIN_NUMBER+10;
		}
		int number =0;
		for (int v=0;v<g_TrainVector2.size();v++)
		{
			pTrainsToBeScheduled[number]=v;		
			number++;
		}	
		pTrainsToBeScheduled[number]=-1;
	}

	bool flag = false;
	for (int i=0;i<MAX_TRAIN_NUMBER;i++)
	{
		if (pTrainsToBeScheduled[i]==-1)
		{
			flag = true;
			break;
		}
	}
	ASSERT(flag);

	int OptimizationHorizon = g_OptimizationHorizon;  // we need to dynamically determine the optimization 

	NetworkForSP* m_pNetwork =NULL;

	int linksize = 0;
	int nodesize = 0;
	if (g_PathBasedScheduling)
	{
		if (g_MaxLinkSize==0)
		{
			g_UpdateMaximumNodeandLinkSize(g_TrainVector);
		}		
		nodesize = g_MaxNodeSize;
		linksize = g_MaxLinkSize;
	}
	else
	{
		nodesize = g_NodeSet.size();
		linksize = g_LinkSet.size();
	}

	m_pNetwork = new NetworkForSP(nodesize, linksize, OptimizationHorizon, g_OptimizationInterval);  

	float TotalTripPrice  = 0;
	int TotalTravelTime =0;
	float totalrascost = 0;


	std::set<CLink*>::iterator iLink;
	std::list<CLink*>::iterator iLink2;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		// reset resource usage counter for each timestamp
		for(int t=0; t< OptimizationHorizon; t++)
		{
			(*iLink)->m_ResourceAry[t].UsageCount = 0;
			(*iLink)->m_ResourceAry[t].Price = 0;			
			if ((*iLink)->m_LinkCapacity[t]==0)//here it("<1" rather than the earlier one ==0) is only applicable for two scenarios and the aggregated capacity is less than 1
			{
				(*iLink)->m_ResourceAry[t].Price=MAX_SPLABEL;
			}			
		}
	}

	// step 1. reset resource usage counter for each timestamp
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		for(int t=0; t< OptimizationHorizon; t++)
		{
			(*iLink)->m_ResourceAry[t].UsageCount =0;
			for (int i=0;i<MAX_TRAIN_NUMBER;i++)
			{
				(*iLink)->m_ResourceAry[t].TrainUseFlag[i]=0;
			}
		}
	}
    		
		
		unsigned int v;
		bool result = false;
		cout << "Deduce to problem feasible solution by Priority Rule-based method"<< endl;
		for(v = 0; v<MAX_TRAIN_NUMBER; v++)
		{
			if (pTrainsToBeScheduled[v]==-1)
			{
				break;
			}
			ASSERT(pTrainsToBeScheduled[v]<g_TrainVector2.size());
			CTrain* pTrain = g_TrainVector2[pTrainsToBeScheduled[v]];

			m_pNetwork->pT = pTrain;	
			g_UpdateLinkResourcePriceByForbiddenLinkIDs(pTrain->ForbiddenLinkIDs,0);
			cout << "Scheduling Train " << pTrain->m_TrainID << endl;	
			//step 2. train by train, schedule trains according to global prority, given in the form of train sequence
			//step 2.1. build time-dependent network with resource price
			m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain);
			//step 2.2 perform shortest path algorithm
			if (!g_PathBasedScheduling)
			{
				result = m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_EntryTime,pTrain->m_DestinationNodeID , pTrain->m_AllowableSlackAtDeparture, pTrain->m_CostPerUnitTimeStopped,pTrain);
			}
			else
			{
				result = m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginMPNodeID , pTrain->m_EntryTime,pTrain->m_DestinationMPNodeID , pTrain->m_AllowableSlackAtDeparture, pTrain->m_CostPerUnitTimeStopped,pTrain);
			}
	
			if (!result)
			{
				break;
			}			
			//step 2.3 fetch the train path solution
			float TripPrice = 0;
			if (!g_PathBasedScheduling)
			{
				pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_EntryTime, pTrain->m_DestinationNodeID,pTrain,TripPrice);
			}
			else
			{
				pTrain->m_NodeSize = m_pNetwork->FindOptimalSolutionMP(pTrain->m_OriginMPNodeID , pTrain->m_EntryTime, pTrain->m_DestinationMPNodeID,pTrain,TripPrice);
			}
			if (pTrain->m_NodeSize==-1)
			{
				result = false;
				break;
			}
			TotalTripPrice+=TripPrice;		
			TotalTravelTime+=pTrain->m_ActualTravelTime;


			//find the link no along the path

			for (int i=1; i< pTrain->m_NodeSize ; i++)
			{
				CLink* pLink = NULL;
				if (pTrain->m_Direction==1)
				{
					pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
				}
				else
				{
					pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i].NodeID  , pTrain->m_aryTN[i-1].NodeID );
				}
				
				ASSERT(pLink!=NULL);
				pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

			}

			// step 3 record resource usage
			// start from n=1, as only elements from n=1 to m_NodeSize hold link information, the first node element has no link info
			for(int n = 1; n< pTrain->m_NodeSize; n++)
			{
				CLink* pLink = g_LinkIDMap[pTrain->m_aryTN[n].LinkID];					

					// inside loop for each link traveled by each train					
					for(int t = pTrain->m_aryTN[n-1].NodeArrivalTimestamp; t< pTrain->m_aryTN[n].NodeArrivalTimestamp+g_SafetyHeadway; t++)
					{
						if(t>=0 && t<OptimizationHorizon)
						{
							if (pLink->m_ResourceAry[t].TrainUseFlag[v]==0)//&&pLink->m_ResourceAry[t].UsageCount<pLink->m_LinkCapacity[t][g_ScenarioID]
							{//only when the usage count is smaller than the link capacity, we increase the usage count by 1. otherwise,it might be confusing
								pLink->m_ResourceAry[t].UsageCount+=1;								
								pLink->m_ResourceAry[t].TrainUseFlag[v]=1;
							}					
							for (iLink2 = pLink->m_LinkGroupList.begin(); iLink2 != pLink->m_LinkGroupList.end(); iLink2++)//deal with link group
							{
								if ((*iLink2)->m_ResourceAry[t].TrainUseFlag[v]==0)//&&(*iLink2)->m_ResourceAry[t].UsageCount<(*iLink2)->m_LinkCapacity[t][g_ScenarioID]
								{
									(*iLink2)->m_ResourceAry[t].UsageCount+=1;								
									(*iLink2)->m_ResourceAry[t].TrainUseFlag[v]=1;
								}
							}	

							if(pLink->m_ResourceAry[t].UsageCount >= pLink->m_LinkCapacity[t])  //over capacity
							{								
								pLink->m_ResourceAry[t].Price  = MAX_SPLABEL;  // set the maximum price so the followers cannot use this time																
								for (iLink2 = pLink->m_LinkGroupList.begin(); iLink2 != pLink->m_LinkGroupList.end(); iLink2++)
								{
									(*iLink2)->m_ResourceAry[t].Price  = MAX_SPLABEL;
								}
							}

						}

					}
				

			}
			g_UpdateLinkResourcePriceByForbiddenLinkIDs(pTrain->ForbiddenLinkIDs,1);

		}
		
		//final export to log file after scheduling all trains
		if (result)
		{		
			//make sure capacity constraints is respected
			for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			{
				int t;
				for(t=0; t< OptimizationHorizon; t++)
				{
					CLink *pl=(*iLink);
					if(pl->m_ResourceAry[t].UsageCount > pl->m_LinkCapacity[t])
					{
						ASSERT(false);//in case "result" is true, this above "if" condition can not be satisfied						
					}
				}
			}			

			cout << "Running Time:" << g_GetAppRunningTime()  << endl;
		}
		
		

	if(m_pNetwork != NULL)// m_pNetwork is used to calculate time-dependent generalized least cost path 
	{
		delete m_pNetwork;
		m_pNetwork = NULL;
	}

	if (pTrainsToBeScheduled!=NULL)
	{
		delete []pTrainsToBeScheduled;
		pTrainsToBeScheduled=NULL;
	}
	
	return result;

}


bool g_DeduceToProblemFeasibleSolution(int deducingmethod)
{
	int iternum=1;
	int linkid,starttime,endtime;
	int* pTrainIndexInTrainVector=NULL;
	CTrain* pTrain;
	bool interre = false;
	pTrainIndexInTrainVector = new int[MAX_TRAIN_NUMBER];
	int method=2;//schedule trains one train by one train, according to global priority
	for (int i=0;i<MAX_TRAIN_NUMBER;i++)
	{
		pTrainIndexInTrainVector[i]=MAX_TRAIN_NUMBER+10;
	}

	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		// reset resource usage counter for each timestamp		
		for(int t=0; t< g_OptimizationHorizon; t++)
		{
			if ((*iLink)->m_LinkCapacity[t]==0)
			{
				(*iLink)->m_ResourceAry[t].Price=MAX_SPLABEL;
			}			
		}
	}
	bool result=false;

	if (deducingmethod==2)//train by train
	{			
		//interre = g_FindPriorityRanking_FIFO(pTrainIndexInTrainVector);	
		interre = g_FindPriorityRanking(pTrainIndexInTrainVector);	
		ASSERT(interre==true);
		result=g_Timetable_Optimization_Priority_Rule_Method(pTrainIndexInTrainVector);		
		pTrainIndexInTrainVector=NULL;

	}
	
	if (pTrainIndexInTrainVector!=NULL)
	{		
		delete []pTrainIndexInTrainVector;
		pTrainIndexInTrainVector=NULL;
	}
	
	return result;
}

bool copytrainvector(std::vector<CTrain*> oriTrainVector,std::vector<CTrain*> destiTrainVector)
{
	std::vector<CTrain*>::iterator iterTrain;
	std::vector<CTrain*>::iterator iterTrain2;

	for (iterTrain = oriTrainVector.begin(); iterTrain != oriTrainVector.end(); iterTrain++)
	{
		for (iterTrain2 = destiTrainVector.begin(); iterTrain2 != destiTrainVector.end(); iterTrain2++)
		{
			if ((*iterTrain2)->m_TrainID==(*iterTrain)->m_TrainID)
			{
				break;
			}
		}
		if ((*iterTrain2)->m_aryTN!=NULL)
		{
			delete [](*iterTrain2)->m_aryTN;
			(*iterTrain2)->m_aryTN = NULL;
			(*iterTrain2)->m_aryTN = new STrainNode[(*iterTrain)->m_NodeSize];
		}
		else
		{
			(*iterTrain2)->m_aryTN = new STrainNode[(*iterTrain)->m_NodeSize];
		}
		(*iterTrain2)->m_NodeSize = (*iterTrain)->m_NodeSize;
		for (int i=0;i<(*iterTrain)->m_NodeSize;i++)
		{
			(*iterTrain2)->m_aryTN[i].LinkID = (*iterTrain)->m_aryTN[i].LinkID;
			(*iterTrain2)->m_aryTN[i].NodeArrivalTimestamp = (*iterTrain)->m_aryTN[i].NodeArrivalTimestamp;
			(*iterTrain2)->m_aryTN[i].NodeDepartureTimestamp = (*iterTrain)->m_aryTN[i].NodeDepartureTimestamp;
			(*iterTrain2)->m_aryTN[i].NodeID = (*iterTrain)->m_aryTN[i].NodeID;
			(*iterTrain2)->m_aryTN[i].TaskProcessingTime = (*iterTrain)->m_aryTN[i].TaskProcessingTime;
			(*iterTrain2)->m_aryTN[i].TaskScheduleWaitingTime = (*iterTrain)->m_aryTN[i].TaskScheduleWaitingTime;
		}
	}
	return true;
}




bool g_UpdateResourceUsageStatus(int LR_Iteration,std::vector<CTrain*> TrainVector,bool updatelastiteratenumber)
{
	std::set<CLink*>::iterator iLink;
	std::list<CLink*>::iterator iLink2;
	int OptimizationHorizon = g_OptimizationHorizon;
	//step 1. reset resource usage counter for each time stamp
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		for(int t=0; t< OptimizationHorizon; t++)
		{
			(*iLink)->m_ResourceAry[t].UsageCount =0;		
			for (int i=0;i<MAX_TRAIN_NUMBER;i++)
			{
				(*iLink)->m_ResourceAry[t].TrainUseFlag[i]=0;
			}
		}
	}

	//step 2. for each train, record their resource usage on the corresponding link
	unsigned int v;
	for(v = 0; v<TrainVector.size(); v++)
	{
		CTrain* pTrain = TrainVector[v];

		//start from n=1, as only elements from n=1 to m_NodeSize hold link information, the first node element has no link info
		for(int n = 1; n< pTrain->m_NodeSize; n++)
		{
			CLink* pLink = g_LinkIDMap[pTrain->m_aryTN[n].LinkID];

			
				// inside loop for each link traveled by each train
				for(int t = pTrain->m_aryTN[n-1].NodeArrivalTimestamp; t<pTrain->m_aryTN[n].NodeArrivalTimestamp+g_SafetyHeadway; t++)
				{
					if(t>=0 && t<OptimizationHorizon)
					{						
						if (pLink->m_ResourceAry[t].TrainUseFlag[v]==0)
						{
							pLink->m_ResourceAry[t].UsageCount+=1;
							if (updatelastiteratenumber)
							{
								pLink->m_ResourceAry[t].LastUseIterationNo = g_CurrentLRIterationNumber;							
							}							
							pLink->m_ResourceAry[t].TrainUseFlag[v]=1;
						
							
						}
						//deal with link group				
						for (iLink2 = pLink->m_LinkGroupList.begin(); iLink2 != pLink->m_LinkGroupList.end(); iLink2++)
						{
							if ((*iLink2)->m_ResourceAry[t].TrainUseFlag[v]==0)
							{
								(*iLink2)->m_ResourceAry[t].UsageCount+=1;								
								(*iLink2)->m_ResourceAry[t].TrainUseFlag[v]=1;
							}
						}
					}

				}
			

		}
	}
	return true;
};




bool g_UpdateUpperBoundByTrainVector(std::vector<CTrain*> TrainVector)
{
	

	float TotalDelayTime=0;

	for (int de = 0;de<TrainVector.size();de++)// if conflict by conflict is applied, here we need to compare it to g_Trainvector3
	{
		CTrain* ptr = TrainVector[de];					
		TotalDelayTime+=abs(ptr->m_RealizedCompletionTime-ptr->m_PlannedCompletionTime);
	}

	double localupperbound = TotalDelayTime;
	if (localupperbound<globalupperbound)
	{
		globalupperbound = localupperbound;						
		g_ExportTimetableDataToXMLFile(g_CurrentLRIterationNumber,TrainVector);
	}

	return true;
}


