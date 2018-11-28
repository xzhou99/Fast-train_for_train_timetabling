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
//
#include "stdafx.h"
#include "Network.h"
#include "FastTrain.h"

#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

using namespace std;


CWinApp theApp;// The one and only application object
ofstream g_LogFile;//for Lagrangian relaxation log file
CTime g_AppStartTime;//for calculating computation time
CTime g_SolutionStartTime;

int g_MaxNumberOfLRIterations=0;//maximum number of Lagrangian relaxation iterations
int g_CurrentLRIterationNumber=-1;
int g_MaxLinkSize = 0;
int g_MaxNodeSize = 0;
int g_SafetyHeadway=0;
int g_PathBasedScheduling=0;//0:simultaneous 1:sequential
int g_PathsPerTrain=1;//number of paths for each train
int g_TrainPathID = 0;//for train path counter, note that the term "TrainPath" does not mean space-time train path, but physical path on the network
float g_MinimumStepSize = 0.1;//for Lagrangian relaxation use
int g_OptimizationHorizon=1440;
int g_MinuteDivision = 1;//for different time resolution, e.g., in 1 minute, 30 seconds, 15 seconds
int g_NumberOfIterationsWithMemory = 5;
int g_OptimizationInterval = 1;
int g_LastestDepartureTime = 0;
int g_TotalFreeRunningTime = 0;
int g_MaxTrainWaitingTime = 5;
int g_MaxSlackTimeAtDeparture = 10;
int g_GlobalTrainPathID = 0;
double globalupperbound=999999;

std::set<CNode*> g_NodeSet;//storing node objects
std::map<int, CNode*> g_NodeIDMap;//Node ID mapping to Node object
std::map<int, int> g_NodeIDToNumberMap;//Node ID mapping to Node Number
std::map<int, int> g_NodeNumbertoIDMap;//Node Number mapping to Node ID
std::map<int, int>g_NodeMPIDToNodeIDMap;//Node ID after super network construction mapping to Node ID in original network

std::set<CLink*> g_LinkSet;//storing link objects
std::map<int, CLink*> g_LinkIDMap;//Link ID mapping to Link object
std::map<int, int>g_LinkMPIDToLinkIDMap;//Link ID after super network construction mapping to Link ID in original network

std::map<unsigned long, CLink*> g_NodeIDtoLinkMap;//From Node ID+ToNodeID mapping to Link object

std::set<TrainPaths*> g_TrainPathsSet;//storing train paths objects
std::set<TrainPath*> g_TrainPathSet;//storing train path objects
std::map<int, TrainPaths*> g_TrainPathsIDMap;//TrainPaths ID mapping to TrainPaths object
std::map<int, TrainPath*> g_TrainPathIDMap;//TrainPath ID mapping to TrainPath object
std::map<unsigned long, TrainPaths*> g_NodeIDtoTrainPathsMap;//Origin Node ID + Destination ID mapping to TrainPaths object

std::vector<CTrain*> g_TrainVector;//for Lagrangian relaxation use
std::vector<CTrain*> g_TrainVector2;//for priority rule-based use: train by train
std::map<int, CTrain*> g_TrainIDMap;//Train ID mapping to Train object: for Lagrangian relaxation use
std::map<int, CTrain*> g_TrainIDMap2;//Train ID mapping to Train object: for priority rule-based use: train by train


CString g_GetAppRunningTime()
{

	CString str;

	CTime EndTime = CTime::GetCurrentTime();
	CTimeSpan ts = EndTime  - g_AppStartTime;

	str = ts.Format( "App Clock: %H:%M:%S --" );
	return str;
}

void g_ComputeAndOutputTrainLinkRunningTime()//for glpk data preparation
{
	std::vector<CTrain*>::iterator iterTrain;
	std::set<CLink*>::iterator iterlink;

	FILE* st;
	fopen_s(&st,"output_train_link_running_time.csv","w");	

	if(st!=NULL)
	{
		for (iterTrain = g_TrainVector.begin(); iterTrain != g_TrainVector.end(); iterTrain++)
		{

			for (iterlink=g_LinkSet.begin();iterlink!=g_LinkSet.end();iterlink++)
			{
				int runningtime = (*iterlink)->GetTrainRunningTimeInMinuteDivision((*iterTrain)->m_SpeedMultiplier,(*iterTrain)->m_Direction);
				if ((*iterTrain)->m_Direction==1)
				{
					fprintf(st,"%d,%d,%d,%d\n",(*iterTrain)->m_TrainID+1,(*iterlink)->m_FromNodeNumber+1,(*iterlink)->m_ToNodeNumber+1,runningtime);
				}
				else
				{
					fprintf(st,"%d,%d,%d,%d\n",(*iterTrain)->m_TrainID+1,(*iterlink)->m_ToNodeNumber+1,(*iterlink)->m_FromNodeNumber+1,runningtime);
				}

			}
		}
	}
	fclose(st);
}




void g_OutputPossibleTrainLinks()//for glpk data preparation
{
	std::vector<CTrain*>::iterator iterTrain;
	std::set<CLink*>::iterator iterlink;

	FILE* st;
	fopen_s(&st,"output_train_possible_links.csv","w");

	if(st!=NULL)
	{
		for (iterTrain = g_TrainVector.begin(); iterTrain != g_TrainVector.end(); iterTrain++)
		{

			for (iterlink=g_LinkSet.begin();iterlink!=g_LinkSet.end();iterlink++)
			{
				bool forbiddenflag = false;
				for (int i=0;i<g_LinkSet.size();i++)
				{
					if ((*iterTrain)->ForbiddenLinkIDs[i]==(*iterlink)->m_LinkID)
					{
						forbiddenflag=true;
					}
				}
				if (forbiddenflag)
				{
					continue;
				}
				if ((*iterTrain)->m_Direction==1)
				{
					fprintf(st,"%d,%d,%d\n",(*iterTrain)->m_TrainID+1,(*iterlink)->m_FromNodeNumber+1,(*iterlink)->m_ToNodeNumber+1);
				}
				else
				{
					fprintf(st,"%d,%d,%d\n",(*iterTrain)->m_TrainID+1,(*iterlink)->m_ToNodeNumber+1,(*iterlink)->m_FromNodeNumber+1);
				}

			}
		}
	}
	fclose(st);
}


void g_ReadInputFiles()//please do not change the sequence of loading input data
{
	g_ReadNodeCSVFile();

	g_ReadLinkCSVFile();

	g_ReadMOWCSVFile();

	g_ReadTrainInfoCSVFile();
	
	if (g_PathBasedScheduling)
	{
		g_GenerateTrainPhysicalPaths();
	}	
	else
	{
		g_GenerateTrainPhysicalPaths();
		g_GenerateTrainForbiddenLinkIDs();
	}
	
	//g_ComputeAndOutputTrainLinkRunningTime();
	//g_OutputPossibleTrainLinks();

	g_LogFile << "Number of Nodes = "<< g_NodeSet.size() << endl;
	g_LogFile << "Number of Links = "<< g_LinkSet.size() << endl;
	g_LogFile << "Number of Trains = "<< g_TrainVector.size() << endl;
	

}

bool g_ReadNodeCSVFile()
{
	FILE* st = NULL;
	cout << "Reading file input_node.csv..."<< endl;

	fopen_s(&st,"input_node.csv","r");
	if(st!=NULL)
	{
		int i=0;
		CNode* pNode = 0;
		while(!feof(st))
		{
			int node_name = g_read_integer(st);

			if(node_name == -1)  // reach end of file
				break;

			if(node_name>= MAX_PHYSICAL_NODE_NUMBER)
			{
				cout << "Error: Node number..."<< endl;
				g_ProgramStop();
			}

			
			// Create and insert the node
			pNode = new CNode;
			pNode->m_NodeID = i;
			
  		    g_NodeSet.insert(pNode);
			g_NodeIDMap[i] = pNode;
			g_NodeIDToNumberMap[i] = node_name;
			g_NodeNumbertoIDMap[node_name]= i;
			i++;
		}
		fclose(st);
	}else
	{
		cout << "Error: File input_node.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	g_MaxNodeSize = g_NodeSet.size();

	return true;
}

bool g_ReadMOWCSVFile()
{
	FILE* st = NULL;
	cout << "Reading file input_MOW.csv..."<< endl;

	fopen_s(&st,"input_MOW.csv","r");

	int i = 0;
	if(st!=NULL)
	{
		CapacityReduction cr;
		CLink*pLink=NULL;
		while(!feof(st))
		{
			int FromNodeName =  g_read_integer(st);
			if(FromNodeName == -1)  // reach end of file
				break;
			int ToNodeName = g_read_integer(st);
			pLink = g_FindLinkWithNodeNumbers(FromNodeName,ToNodeName);
			if (pLink!=NULL)
			{
				cr.from_node_id=g_NodeNumbertoIDMap[FromNodeName];
				cr.to_node_id=g_NodeNumbertoIDMap[ToNodeName];
				cr.start_time_in_minutes=g_read_integer(st);
				cr.end_time_in_minutes=g_read_integer(st);
				cr.speed_limit_inMilePerHour=g_read_integer(st);
				cr.linkcapacity = g_read_float(st);
				cr.occurenceprob = g_read_float(st);
				pLink->CapacityReductionVector.push_back(cr);
				for (int i=cr.start_time_in_minutes*g_MinuteDivision-1;i<=cr.end_time_in_minutes*g_MinuteDivision;i++)
				{
					if (i==-1)
					{
						i=0;
						continue;
					}
					pLink->m_LinkCapacity[i] = (int)cr.linkcapacity;
				}
			}		

		}
		fclose(st);
		return true;
	}else
	{
		cout << "Error: File input_MOW.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}
	return true;
}


bool g_ReadLinkCSVFile()
{

	FILE* st = NULL;
	cout << "Reading file input_link.csv..."<< endl;


	fopen_s(&st,"input_link.csv","r");

	int i = 0;
	if(st!=NULL)
	{
		CLink* pLink = 0;
		while(!feof(st))
		{
			int FromNodeName =  g_read_integer(st);
			if(FromNodeName == -1)  // reach end of file
				break;
			int ToNodeName = g_read_integer(st);

			pLink = new CLink(g_OptimizationHorizon);
			pLink->m_LinkID = i;
			pLink->m_FromNodeNumber = FromNodeName;
			pLink->m_ToNodeNumber = ToNodeName;
			pLink->m_FromNodeID = g_NodeNumbertoIDMap[FromNodeName ];
			pLink->m_ToNodeID= g_NodeNumbertoIDMap[ToNodeName];
			pLink->m_Length= g_read_float(st);//the length data given in the RAS test set is too small. so here we multiply it by 10 (not now!)			
			pLink->m_FTSpeedLimit= g_read_integer(st);
			pLink->m_TFSpeedLimit= g_read_integer(st);
			float linkcapacity = g_read_float(st);
			for (int i=0;i<g_OptimizationHorizon;i++)
			{
				
					pLink->m_LinkCapacity[i] = linkcapacity;
				
			}
			pLink->m_LinkType= g_read_integer(st);
			pLink->m_BidirectionalFlag=g_read_integer(st);
			g_LinkSet.insert(pLink);
			g_LinkIDMap[i]  = pLink;
			unsigned long LinkKey = g_GetLinkKey( pLink->m_FromNodeID, pLink->m_ToNodeID);
			g_NodeIDtoLinkMap[LinkKey] = pLink;

			i++;

		}
		fclose(st);
	}else
	{
		cout << "Error: File input_link.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}

	g_MaxLinkSize = g_LinkSet.size();

	cout << "Number of Physical Nodes = "<< g_NodeSet.size() << endl;
	cout << "Number of Physical Links = "<< g_LinkSet.size() << endl;


	return true;
}


bool g_ReadTrainInfoCSVFile()
{
	FILE* st = NULL;
	cout << "Reading file input_train_info.csv..."<< endl;
	fopen_s(&st,"input_train_info.csv","r");
	
	if(st!=NULL)
	{
		int train_no = 0;
		while(!feof(st))
		{
			int train_id =  g_read_integer(st);

			if(train_id == -1)
				break;
			CTrain* pTrain = new CTrain();
			pTrain->m_TrainID = train_id;
			pTrain->TraversedLinkIDs= new int[g_LinkSet.size()];
			pTrain->ForbiddenLinkIDs= new int[g_LinkSet.size()];
			
			for (int i=0;i<g_LinkSet.size();i++)
			{
				pTrain->TraversedLinkIDs[i]=-1;	
				pTrain->ForbiddenLinkIDs[i]=-1;	
			}
	
			pTrain->m_OriginNodeNumber =  g_read_integer(st);
			pTrain->m_DestinationNodeNumber =  g_read_integer(st);
			pTrain->m_OriginNodeID =  g_NodeNumbertoIDMap[pTrain->m_OriginNodeNumber];
			pTrain->m_DestinationNodeID =  g_NodeNumbertoIDMap[pTrain->m_DestinationNodeNumber ];			
			pTrain->m_Direction =  g_read_integer(st);
			pTrain->m_TOB = g_read_float(st);
			pTrain->m_Length = g_read_float(st);
			pTrain->m_HazMat = g_read_integer(st);
			pTrain->m_SpeedMultiplier=g_read_float(st);
			pTrain->m_EntryTime=g_read_integer(st);
			pTrain->m_EntryTime=pTrain->m_EntryTime*g_MinuteDivision;//change to minute division
			pTrain->m_CostPerUnitTimeStopped =  g_read_float(st);
			pTrain->m_AllowableSlackAtDeparture = g_MaxSlackTimeAtDeparture;
			pTrain->m_AllowableSlackAtDeparture = pTrain->m_AllowableSlackAtDeparture*g_MinuteDivision;
			pTrain->m_EarlyDepartureTime=  g_read_integer(st);	
			pTrain->m_EarlyDepartureTime = pTrain->m_EarlyDepartureTime*g_MinuteDivision;
			pTrain->m_CostPerUnitTimeRunning = g_read_float(st);
			pTrain->m_PlannedCompletionTime = g_read_integer(st);
			pTrain->m_NodeSize = 0;
			pTrain->m_ActualTravelTime = 0;

			g_TrainVector.push_back(pTrain);
			g_TrainIDMap[pTrain->m_TrainID]=pTrain;

			CTrain* pTrain2 = new CTrain();
			pTrain2->m_TrainID = pTrain->m_TrainID;
			pTrain2->TraversedLinkIDs= new int[g_LinkSet.size()];
			pTrain2->ForbiddenLinkIDs = new int[g_LinkSet.size()];
		
			for (int i=0;i<g_LinkSet.size();i++)
			{
				pTrain2->TraversedLinkIDs[i]=-1;	
				pTrain2->ForbiddenLinkIDs[i]=-1;	
			}

			pTrain2->m_OriginNodeNumber =  pTrain->m_OriginNodeNumber;
			pTrain2->m_DestinationNodeNumber =  pTrain->m_DestinationNodeNumber;
			pTrain2->m_OriginNodeID =  pTrain->m_OriginNodeID ;
			pTrain2->m_DestinationNodeID =  pTrain->m_DestinationNodeID;			
			pTrain2->m_Direction =  pTrain->m_Direction ;
			pTrain2->m_TOB = pTrain->m_TOB;
			pTrain2->m_Length = pTrain->m_Length ;
			pTrain2->m_HazMat = pTrain->m_HazMat;
			pTrain2->m_SpeedMultiplier=pTrain->m_SpeedMultiplier;
			pTrain2->m_EntryTime=pTrain->m_EntryTime;
			pTrain2->m_CostPerUnitTimeStopped =  pTrain->m_CostPerUnitTimeStopped;
			pTrain2->m_CostPerUnitTimeRunning =  pTrain->m_CostPerUnitTimeRunning;			
			pTrain2->m_AllowableSlackAtDeparture=  pTrain->m_AllowableSlackAtDeparture;
			pTrain2->m_EarlyDepartureTime=  pTrain->m_EarlyDepartureTime;
			pTrain2->m_PlannedCompletionTime=pTrain->m_PlannedCompletionTime;
			pTrain2->m_NodeSize = pTrain->m_NodeSize;
			pTrain2->m_ActualTravelTime = pTrain->m_ActualTravelTime;

			g_TrainVector2.push_back(pTrain2);
			g_TrainIDMap2[pTrain2->m_TrainID]=pTrain2;			
			train_no++;
		}

		fclose(st);

	}else
	{
		cout << "Error: File input_train_info.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
	
	cout << "Number of Trains = "<< g_TrainVector.size() << endl;
		
	return true;
}




bool g_GenerateTrainPhysicalPaths()
{
	std::vector<CTrain*>::iterator iterTrain;
	TrainPaths*pTPs = NULL;
	int trainpathsid =1;
	int originnodenumber=-1;
	int destinodenumber = -1;
	for (iterTrain = g_TrainVector.begin(); iterTrain != g_TrainVector.end(); iterTrain++)
	{
		originnodenumber = (*iterTrain)->m_OriginNodeNumber;
		destinodenumber = (*iterTrain)->m_DestinationNodeNumber;

		pTPs = g_FindTrainPathsWithNodeNumbers(originnodenumber,destinodenumber);
		if (pTPs==NULL)
		{
			pTPs = new TrainPaths();
			g_TrainPathsSet.insert(pTPs);
			pTPs->m_PathsID = trainpathsid;
			pTPs->m_OriginNodeNumber = originnodenumber;
			pTPs->m_OriginNodeID =  g_NodeNumbertoIDMap[originnodenumber];
			pTPs->m_DestinationNodeNumber = destinodenumber;
			pTPs->m_DestinationNodeID = g_NodeNumbertoIDMap[destinodenumber];			
			trainpathsid+=1;
			g_TrainPathsIDMap[pTPs->m_PathsID] = pTPs;
			unsigned long tpskey = g_GetTrainPathsKey(g_NodeNumbertoIDMap[originnodenumber],g_NodeNumbertoIDMap[destinodenumber]);
			g_NodeIDtoTrainPathsMap[tpskey]=pTPs;	
			if (g_GeneratePhysicalPaths(originnodenumber,destinodenumber,pTPs,1)==false)
			{
				//no path in eastern bound direction. then try the other direction
				if (g_GeneratePhysicalPaths(originnodenumber,destinodenumber,pTPs,2)==false)
				{
					cout << "Error: No path found for some OD pair."<< endl;
					g_ProgramStop();
				}
			} 			
		} 		
		g_FilterPossiblePaths((*iterTrain));
	}	

	return true;
}

bool g_GenerateTrainForbiddenLinkIDs()
{
	std::vector<CTrain*>::iterator iterTrain;
	CTrain*pT = NULL;
	int number = 0;
	for (iterTrain = g_TrainVector.begin(); iterTrain != g_TrainVector.end(); iterTrain++)
	{
		pT = *iterTrain;
		number = 0;
		std::set<CLink*>::iterator iterLink;

		for (iterLink = g_LinkSet.begin(); iterLink != g_LinkSet.end(); iterLink++)
		{		
			if ((*iterLink)->m_Length<pT->m_Length&&(*iterLink)->m_LinkType==4/*siding*/)//for long trains
			{	
				pT->ForbiddenLinkIDs[number]=(*iterLink)->m_LinkID;
				number++;
				continue;
				
			}
			if (pT->m_HazMat==true&&(*iterLink)->m_LinkType==4)//for hazardous train
			{
				pT->ForbiddenLinkIDs[number]=(*iterLink)->m_LinkID;
				number++;	
			}

		}		
		pT->ForbiddenLinkIDs[number]=-1;
	}


	return true;
}


bool g_GeneratePhysicalPaths(int fromnodenumber,int tonodenumber,TrainPaths*pTPs, int direction/*1-east bound;2-west bound*/)
{
	long TravelTimeBound= 10000000;
	int m_TreeListTail;
	int OriginNodeID = g_NodeNumbertoIDMap[fromnodenumber];
	int DestinationNodeID = g_NodeNumbertoIDMap[tonodenumber];
	NetworkForSP* pNSP = new NetworkForSP(g_NodeSet.size(),g_LinkSet.size(),g_OptimizationHorizon,g_OptimizationInterval);	
	pNSP->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet,&g_LinkSet,NULL);
	SearchTreeElement* pSTE = NULL;	

	pSTE = pNSP->GenerateSearchTree(OriginNodeID,DestinationNodeID,g_NodeSet.size()*5,TravelTimeBound, m_TreeListTail,direction);
	if (pSTE==NULL)
	{	
		if (pNSP!=NULL)
		{
			delete pNSP;
			pNSP = NULL;
		}
		return false;
	}
	int destireached = 0;
	for(int i = 0; i < m_TreeListTail; i++)
	{
		if(pSTE[i].CurrentNode == DestinationNodeID)
		{
			destireached = 1;
		}
	}	
	if (destireached==0)
	{
		return false;
	}	
	int * NodeList = new int[m_TreeListTail];
	FILE* st = NULL;

	for(int i = 0; i < m_TreeListTail; i++)
	{
		if(pSTE[i].CurrentNode == DestinationNodeID)
		{
			int nodeindex = 0;
			NodeList[nodeindex++] = pSTE[i].CurrentNode;
			int Pred = pSTE[i].PredecessorNode;

			while(Pred!=0)
			{
				NodeList[nodeindex++] = pSTE[Pred].CurrentNode;
				Pred = pSTE[Pred].PredecessorNode;
			}
			NodeList[nodeindex++] = pSTE[Pred].CurrentNode;			
			TrainPath*pTP = new TrainPath();
			pTP->m_PathID=g_GlobalTrainPathID;
			ASSERT(g_GlobalTrainPathID<MAX_PATH_NUMBER_PER_TRAIN);
			g_GlobalTrainPathID++;
			g_TrainPathIDMap[pTP->m_PathID]=pTP;
			pTPs->m_TrainPathList.push_back(pTP);
			pTP->m_OriginNodeNumber = fromnodenumber;
			pTP->m_OriginNodeID = g_NodeNumbertoIDMap[fromnodenumber];
			pTP->m_DestinationNodeNumber = tonodenumber;
			pTP->m_DestinationNodeID = g_NodeNumbertoIDMap[tonodenumber];
			CLink*pLink = NULL;
			for(int n = nodeindex-1; n>=1; n--)
			{
				if (direction==1)
				{
					pLink = g_FindLinkWithNodeIDs(NodeList[n],NodeList[n-1]);
					ASSERT(pLink!=NULL);
					pTP->m_LinkList.push_back(pLink);
					pTP->m_totallength +=pLink->m_Length;
					pTP->m_totalminimumtraveltime+=pLink->GetTrainRunningTimeInMinuteDivision(1,direction);
				} 
				if (direction==2)
				{
					pLink = g_FindLinkWithNodeIDs(NodeList[n-1],NodeList[n]);
					ASSERT(pLink!=NULL);
					pTP->m_LinkList.push_back(pLink);
					pTP->m_totallength +=pLink->m_Length;					
				}	

			}
		}

	}

	if(pSTE!=NULL)
	{
		delete []pSTE;		
		pSTE = NULL;
	}
		
	if (pNSP!=NULL)
	{
		delete pNSP;
		pNSP = NULL;
	}
	
	if (NodeList!=NULL)
	{
		delete []NodeList;
		NodeList=NULL;
	}
	
	return true;
}

bool MyComparatorTotalTravelTime(const TrainPath* a,const TrainPath* b) 
{ 
   return  (a->m_totalminimumtraveltime) <(b->m_totalminimumtraveltime) ; 
} 

bool g_FilterPossiblePaths(CTrain* pT)
{
	int originnodenumber = pT->m_OriginNodeNumber;
	int destinodenumber = pT->m_DestinationNodeNumber;

	CTrain* pT2 = g_TrainIDMap2[pT->m_TrainID];
	
	TrainPaths* pTPs = g_FindTrainPathsWithNodeNumbers(originnodenumber,destinodenumber);
	ASSERT(pTPs!=NULL);
	ASSERT(pTPs->m_TrainPathList.size()>0);
	
	if (pTPs->m_TrainPathList.size()<pT->MaxNumberofTrainPaths)
	{
		pT->MaxNumberofTrainPaths = pTPs->m_TrainPathList.size();
	}
	
	std::list<TrainPath*>::iterator iterTP;
	std::vector<TrainPath*>::iterator iterTPV;
	std::list<CLink*>::iterator iterlink;

	for (iterTP = pTPs->m_TrainPathList.begin(); iterTP != pTPs->m_TrainPathList.end(); iterTP++)
	{
		bool feasibleflag = true;
		for (iterlink = (*iterTP)->m_LinkList.begin();iterlink != (*iterTP)->m_LinkList.end();iterlink++)
		{
			if ((*iterlink)->m_Length<pT->m_Length&&(*iterlink)->m_LinkType==4/*siding*/)//for long trains
			{
				feasibleflag=false;
				break;
			}
			if (pT->m_HazMat==true&&(*iterlink)->m_LinkType==4)//for hazardous train
			{
				feasibleflag = false;
				break;
			}

		}		

		if (feasibleflag)
		{
			pT->m_TrainPathList.push_back(*iterTP);
			pT2->m_TrainPathList.push_back(*iterTP);
			
		}
		
	}

    ASSERT(pT->m_TrainPathList.size()>0);

	std::vector<TrainPath*> j_vector;
	for (iterTP = pT->m_TrainPathList.begin(); iterTP != pT->m_TrainPathList.end(); iterTP++)
	{
		j_vector.push_back(*iterTP);
	}

	
	pT->m_TrainPathList.clear();
	pT2->m_TrainPathList.clear();
	

	std::sort(j_vector.begin(),j_vector.end(),MyComparatorTotalTravelTime);// sorted by m_totalminimumtraveltime
	
	for (iterTPV = j_vector.begin(); iterTPV != j_vector.end(); iterTPV++)
	{
		if(pT->m_TrainPathList.size()<pT->MaxNumberofTrainPaths)
		{
			bool flagin = false;
			for (iterTP=pT->m_TrainPathList.begin();iterTP!=pT->m_TrainPathList.end();iterTP++)
			{
				if ((*iterTP)->m_PathID==(*iterTPV)->m_PathID)
				{
					flagin=true;
				}
			}
			if (!flagin)
			{
				pT->m_TrainPathList.push_back(*iterTPV);
				pT2->m_TrainPathList.push_back(*iterTPV);

			}
	
		}
		else			
		{
			break;
		}
	}
	ASSERT(pT->m_TrainPathList.size()>0);
	  
	return true;
}


int g_InitializeLogFiles()
{
	g_AppStartTime = CTime::GetCurrentTime();
	CString s;
	if (g_PathBasedScheduling)
	{
		s.Format("_PathBased-%d_PathsPerTrain-%d_MinuteDivision-%d",g_PathBasedScheduling,g_PathsPerTrain,g_MinuteDivision);
	}
	else
	{
		s.Format("_PathBased-%d_MinuteDivision-%d",g_PathBasedScheduling,g_MinuteDivision);
	}
	
	g_LogFile.open (".\\summary_log\\Summary"+s+".csv", ios::out);
	if (g_LogFile.is_open())
	{
		g_LogFile.width(12);
		g_LogFile.precision(3);
		g_LogFile.setf(ios::fixed);
	}else
	{
		cout << "File summary.csv cannot be opened, and it mig ht be locked by another program or the target data folder is read-only." << endl;
		cin.get();// pause
		return 0;
	}

	cout << "FastTrain: A Fast Open-Source Networked Train Rerouting and Rescheduling Engine"<< endl;
	cout << "http://code.google.com/p/fast-train/"<< endl;
	cout << "Version 1.0, Release Date 04/30/2013."<< endl;

	return 1;
}

bool g_ReadSchedulingSettings()
{
	TCHAR IniFilePath_FT[_MAX_PATH] = _T("./FTSettings.ini");

	g_OptimizationHorizon= g_GetPrivateProfileInt("optimization", "OptimizationHorizon", 1440, IniFilePath_FT);
	g_OptimizationInterval=g_GetPrivateProfileInt("optimization", "OptimizationInterval", 1440, IniFilePath_FT);	
    g_PathBasedScheduling = g_GetPrivateProfileInt("optimization", "PathBasedScheduling", 0, IniFilePath_FT);		
	g_PathsPerTrain = g_GetPrivateProfileInt("optimization", "NumberofPathsPerTrain", 0, IniFilePath_FT);	
	g_MinuteDivision = g_GetPrivateProfileInt("optimization", "MinuteDivision", 0, IniFilePath_FT);

	g_MaxNumberOfLRIterations = g_GetPrivateProfileInt("Lagrangian", "MaxNumberofLRIterations", 2, IniFilePath_FT);	
	g_MinimumStepSize = g_GetPrivateProfileFloat("Lagrangian", "MinimumStepSize", 0, IniFilePath_FT);
	g_SafetyHeadway = g_GetPrivateProfileInt("Lagrangian", "SafetyHeadway", 2, IniFilePath_FT);
	g_MaxTrainWaitingTime = g_GetPrivateProfileInt("Lagrangian", "MaxTrainWaitingTime", 0, IniFilePath_FT);
	g_MaxSlackTimeAtDeparture = g_GetPrivateProfileInt("Lagrangian", "MaxSlackTimeAtDeparture", 0, IniFilePath_FT);
	g_NumberOfIterationsWithMemory = g_GetPrivateProfileInt("Lagrangian", "NumberOfIterationsWithMemory", 0, IniFilePath_FT);
	
	//all above variables regarding time are in minutes. Now change to any time unit according to g_MinuteDivision
	g_OptimizationHorizon = g_OptimizationHorizon*g_MinuteDivision;
	g_SafetyHeadway = g_SafetyHeadway*g_MinuteDivision;
	g_MaxTrainWaitingTime = g_MaxTrainWaitingTime*g_MinuteDivision;
	g_MaxSlackTimeAtDeparture = g_MaxSlackTimeAtDeparture*g_MinuteDivision;

	return true;

}

bool g_ExportTimetableDataToCSVFile(int iterationnumber,std::vector<CTrain*> trainvector)
{
	FILE* st;
	CString s,s1;	
	s.Format("%d",iterationnumber);
	fopen_s(&st,".\\internal_timetable\\output_timetable_"+s+".csv","w");
		
	if(st!=NULL)
	{
		fprintf(st, "train ID,train type,origin,destination,departure time,# of nodes, actual trip time,,node, time_stamp, node_position\n");
		
		float TotalTripTime = 0;

		for(unsigned int v = 0; v<trainvector.size(); v++)
		{

			CTrain* pTrain = trainvector[v];

			pTrain->m_ActualTravelTime = pTrain->m_aryTN[pTrain->m_NodeSize -1].NodeArrivalTimestamp - pTrain->m_aryTN[0].NodeArrivalTimestamp;


			int n;
			int number_of_physical_nodes = 0;

			for( n = 0; n< pTrain->m_NodeSize; n++)
			{
				int NodeID = pTrain->m_aryTN[n].NodeID;

				if(g_NodeIDToNumberMap[NodeID]<MAX_PHYSICAL_NODE_NUMBER )
					number_of_physical_nodes++;

			}

			fprintf(st,"%d,%d,%d,%d,%d,%d\n", pTrain->m_TrainID , g_NodeIDToNumberMap [pTrain->m_OriginNodeID] ,g_NodeIDToNumberMap[pTrain->m_DestinationNodeID ],pTrain->m_EntryTime,
				number_of_physical_nodes,pTrain->m_ActualTravelTime);

			
			TotalTripTime += pTrain->m_ActualTravelTime ;

			for( n = 0; n< pTrain->m_NodeSize; n++)
			{
				int NodeID = pTrain->m_aryTN[n].NodeID;
				if(g_NodeIDToNumberMap[NodeID]<MAX_PHYSICAL_NODE_NUMBER)
				{
				fprintf(st,",,,,,,,,%d,%d\n", g_NodeIDToNumberMap[NodeID], pTrain->m_aryTN[n].NodeArrivalTimestamp);
				}

			}
		}

		fclose(st);

		

		return true;
	}else
	{
		cout << "Error: File output_timetable.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
	return false;
}


bool g_ExportTimetableDataToXMLFile(int iterationnumber,std::vector<CTrain*> trainvector)
{
	FILE* st;
	CString s;
	s.Format("%d",iterationnumber);
	fopen_s(&st,".\\internal_timetable\\output_timetable_"+s+".XML","w");

	if(st!=NULL)
	{		
		fprintf(st, "###########################################################################\n");
		fprintf(st,"<solution territory='RAS DATA SET 3'>\n");
		fprintf(st,"	<trains>\n");
		for(unsigned int v = 0; v<trainvector.size(); v++)
		{

			CTrain* pTrain = trainvector[v];
			//fprintf(st,"		<train index='%d'>\n",pTrain->m_TrainID);
			fprintf(st,"			<movements>\n");

			for(int n = 0; n< pTrain->m_NodeSize-1; n++)
			{
				int fromNodeID = pTrain->m_aryTN[n].NodeID;
				int toNodeID = pTrain->m_aryTN[n+1].NodeID;
				int fromnodenumber= g_NodeIDToNumberMap[fromNodeID];
				int tonodenumber= g_NodeIDToNumberMap[toNodeID];
				int entrytime = pTrain->m_aryTN[n].NodeArrivalTimestamp*60/g_MinuteDivision;//to seconds. as nexta gui requires this
				int exittime = pTrain->m_aryTN[n+1].NodeArrivalTimestamp*60/g_MinuteDivision;
				int entrytimeinminute = pTrain->m_aryTN[n].NodeArrivalTimestamp/g_MinuteDivision;
				int exittimeinminute = pTrain->m_aryTN[n+1].NodeArrivalTimestamp/g_MinuteDivision;

				fprintf(st,"				<movement arc='(%d,%d)' entry='%d.000' exit='%d.000' entryinminute='%d.000' exitinminute='%d.000' />\n",
					fromnodenumber,tonodenumber,entrytime,exittime,entrytimeinminute,exittimeinminute);
			}			
			fprintf(st,"				<destination entry='%d.000'/>\n",(int)(pTrain->m_aryTN[pTrain->m_NodeSize-1].NodeArrivalTimestamp*1.0/g_MinuteDivision+1.0f)-1);
			fprintf(st,"			</movements>\n");
			fprintf(st,"		</train>\n");
		}
		fprintf(st,"	<trains>\n");
		fprintf(st,"<solution>\n");
		fprintf(st, "###########################################################################");
		fclose(st);
		return true;
	}else
	{
		cout << "Error: File output_timetable.xml cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	return false;
}



bool g_ExportTDLabelCostToTXTFile(NetworkForSP *pN,int departure_time,int* ids)
{

	ASSERT(pN!=NULL);

	FILE* st;
	CString s,s1;	
	s.Format("%d-%d",g_CurrentLRIterationNumber,pN->pT->m_TrainID);
	fopen_s(&st,".\\td_label_cost\\TDLabelCost"+s+".CSV","w");
	s=" ,";
	for(int t=0; t <pN->m_OptimizationHorizon; t+=pN->m_OptimizationTimeInveral)
	{
		s1.Format("%d,",t);
		s=s+s1;
	}
	fprintf(st,s);
	fprintf(st,"\n");
	if(st!=NULL)
	{		
		for (int i=0;i<MAX_PHYSICAL_NODE_NUMBER;i++)
		{
			if (ids[i]==-1)
			{
				break;
			}
			if (!g_PathBasedScheduling)
			{
				s.Format("%d,",g_NodeIDToNumberMap[ids[i]]);
			}
			else
			{
				s.Format("%d,",ids[i]);
			}
			
			for(int t=0; t <pN->m_OptimizationHorizon; t+=pN->m_OptimizationTimeInveral)
			{
				s1.Format("%.1f,",pN->TD_LabelCostAry[ids[i]][t]);
				s=s+s1;
			}
			fprintf(st,s);
			fprintf(st,"\n");
		}
		
		
		fclose(st);
		return true;
	}else
	{
		cout << "Error: File TDLabelCost.CSV cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
}

bool g_ExportResourceUsageCount()
{
	FILE* st;
	CString s,s1;	
	s.Format("%d",g_CurrentLRIterationNumber);
	fopen_s(&st,".\\resource_usagecount\\ResourceUsageCount"+s+".CSV","w");
	s=", ,";
	for(int t=0; t <g_OptimizationHorizon; t+=g_OptimizationInterval)
	{
		s1.Format("%d,",t);
		s=s+s1;
	}
	fprintf(st,s);
	fprintf(st,"\n");
	if(st!=NULL)
	{	
		std::set<CLink*>::iterator iLink;
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{
			s.Format("%d,%d,",(*iLink)->m_FromNodeNumber,(*iLink)->m_ToNodeNumber);
			for(int t=0; t< g_OptimizationHorizon; t++)
			{	
				s1.Format("%d,",(*iLink)->m_ResourceAry[t].UsageCount);			
				s=s+s1;			
			}
			fprintf(st,s);
			fprintf(st,"\n");
		}

		fclose(st);
		return true;
	}else
	{
		cout << "Error: File ResourcePrice.CSV cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
}

bool g_ExportResourcePriceToTXTFile()
{
	FILE* st;
	CString s,s1;	
	s.Format("%d",g_CurrentLRIterationNumber);
	fopen_s(&st,".\\resource_price\\ResourcePrice"+s+".CSV","w");
	s=", ,";
	for(int t=0; t <g_OptimizationHorizon; t+=g_OptimizationInterval)
	{
		s1.Format("%d,",t);
		s=s+s1;
	}
	fprintf(st,s);
	fprintf(st,"\n");
	if(st!=NULL)
	{	
		std::set<CLink*>::iterator iLink;
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{
			s.Format("%d,%d,",(*iLink)->m_FromNodeNumber,(*iLink)->m_ToNodeNumber);
			for(int t=0; t< g_OptimizationHorizon; t++)
			{	
				s1.Format("%.2f,",(*iLink)->m_ResourceAry[t].Price);			
				s=s+s1;			
			}
			fprintf(st,s);
			fprintf(st,"\n");
		}

		fclose(st);
		return true;
	}else
	{
		cout << "Error: File ResourcePrice.CSV cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
}


bool g_ExportResourceUsageToTXTFile(CLink*pl,int t)
{
	ASSERT(pl!=NULL&&t>-1);

	FILE* st;
	CString s,s1;	
	s.Format("%d-%d-%d-%d",g_CurrentLRIterationNumber,pl->m_FromNodeNumber,pl->m_ToNodeNumber,t);
	fopen_s(&st,".\\exceeded_resource_usage\\ResourceUsage_"+s+".txt","w");	
	if(st!=NULL)
	{		
					
			for (int i=0;i<MAX_TRAIN_NUMBER;i++)
			{
				if(pl->m_ResourceAry[t].TrainUseFlag[i]==1)
				{
					s1.Format("time:%d,train index:%d,",t,i);
					fprintf(st,s1);
					fprintf(st,"\n");
				}
			}
		

		fclose(st);
		return true;
	}else
	{
		cout << "Error: File TDLabelCost.CSV cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
}


void g_FreeMemory()
{
	cout << "Free memory... " << endl;
	
	std::set<CNode*>::iterator iterNode;
	std::set<CLink*>::iterator iterLink;
	std::set<TrainPaths*>::iterator iterTrainPaths;
	std::set<TrainPath*>::iterator iterTrainPath;
	std::vector<CTrain*>::iterator iterTrain;	

	cout << "Free node set... " << endl;

	for (iterNode = g_NodeSet.begin(); iterNode != g_NodeSet.end(); iterNode++)
	{
		delete *iterNode;
	}

	g_NodeSet.clear();
	g_NodeIDMap.clear();
	g_NodeIDToNumberMap.clear();

	cout << "Free link set... " << endl;
	for (iterLink = g_LinkSet.begin(); iterLink != g_LinkSet.end(); iterLink++)
	{
		delete *iterLink;
	}

	g_LinkSet.clear();
	g_LinkIDMap.clear();
	g_NodeIDtoLinkMap.clear();

	cout << "Free train set... " << endl;
	for (iterTrain = g_TrainVector.begin(); iterTrain != g_TrainVector.end(); iterTrain++)
	{
		delete *iterTrain;
	}

	for (iterTrain = g_TrainVector2.begin(); iterTrain != g_TrainVector2.end(); iterTrain++)
	{
		delete *iterTrain;
	}



	cout << "Free train path set... " << endl;
	for (iterTrainPaths = g_TrainPathsSet.begin(); iterTrainPaths != g_TrainPathsSet.end(); iterTrainPaths++)
	{
		delete *iterTrainPaths;
	}

	g_TrainPathsSet.clear();
	g_TrainPathsIDMap.clear();

	for (iterTrainPath = g_TrainPathSet.begin(); iterTrainPath != g_TrainPathSet.end(); iterTrainPath++)
	{
		delete *iterTrainPath;
	}

	g_TrainPathSet.clear();
	g_TrainPathIDMap.clear();
	g_NodeIDtoTrainPathsMap.clear();

	g_LogFile.close();
	
}

//entry point of the program
int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{	
	//1.Initialize MFC and return on failure
	if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: MFC initialization failed\n"));
		return false;
	}

	//2. Read configuration parameters 
	if(g_ReadSchedulingSettings()==false)
		return false;

    //3. Set log file name, path etc.
	if(g_InitializeLogFiles()==0) 
		return false;
    
	//4. Read network, train information etc.
	g_ReadInputFiles();
	
	//5. Simultaneous routing and scheduling trains by Lagrangian relaxation approach
         //LR is used for obtaining lower bound and priority rule based algorithm (train by train) 
	     // is used for obtaining upper bound
	g_Timetable_Optimization_Lagrangian_Method();

	//6. Freeing memory is required because of C++ language
	g_FreeMemory();
	
	//7. Print prompts
	g_ProgramStop();

	//8. Nothing unexpected happened
	return true;
}




