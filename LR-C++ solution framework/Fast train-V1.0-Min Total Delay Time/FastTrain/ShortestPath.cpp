//  Portions Copyright 2013 Lingyun meng (lymeng@bjtu.edu.cn) and Xuesong Zhou (xzhou99@gmail.com)

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of FastTrain Version 3 (Open-source).

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

//shortest path calculation

// note that the current implementation is only suitable for time-dependent minimum time shortest path on FIFO network, rather than time-dependent minimum cost shortest path
// the key reference (1) Shortest Path Algorithms in Transportation Models http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.51.5192
// (2) most efficient time-dependent minimum cost shortest path algorithm for all departure times
// Time-dependent, shortest-path algorithm for real-time intelligent vehicle highway system applications&quot;, Transportation Research Record 1408 ?Ziliaskopoulos, Mahmassani - 1993
 
#include "stdafx.h"
#include "Network.h"
#include "FastTrain.h"


extern int g_MaxLinkSize;
extern int g_MaxNodeSize;
extern std::map<int,int>g_LinkMPIDToLinkIDMap;


int  NetworkForSP::GetLinkNoByNodeIndex(int usn_index, int dsn_index)
{
	int LinkNo = -1;
	for(int i=0; i < m_OutboundSizeAry[usn_index]; i++)
	{

		if(m_OutboundNodeAry[usn_index][i] == dsn_index)
		{
			LinkNo = m_OutboundLinkAry[usn_index][i];
			return LinkNo;
		}
	}

	cout << " Error in GetLinkNoByNodeIndex " ;

	g_ProgramStop();

	return -1;
}


void NetworkForSP::BuildSpaceTimeNetworkForTimetabling(std::set<CNode*>* p_NodeSet, std::set<CLink*>* p_LinkSet, CTrain* pTrain)
{
   if (!g_PathBasedScheduling||pTrain==NULL)//note that pTrain==Null means this function is called by the function "GeneratePhysicalPaths"
   {
	   std::list<CNode*>::iterator iterNode;
	   std::set<CLink*>::iterator iterLink;

	   m_NodeSize = p_NodeSet->size();
	   m_LinkSize = p_LinkSet->size();

	   int FromID, ToID;
        int i,j,t;
	 

	   for(i=0; i< m_NodeSize; i++)
	   {
		   m_OutboundSizeAry[i] = 0;
		   m_InboundSizeAry[i] = 0;

	   }


	   for (i=0;i<m_LinkSize;i++)
	   {
		   m_FromIDAry[i]=-1;
		   m_ToIDAry[i]=-1;
	   }

	   for (i=0;i<m_NodeSize;i++)
	   {
		   for (j=0;j<m_AdjLinkSize;j++)
		   {
			   m_OutboundNodeAry[i][j]=-1;
			   m_OutboundLinkAry[i][j]=-1;
			   m_InboundLinkAry[i][j]=-1;
		   }
	   }

	   // add physical links 

	   for(iterLink = p_LinkSet->begin(); iterLink != p_LinkSet->end(); iterLink++)
	   {	   

				 if (pTrain==NULL)
				 {
					 FromID = (*iterLink)->m_FromNodeID;
					 ToID   = (*iterLink)->m_ToNodeID;
				 } 
				 else
				 {
					 if (pTrain->m_Direction==1)//need to consider the direction of the train under consideration
					 {
						 //east bound
						 FromID = (*iterLink)->m_FromNodeID;
						 ToID   = (*iterLink)->m_ToNodeID;
					 } 
					 else
					 {
						 FromID = (*iterLink)->m_ToNodeID;
						 ToID   = (*iterLink)->m_FromNodeID;
					 }
				 }
			  

			   m_FromIDAry[(*iterLink)->m_LinkID] = FromID;
			   m_ToIDAry[(*iterLink)->m_LinkID]   = ToID;

			   m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
			   m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = (*iterLink)->m_LinkID ;
			   m_OutboundSizeAry[FromID] +=1;

			   m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = (*iterLink)->m_LinkID  ;
			   m_InboundSizeAry[ToID] +=1;
			   
			   ASSERT(m_AdjLinkSize > m_OutboundSizeAry[FromID]);

			   m_LinkTDMaxWaitingTimeAry[(*iterLink)->m_LinkID] = (*iterLink)->GetTrainMaxWaitingTimeInMinuteDivision ();  // in the future, we can extend it to time-dependent max waiting time

			   for(t=0; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
			   {
				   if (pTrain==NULL)
				   {
					   m_LinkTDTimeAry[(*iterLink)->m_LinkID][t] = (*iterLink)->GetTrainRunningTimeInMinuteDivision(1,1);  // in the future, we can extend it to time-dependent running time		
				   } 
				   else
				   {
					   double a = pTrain->m_SpeedMultiplier;
					   m_LinkTDTimeAry[(*iterLink)->m_LinkID][t] = (*iterLink)->GetTrainRunningTimeInMinuteDivision(a,pTrain->m_Direction);  // in the future, we can extend it to time-dependent running time		
				   }
				   
				   m_LinkTDCostAry[(*iterLink)->m_LinkID][t] = (*iterLink)->m_ResourceAry[t].Price;  // for all train types

			   }

		   
	   }
	   m_LinkSize = p_LinkSet->size();
   } 
   else//sequential
   { 
	   //below is for (multiple) physical paths based train scheduling, we ignore the g_NodeSet, g_LinkSet	  
	   ASSERT(pTrain!=NULL);
	   ASSERT(pTrain->m_TrainPathList.size()>0);

	   std::list<CNode*>::iterator iterNode;
	   std::set<CNode*>::iterator iterNode2;
	   std::list<CLink*>::iterator iterLink;
	   std::set<CLink*>::iterator iterLink2;
	   std::list<TrainPath*>::iterator iterPath;

	   int FromID, ToID;
	   int i,j,t;	   
	   m_LinkSize = 0;
	   m_NodeSize = 0;
	   int locallinkcounter = 0;
	   int localnodecounter = 0;
	   CNode*pnode = NULL;

	  	for (iterLink2=g_LinkSet.begin();iterLink2!=g_LinkSet.end();iterLink2++)
		{
			for (int i=0;i<MAX_PATH_NUMBER_PER_TRAIN;i++)
			{
				(*iterLink2)->m_MPLinkID[i]=-1;
				(*iterLink2)->m_FromMPNodeID[i]=-1;
				(*iterLink2)->m_ToMPNodeID[i]=-1;
			}
		}
	   //update m_MPLinkID and m_MPNodeID
	   bool flag = false;
	   CLink*plk = NULL;
	   plk = g_LinkIDMap[0];
	   for (iterPath = pTrain->m_TrainPathList.begin();iterPath!=pTrain->m_TrainPathList.end();iterPath++)
	   {		  
		  int pretoid = 0;
		  int pathid = (*iterPath)->m_PathID;
		   for(iterLink = (*iterPath)->m_LinkList.begin(); iterLink != (*iterPath)->m_LinkList.end(); iterLink++)
		   {
			    //update link counter			
				(*iterLink)->m_MPLinkID[pathid] = locallinkcounter;	
				g_LinkMPIDToLinkIDMap[locallinkcounter]=(*iterLink)->m_LinkID;
				locallinkcounter++;
				
				//update node counter
				if (iterLink == (*iterPath)->m_LinkList.begin())
				{
					(*iterLink)->m_FromMPNodeID[pathid] = localnodecounter;
					if (pTrain->m_Direction==1)
					{
						g_NodeMPIDToNodeIDMap[localnodecounter] = (*iterLink)->m_FromNodeID;
					}
					else
					{
						g_NodeMPIDToNodeIDMap[localnodecounter] = (*iterLink)->m_ToNodeID;
					}
					localnodecounter++;

					(*iterLink)->m_ToMPNodeID[pathid] = localnodecounter;
					if (pTrain->m_Direction==1)
					{
						g_NodeMPIDToNodeIDMap[localnodecounter] = (*iterLink)->m_ToNodeID;
					}
					else
					{
						g_NodeMPIDToNodeIDMap[localnodecounter] = (*iterLink)->m_FromNodeID;
					}

					pretoid = localnodecounter;
					localnodecounter++;
				}
				else
				{
				    (*iterLink)->m_FromMPNodeID[pathid] = pretoid; 
					(*iterLink)->m_ToMPNodeID[pathid] = localnodecounter;
					if (pTrain->m_Direction==1)
					{
						g_NodeMPIDToNodeIDMap[localnodecounter] = (*iterLink)->m_ToNodeID;
					}
					else
					{
						g_NodeMPIDToNodeIDMap[localnodecounter] = (*iterLink)->m_FromNodeID;
					}
					pretoid = localnodecounter;
					localnodecounter++;
				}
		   }		   
	   }	   

	   //update m_linksize and m_nodesize
	   m_LinkSize = locallinkcounter+pTrain->m_TrainPathList.size()*2;
	   m_NodeSize = localnodecounter+2;

	   //reset m_OutboundSizeAry, m_InboundSizeAry,m_FromIDAry, m_ToIDAry	
	   for(i=0; i< g_MaxNodeSize; i++)
	   {
		   m_OutboundSizeAry[i] = 0;
		   m_InboundSizeAry[i] = 0;
	   }

	   for (i=0;i<g_MaxLinkSize;i++)
	   {
		   m_FromIDAry[i]=-1;
		   m_ToIDAry[i]=-1;
	   }

	   for (i=0;i<g_MaxNodeSize;i++)
	   {
		   for (j=0;j<m_AdjLinkSize;j++)
		   {
			   m_OutboundNodeAry[i][j]=-1;
			   m_OutboundLinkAry[i][j]=-1;
			   m_InboundLinkAry[i][j]=-1;
		   }
	   }

	   //add virtual source node and corresponding links
	   int VirtualLinkToID = -1;
	   int VirtualLinkFromID = -1;
	   pTrain->m_OriginMPNodeID=localnodecounter;	   

	   for (iterPath = pTrain->m_TrainPathList.begin();iterPath!=pTrain->m_TrainPathList.end();iterPath++)
	   {	
		   int pathid = (*iterPath)->m_PathID;
		   iterLink = (*iterPath)->m_LinkList.begin();
		   
		   g_LinkMPIDToLinkIDMap[locallinkcounter]=-1;

		   VirtualLinkToID = (*iterLink)->m_FromMPNodeID[pathid];			
		   
		
		   m_FromIDAry[locallinkcounter] = localnodecounter;
		   m_ToIDAry[locallinkcounter]   = VirtualLinkToID;

		   m_OutboundNodeAry[localnodecounter][m_OutboundSizeAry[localnodecounter]] = VirtualLinkToID;
		   m_OutboundLinkAry[localnodecounter][m_OutboundSizeAry[localnodecounter]] = locallinkcounter ;
		   m_OutboundSizeAry[localnodecounter] +=1;

		   m_InboundLinkAry[VirtualLinkToID][m_InboundSizeAry[VirtualLinkToID]] = locallinkcounter;
		   m_InboundSizeAry[VirtualLinkToID] +=1;

		   m_LinkTDMaxWaitingTimeAry[locallinkcounter] = 0;  // in the future, we can extend it to time-dependent max waiting time

		   for(t=0; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
		   {
			   m_LinkTDTimeAry[locallinkcounter][t] = 0;  // in the future, we can extend it to time-dependent running time		
			   m_LinkTDCostAry[locallinkcounter][t] = 0;  // for all train types
		   }
		   locallinkcounter++;	   

	   }

	    
	   //add virtual sink node and corresponding links
	   localnodecounter++;
	   pTrain->m_DestinationMPNodeID=localnodecounter;
	   for (iterPath = pTrain->m_TrainPathList.begin();iterPath!=pTrain->m_TrainPathList.end();iterPath++)
	   {	
		   g_LinkMPIDToLinkIDMap[locallinkcounter]=-1;
		   int pathid = (*iterPath)->m_PathID;
		   int lc = 0;
		   for (iterLink = (*iterPath)->m_LinkList.begin();iterLink!=(*iterPath)->m_LinkList.end();iterLink++)
		   {
			   if (lc+1==(*iterPath)->m_LinkList.size())
			   {
				   break;
			   }
			   lc++;
		   }
		   		   	   	
		   VirtualLinkFromID = (*iterLink)->m_ToMPNodeID[pathid];
		 

		   m_FromIDAry[locallinkcounter] = VirtualLinkFromID;
		   m_ToIDAry[locallinkcounter]   = localnodecounter;

		   m_OutboundNodeAry[VirtualLinkFromID][m_OutboundSizeAry[VirtualLinkFromID]] = localnodecounter;
		   m_OutboundLinkAry[VirtualLinkFromID][m_OutboundSizeAry[VirtualLinkFromID]] = locallinkcounter ;
		   m_OutboundSizeAry[VirtualLinkFromID] +=1;

		   m_InboundLinkAry[localnodecounter][m_InboundSizeAry[localnodecounter]] = locallinkcounter;
		   m_InboundSizeAry[localnodecounter] +=1;

		   m_LinkTDMaxWaitingTimeAry[locallinkcounter] = 0;  

		   for(t=0; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
		   {
			   m_LinkTDTimeAry[locallinkcounter][t] = 0;  	
			   m_LinkTDCostAry[locallinkcounter][t] = 0;  
		   }
		   locallinkcounter++;
	   }
	   
	   //fulfill m_OutboundSizeAry, m_InboundSizeAry,m_FromIDAry, m_ToIDAry	
	   for (iterPath = pTrain->m_TrainPathList.begin();iterPath!=pTrain->m_TrainPathList.end();iterPath++)
	   {	
		   int pathid = (*iterPath)->m_PathID;
		   for(iterLink = (*iterPath)->m_LinkList.begin(); iterLink != (*iterPath)->m_LinkList.end(); iterLink++)
		   {
			   					
				   FromID = (*iterLink)->m_FromMPNodeID[pathid];				
				   ToID = (*iterLink)->m_ToMPNodeID[pathid];

				   m_FromIDAry[(*iterLink)->m_MPLinkID[pathid]] = FromID;
				   m_ToIDAry[(*iterLink)->m_MPLinkID[pathid]]   = ToID;

				   m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
				   m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = (*iterLink)->m_MPLinkID[pathid];
				   m_OutboundSizeAry[FromID] +=1;

				   m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = (*iterLink)->m_MPLinkID[pathid];
				   m_InboundSizeAry[ToID] +=1;
				  
				   ASSERT(m_AdjLinkSize > m_OutboundSizeAry[FromID]);

				   m_LinkTDMaxWaitingTimeAry[(*iterLink)->m_MPLinkID[pathid]] = (*iterLink)->GetTrainMaxWaitingTimeInMinuteDivision ();  // in the future, we can extend it to time-dependent max waiting time

				   for(t=0; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
				   {
					   m_LinkTDTimeAry[(*iterLink)->m_MPLinkID[pathid]][t] = (*iterLink)->GetTrainRunningTimeInMinuteDivision(pTrain->m_SpeedMultiplier,pTrain->m_Direction);  // in the future, we can extend it to time-dependent running time		
					   m_LinkTDCostAry[(*iterLink)->m_MPLinkID[pathid]][t] = (*iterLink)->m_ResourceAry[t].Price;  // for all train types
					   ASSERT (m_LinkTDCostAry[(*iterLink)->m_MPLinkID[pathid]][t]>=0);					   
				   }
			   			   
		   }		 
	   }
   }
	
}


double GetLocalResourceCost(NetworkForSP* pNFSP,int stime,int etime,int LinkNo)
{
	
	double localresourcecost=0;
	for (int i=stime;i<=etime;i++)//attention, here i<=etime, not i<etime
	{
		if (i>=0&&i<g_OptimizationHorizon)
		{
			localresourcecost+=pNFSP->m_LinkTDCostAry[LinkNo][i];
		}								   
	}
	return localresourcecost;
}

double GetLocalResourceCost(CLink* pLink,int stime,int etime)
{
	ASSERT(pLink!=NULL&&stime<=etime);
	double localresourcecost=0;
	for (int i=stime;i<=etime;i++)//attention, here i<=etime, not i<etime
	{
		if (i>=0&&i<g_OptimizationHorizon)
		{
			localresourcecost+=pLink->m_ResourceAry[i].Price;
		}								   
	}
	return localresourcecost;
}


bool NetworkForSP::OptimalTDLabelCorrecting_DoubleQueue(int origin, int departure_time, int destination, int AllowableSlackAtDeparture, int CostPerUnitTimeStopped,CTrain * pT)
// time-dependent label correcting algorithm with double queue implementation
{	
		int i;
		int debug_flag = 0;//set 1 to debug the detail information
		int *idsAry = new int[MAX_PHYSICAL_NODE_NUMBER];
		if(debug_flag)
		{	
			;//TRACE("\nCompute shortest path from %d at time %d",g_NodeIDToNumberMap[origin], departure_time);
		}

		bool bFeasiblePathFlag  = false;

		if(m_OutboundSizeAry[origin]== 0)
		{
			return false;
		}

		for(i=0; i <m_NodeSize; i++) //Initialization for all nodes
		{
			NodeStatusAry[i] = 0;

			for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
			{
				TD_LabelCostAry[i][t] = MAX_SPLABEL;
				TD_NodePredAry[i][t] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
				TD_TimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
			}

		}	

		//TD_LabelCostAry[origin][departure_time] = 0;
		//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node
		//+1 in "departure_time + 1+ MaxAllowedStopTime" is to allow feasible value for t = departure time
		for(int t=departure_time; t < departure_time + 1+ AllowableSlackAtDeparture; t+=m_OptimizationTimeInveral)
		{
			if (t>=m_OptimizationHorizon)
			{
				break;
			}		
			
			TD_LabelCostAry[origin][t]= 0;
			
		}

		SEList_clear();
		SEList_push_front(origin);

		int nodenumber = 0;
		while(!SEList_empty())
		{
			int FromID  = SEList_front();//pop a node FromID for scanning

			idsAry[nodenumber]=FromID;
			nodenumber++;

			SEList_pop_front();  // remove current node FromID from the SE list


			NodeStatusAry[FromID] = 2;  //scanned

			//scan all outbound nodes of the current node
			for(i=0; i<m_OutboundSizeAry[FromID]; i++)  // for each arc (i,j) belong A(i)
			{
				int LinkNo = m_OutboundLinkAry[FromID][i];
				int physicallinkno = -1;

				if (g_PathBasedScheduling)//needs to map to physical link id
				{					
					physicallinkno = g_LinkMPIDToLinkIDMap[LinkNo];
				}
				else
				{
					physicallinkno = LinkNo;
				}

				int ToID = m_OutboundNodeAry[FromID][i];
		
				if(ToID == origin)  // remove possible loop back to the origin
					continue;

		        int MaxAllowedStopTime = 0;
				if (physicallinkno<0)//virtual source link or sink link, MaxAllowedStopTime should be 0
				{
					MaxAllowedStopTime = 0;
				}
				else//physical link, use LinkNo below, 1)if path_based scheduling, m_LinkTDTimeAry is constructed according to physical travel time corresonding to LinkNo index
					//                                 2)if not, LinkNo equals to physical link id
				{
					MaxAllowedStopTime = m_LinkTDMaxWaitingTimeAry[LinkNo];
				}				 

				// for each time step, starting from the departure time			
				for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
				{				
					if(TD_LabelCostAry[FromID][t]<MAX_SPLABEL-1)  // for feasible time-space point only
					{   
						for(int time_stopped = 0; time_stopped <= MaxAllowedStopTime; time_stopped++)
						{
							int NewToNodeArrivalTime = 0;
							if (physicallinkno<0)//virtual source link or sink link, travel time should be 0
							{
								NewToNodeArrivalTime = (int)(t + time_stopped + 0);  // time-dependent travel times for different train type
							}
							else
							{
								NewToNodeArrivalTime = (int)(t + time_stopped + m_LinkTDTimeAry[LinkNo][t]);  // time-dependent travel times for different train type														
							}
							if (NewToNodeArrivalTime > (m_OptimizationHorizon -1))  //prevent from bound error
								NewToNodeArrivalTime = (m_OptimizationHorizon-1);
							
							float NewCost;
							double localresourcecost= 0;
							
							if (physicallinkno<0)//virtual source link or sink link in path based scheduling, local resource cost should be 0
							{
								localresourcecost = 0;
							}
							else
							{
								localresourcecost = GetLocalResourceCost(this,t,NewToNodeArrivalTime+g_SafetyHeadway-1,LinkNo);	
							}
														
							
							if (physicallinkno<0)//virtual source link or sink link in path-based-scheduling case
							{
								NewCost  = TD_LabelCostAry[FromID][t] + localresourcecost /*+ 0*pT->m_CostPerUnitTimeRunning + time_stopped*pT->m_CostPerUnitTimeStopped*/;
							}
							else
							{
								NewCost  = TD_LabelCostAry[FromID][t] + localresourcecost /*+ m_LinkTDTimeAry[LinkNo][t]*pT->m_CostPerUnitTimeRunning + time_stopped*pT->m_CostPerUnitTimeStopped*/;
							}
								
							if(ToID==destination)
							{
								NewCost+=abs(NewToNodeArrivalTime-pT->m_PlannedCompletionTime);
							}
											

							if(NewCost < TD_LabelCostAry[ToID][NewToNodeArrivalTime] ) // we only compare cost at the downstream node ToID at the new arrival time t
							{	

								if(ToID == destination)
									bFeasiblePathFlag = true; 

								// update cost label and node/time predecessor

								TD_LabelCostAry[ToID][NewToNodeArrivalTime] = NewCost;
								TD_NodePredAry[ToID][NewToNodeArrivalTime] = FromID;  // pointer to previous physical NODE INDEX from the current label at current node and time
								TD_TimePredAry[ToID][NewToNodeArrivalTime] = t;  // pointer to previous TIME INDEX from the current label at current node and time

								// Dequeue implementation
								if(NodeStatusAry[ToID]==2) // in the SEList_TD before
								{
									SEList_push_front(ToID);
									NodeStatusAry[ToID] = 1;
								}
								if(NodeStatusAry[ToID]==0)  // not be reached
								{
									SEList_push_back(ToID);
									NodeStatusAry[ToID] = 1;
								}

							}
						}
					}
					//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
				}			

			}//end of for each link

		}//end of while

		idsAry[nodenumber]=-1;
		ASSERT(this->pT!=NULL);
		//g_ExportTDLabelCostToTXTFile(this,departure_time,ids);
		if (true)
		{
			//g_ExportTDLabelCostToTXTFile(this,departure_time,idsAry);
			delete []idsAry;
			return bFeasiblePathFlag;
		}		

		return bFeasiblePathFlag;	
	
}

int NetworkForSP::FindOptimalSolution(int origin, int departure_time, int destination, CTrain* pTrain, float& TripPrice)  // the last pointer is used to get the node array
{

	// step 1: scan all the time label at destination node, consider time cost
	// step 2: backtrack to the origin (based on node and time predecessors)
	// step 3: reverse the backward path
	// step 4: return final optimal solution

	// step 1: scan all the time label at destination node, consider time cost
	STrainNode tmp_AryTN[MAX_NODE_SIZE_IN_A_PATH]; //backward temporal solution

	float min_cost = MAX_SPLABEL;
	int min_cost_time_index = -1;

	for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
	{
		if(TD_LabelCostAry[destination][t] < min_cost)
		{
			min_cost = TD_LabelCostAry[destination][t];
			min_cost_time_index = t;
		}

	}

	for (int i=0;i<g_LinkSet.size();i++)
	{
		pTrain->TraversedLinkIDs[i]=-1;	
	}
	
	ASSERT(min_cost_time_index>0); // if min_cost_time_index ==-1, then no feasible path if found

	// step 2: backtrack to the origin (based on node and time predecessors)
	int	NodeSize = 0;	
	tmp_AryTN[NodeSize].NodeID = destination;//record the first node backward, destination node
	tmp_AryTN[NodeSize].NodeArrivalTimestamp = min_cost_time_index;

	NodeSize++;

	int PredTime = TD_TimePredAry[destination][min_cost_time_index];
	int PredNode = TD_NodePredAry[destination][min_cost_time_index];

	while(PredNode != origin && PredNode!=-1 && NodeSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
	{
		ASSERT(NodeSize< MAX_NODE_SIZE_IN_A_PATH-1);

		tmp_AryTN[NodeSize].NodeID = PredNode;
		tmp_AryTN[NodeSize].NodeArrivalTimestamp = PredTime;

		NodeSize++;

		//record current values of node and time predecessors, and update PredNode and PredTime
		int PredTime_cur = PredTime;
		int PredNode_cur = PredNode;

		PredNode = TD_NodePredAry[PredNode_cur][PredTime_cur];
		PredTime = TD_TimePredAry[PredNode_cur][PredTime_cur];

	}

	tmp_AryTN[NodeSize].NodeID = origin;
	tmp_AryTN[NodeSize].NodeArrivalTimestamp = PredTime;//originally it (predtime) is departure_time
	NodeSize++;

	// step 3: reverse the backward solution

	if(pTrain->m_aryTN !=NULL)
	{
		delete []pTrain->m_aryTN;
		pTrain->m_aryTN = NULL;
		pTrain->m_NodeSize=0;
	}

	pTrain->m_aryTN = new STrainNode[NodeSize];


	int i;
	for(i = 0; i< NodeSize; i++)
	{
		pTrain->m_aryTN[i].NodeID			= tmp_AryTN[NodeSize-1-i].NodeID;
		pTrain->m_aryTN[i].NodeArrivalTimestamp	= tmp_AryTN[NodeSize-1-i].NodeArrivalTimestamp;
		
	}
	
    pTrain->m_ResourceCostUsedByThisTrain=0;
	for(i = 0; i< NodeSize; i++)
	{
		if(i == NodeSize-1)  // destination
		{
			pTrain->m_aryTN[i].NodeDepartureTimestamp	= pTrain->m_aryTN[i].NodeArrivalTimestamp;
		}
		else
		{
			CLink* pLink = NULL;
			if (pTrain->m_Direction==1)
			{
				pLink=g_FindLinkWithNodeIDs(pTrain->m_aryTN[i].NodeID , pTrain->m_aryTN[i+1].NodeID);
			} 
			else
			{
				pLink=g_FindLinkWithNodeIDs(pTrain->m_aryTN[i+1].NodeID, pTrain->m_aryTN[i].NodeID);
			}
				
			ASSERT(pLink!=NULL);
			pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID;
			pTrain->TraversedLinkIDs[i] = pLink->m_LinkID;
			int LinkTravelTime = pLink->GetTrainRunningTimeInMinuteDivision (pTrain->m_SpeedMultiplier,pTrain->m_Direction);
			pTrain->m_aryTN[i].NodeDepartureTimestamp	= pTrain->m_aryTN[i+1].NodeArrivalTimestamp - LinkTravelTime ;
            pTrain->m_ResourceCostUsedByThisTrain  += GetLocalResourceCost(pLink,pTrain->m_aryTN[i].NodeArrivalTimestamp,pTrain->m_aryTN[i+1].NodeArrivalTimestamp+g_SafetyHeadway-1);
					
		}

	}
	
	//update related values for this train
	

	pTrain->m_ActualTravelTime = pTrain->m_aryTN[NodeSize-1].NodeArrivalTimestamp - pTrain->m_EntryTime ;//in minute division
	pTrain->m_RealizedCompletionTime = pTrain->m_aryTN[NodeSize-1].NodeArrivalTimestamp;
	pTrain->m_FreeRunningTime = pTrain->GetFreeRunningTimeAtTraversedLinksInMinuteDivision();//in minute division
	pTrain->m_WaitingTime=pTrain->m_ActualTravelTime-pTrain->m_FreeRunningTime;//in minute division
	double traveltimecost = pTrain->m_CostPerUnitTimeStopped*pTrain->m_WaitingTime+pTrain->m_CostPerUnitTimeRunning*pTrain->m_FreeRunningTime;
	//ASSERT(abs(pTrain->m_TripPrice-traveltimecost/*updated above*/-pTrain->m_ResourceCostUsedByThisTrain)<0.5);

	TripPrice = min_cost; 

	pTrain->m_TripPrice = TripPrice;
	//2) for minimum RAS cost: cost according to RAS function, and all resource price consumed by this train

	return NodeSize;
}

int NetworkForSP::FindOptimalSolutionMP(int origin, int departure_time, int destination, CTrain* pTrain, float& TripPrice)  // the last pointer is used to get the node array
{   //for multiple train path scheduling: origin and destination are mp id

	// step 1: scan all the time label at destination node, consider time cost
	// step 2: backtrack to the origin (based on node and time predecessors)
	// step 3: reverse the backward path
	// step 4: return final optimal solution

	// step 1: scan all the time label at destination node, consider time cost
	STrainNode tmp_AryTN[MAX_NODE_SIZE_IN_A_PATH]; //backward temporal solution

	float min_cost = MAX_SPLABEL;
	int min_cost_time_index = -1;

	for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
	{
		if(TD_LabelCostAry[destination][t] < min_cost)
		{
			min_cost = TD_LabelCostAry[destination][t];
			min_cost_time_index = t;
		}
	}

	for (int i=0;i<g_LinkSet.size();i++)
	{
		pTrain->TraversedLinkIDs[i]=-1;	
	}
	ASSERT(min_cost_time_index>0); // if min_cost_time_index ==-1, then no feasible path if founded

	// step 2: backtrack to the origin (based on node and time predecessors)

	int	NodeSize = 0;
	tmp_AryTN[NodeSize].NodeID = destination;//record the first node backward, destination node
	tmp_AryTN[NodeSize].NodeArrivalTimestamp = min_cost_time_index;

	NodeSize++;

	int PredTime = TD_TimePredAry[destination][min_cost_time_index];
	int PredNode = TD_NodePredAry[destination][min_cost_time_index];

	while(PredNode != origin && PredNode!=-1 && NodeSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
	{
		ASSERT(NodeSize< MAX_NODE_SIZE_IN_A_PATH-1);

		tmp_AryTN[NodeSize].NodeID = PredNode;
		tmp_AryTN[NodeSize].NodeArrivalTimestamp = PredTime;

		NodeSize++;

		//record current values of node and time predecessors, and update PredNode and PredTime
		int PredTime_cur = PredTime;
		int PredNode_cur = PredNode;

		PredNode = TD_NodePredAry[PredNode_cur][PredTime_cur];
		PredTime = TD_TimePredAry[PredNode_cur][PredTime_cur];

	}

	tmp_AryTN[NodeSize].NodeID = origin;
	tmp_AryTN[NodeSize].NodeArrivalTimestamp = PredTime;//originally it (predtime) is departure_time
	NodeSize++;

	// step 3: reverse the backward solution, need to map mpnodeid to physical node id

	if(pTrain->m_aryTN !=NULL)
	{
		delete []pTrain->m_aryTN;
		pTrain->m_aryTN =NULL;
		pTrain->m_NodeSize=0;
	}

	pTrain->m_aryTN = new STrainNode[NodeSize-2];


	int i;
	for(i = 0; i< NodeSize-2; i++)
	{
		pTrain->m_aryTN[i].NodeID = g_NodeMPIDToNodeIDMap[tmp_AryTN[NodeSize-2-i].NodeID];
		pTrain->m_aryTN[i].NodeArrivalTimestamp	= tmp_AryTN[NodeSize-2-i].NodeArrivalTimestamp;
	}

	pTrain->m_ResourceCostUsedByThisTrain=0;
	for(i = 0; i< NodeSize-2; i++)
	{
		if(i == NodeSize-3)  // destination
		{
			pTrain->m_aryTN[i].NodeDepartureTimestamp	= pTrain->m_aryTN[i].NodeArrivalTimestamp;
		}
		else
		{
			CLink* pLink = NULL;
			if (pTrain->m_Direction==1)
			{
				pLink=g_FindLinkWithNodeIDs(pTrain->m_aryTN[i].NodeID , pTrain->m_aryTN[i+1].NodeID);
			} 
			else
			{
				pLink=g_FindLinkWithNodeIDs(pTrain->m_aryTN[i+1].NodeID, pTrain->m_aryTN[i].NodeID);
			}

			ASSERT(pLink!=NULL);
			pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID;
			pTrain->TraversedLinkIDs[i] = pLink->m_LinkID;
			int LinkTravelTime = pLink->GetTrainRunningTimeInMinuteDivision (pTrain->m_SpeedMultiplier,pTrain->m_Direction);
			pTrain->m_aryTN[i].NodeDepartureTimestamp	= pTrain->m_aryTN[i+1].NodeArrivalTimestamp - LinkTravelTime ;
			pTrain->m_ResourceCostUsedByThisTrain  += GetLocalResourceCost(pLink,pTrain->m_aryTN[i].NodeArrivalTimestamp,pTrain->m_aryTN[i+1].NodeArrivalTimestamp+g_SafetyHeadway-1);
		}

	}

	//update related values for this train
	  // this is the trip price including 1) for minimum travel time: pure running time cost, stop time cost, and all resource price consumed by this train
	                                                         //2) for minimum RAS cost: cost according to RAS function, and all resource price consumed by this train
	

	pTrain->m_ActualTravelTime = pTrain->m_aryTN[NodeSize-3].NodeArrivalTimestamp - pTrain->m_EntryTime;//in minute division
	
	pTrain->m_RealizedCompletionTime = pTrain->m_aryTN[NodeSize-3].NodeArrivalTimestamp;
   
	pTrain->m_FreeRunningTime = pTrain->GetFreeRunningTimeAtTraversedLinksInMinuteDivision();//in minute division
	pTrain->m_WaitingTime=pTrain->m_ActualTravelTime-pTrain->m_FreeRunningTime;//in minute division

	TripPrice = min_cost;

	pTrain->m_TripPrice = TripPrice;
	//ensure trip price is larger than schedule cost
	double traveltimecost = pTrain->m_CostPerUnitTimeStopped*pTrain->m_WaitingTime+pTrain->m_CostPerUnitTimeRunning*pTrain->m_FreeRunningTime;


	return NodeSize-2;
}

