using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace mapf;

class WorldStateForPartialExpansion : WorldState
{
    public bool alreadyExpanded;
    /// <summary>
    /// Starts at zero, incremented after a node is expanded once. Set on Expand.
    /// </summary>
    public ushort targetDeltaF;
    /// <summary>
    /// Remaining delta F towards targetDeltaF. Reset on Expand.
    /// </summary>
    public ushort remainingDeltaF;
    /// <summary>
    /// For each agent and each direction it can go, the effect of that move on F
    /// byte.MaxValue means this is an illegal move. Only computed on demand.
    /// </summary>
    protected byte[][] singleAgentDeltaFs;
    /// <summary>
    /// For each pair and each direction it can go, the effect of that move on F
    /// byte.MaxValue means this is an illegal move. Only computed on demand.
    /// </summary>
    protected byte[][,] pairsDeltaFs;
    /// <summary>
    /// Only computed on demand
    /// </summary>
    protected ushort maxDeltaF;
    public enum DeltaFAchievable : sbyte
    {
        YES = 1,
        NO = -1,
        NOT_YET_COMPUTED = 0
    }
    /// <summary>
    /// Per each agent and delta F, has 1 if that delta F is achievable by moving the agents starting from this one on,
    /// -1 if it isn't, and 0 if we don't know yet.
    /// Only computed on demand
    /// </summary>
    protected DeltaFAchievable[][] fLookup;

    /// <summary>
    /// Per each pair and delta F, has 1 if that delta F is achievable by moving the agents starting from this one on,
    /// -1 if it isn't, and 0 if we don't know yet.
    /// Only computed on demand
    /// </summary>
    protected DeltaFAchievable[][] pairsFLookup;

    /// <summary>
    /// The node's SIC heuristic estimate
    /// </summary>
    public int sic;

    /// <summary>
    /// Create a state with the given state for every agent.
    /// </summary>
    /// <param name="allAgentsState"></param>
    /// <param name="minDepth"></param>
    /// <param name="minCost"></param>
    public WorldStateForPartialExpansion(AgentState[] allAgentState, int minDepth = -1,
                                            int minCost = -1, MDDNode mddNode = null):
        base(allAgentState, minDepth, minCost, mddNode)
    {
        this.alreadyExpanded = false;
        this.maxDeltaF = 0;
        this.singleAgentDeltaFs = null;
        this.pairsDeltaFs = null;
        this.fLookup = null;
        this.pairsFLookup = null;
    }

    /// <summary>
    /// Copy constructor
    /// </summary>
    /// <param name="cpy"></param>
    public WorldStateForPartialExpansion(WorldStateForPartialExpansion cpy)
        : base(cpy)
    {
        alreadyExpanded = false; // Creating a new unexpanded node from cpy

        // For intermediate nodes created during expansion (fully expanded nodes have these fields recalculated when they're expanded)
        remainingDeltaF = cpy.remainingDeltaF;
        singleAgentDeltaFs = cpy.singleAgentDeltaFs; // For the UpdateRemainingDeltaF call on temporary nodes.
                                                        // Notice that after an agent is moved its row won't be up-to-date.
        pairsDeltaFs = cpy.pairsDeltaFs;
        fLookup = cpy.fLookup; // For the hasChildrenForCurrentDeltaF call on temporary nodes.
                                // Notice that after an agent is moved, all rows up to and including the one of the agent that moved
                                // won't be up-to-date.
        pairsFLookup = cpy.pairsFLookup;
        maxDeltaF = cpy.maxDeltaF; // Not necessarily achievable after some of the agents moved.
        // The above is OK because we won't be using data for agents that already moved.
    }

    /// <summary>
    /// From generated nodes. Allows expansion table to be garbage collected before all generated nodes are expanded.
    /// </summary>
    public void ClearExpansionData()
    {
        this.singleAgentDeltaFs = null;
        this.fLookup = null;
    }

    public override string ToString()
    {
        return $"{base.ToString()} (sic:{this.sic} target deltaF: {this.targetDeltaF})";
    }

    public delegate bool ValidityChecker(TimedMove move, IReadOnlyDictionary<TimedMove, int> currentMoves, int makespan, int agentIndex, WorldState node, WorldState intermediateNode);

    private static readonly IReadOnlyDictionary<TimedMove, int> noMoves = new Dictionary<TimedMove, int>();

    /// <summary>
    /// Calculates for each agent and each direction it can go, the effect of that move on F. Illegal moves get byte.MaxValue.
    /// Also calcs maxDeltaF.
    /// Implicitly uses the SIC heuristic.
    /// </summary>
    /// <param name="problem">For GetSingleAgentOptimalCost</param>
    /// <param name="isValid"></param>
    /// <returns></returns>
     public void calcSingleAgentDeltaFs(ProblemInstance problem, ValidityChecker isValid)
    {
        // Init
        this.singleAgentDeltaFs = new byte[allAgentsState.Length][];
        for (int i = 0; i < singleAgentDeltaFs.Length; i++)
        {
            this.singleAgentDeltaFs[i] = new byte[Constants.NUM_ALLOWED_DIRECTIONS];
        }

        int hBefore, hAfter;

        this.maxDeltaF = 0;

        // Set values
        for (int i = 0; i < allAgentsState.Length; i++)
        {
            hBefore = problem.GetSingleAgentOptimalCost(allAgentsState[i]);
                
            int singleAgentMaxLegalDeltaF = -1;

            foreach (TimedMove check in allAgentsState[i].lastMove.GetNextMoves())
            {
                if (isValid(check, noMoves, this.makespan + 1, i, this, this) == false)  // Is this move by itself invalid because of constraints or obstacles
                {
                        singleAgentDeltaFs[i][(int)check.direction] = byte.MaxValue;
                }
                else
                {
                    hAfter = problem.GetSingleAgentOptimalCost(allAgentsState[i].agent.agentNum, check);

                    if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
                    {
                        if (hBefore != 0)
                            singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                        else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
                            singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                        else
                            singleAgentDeltaFs[i][(int)check.direction] = 0; // This is a WAIT move at the goal.
                    }
                    else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
                    {
                        if (hBefore == 0 && hAfter == 0)
                            singleAgentDeltaFs[i][(int)check.direction] = 0; // This is a WAIT move at the goal.
                        else
                            singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                    }
                    singleAgentMaxLegalDeltaF = Math.Max(singleAgentMaxLegalDeltaF, singleAgentDeltaFs[i][(int)check.direction]);
                }
            }

            if (singleAgentMaxLegalDeltaF == -1) // No legal action for this agent, so no legal children exist for this node
            {
                this.maxDeltaF = 0; // Can't make it negative without widening the field.
                break;
            }

            this.maxDeltaF += (byte) singleAgentMaxLegalDeltaF;
        }

        fLookup = new DeltaFAchievable[allAgentsState.Length][];
        for (int i = 0; i < fLookup.Length; i++)
        {
            fLookup[i] = new DeltaFAchievable[this.maxDeltaF + 1]; // Towards the last agents most of the row will be wasted (the last one can do delta F of 0 or 1),
                                                                    // but it's easier than fiddling with array sizes
        }
    }





    /// <summary>
    /// Calculates for each pair and each direction it can go, the effect of that move on F. Illegal moves get byte.MaxValue.
    /// Also calcs maxDeltaF.
    /// Implicitly uses the SIC heuristic.
    /// </summary>
    /// <param name="problem">For GetPairsOptimalCost</param>
    /// <param name="isValid"></param>
    /// <returns></returns>
    public void calcPairsDeltaFs(ProblemInstance problem, ValidityChecker isValid)
    {
        // Init
        this.pairsDeltaFs = new byte[(int)Math.Ceiling((double)allAgentsState.Length/2)][,];

        int threshold = allAgentsState.Length % 2 == 0 ? allAgentsState.Length : allAgentsState.Length - 2; 

        for (int i = 0; i < (int)Math.Ceiling((double)allAgentsState.Length/2) ; i++)
        {
            this.pairsDeltaFs[i] = new byte[Constants.NUM_ALLOWED_DIRECTIONS, Constants.NUM_ALLOWED_DIRECTIONS];
        }

        int hBefore = 0, hAfter = 0;

        this.maxDeltaF = 0;

        // Set values
        for (int pairId = 0; pairId < threshold; pairId = pairId + 2)
        {
            hBefore = problem.GetPairsOptimalCost(pairId / 2, allAgentsState[pairId], allAgentsState[pairId+1]);
                
            int pairsMaxLegalDeltaF = -1;

            foreach (TimedMove check1 in allAgentsState[pairId].lastMove.GetNextMoves())
            {
                foreach (TimedMove check2 in allAgentsState[pairId + 1].lastMove.GetNextMoves())
            {
                if (isValid(check1, noMoves, this.makespan + 1, pairId, this, this) == false || isValid(check2, noMoves, this.makespan + 1, pairId + 1, this, this) == false)  // Is this move by itself invalid because of constraints or obstacles
                {
                        pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction] = byte.MaxValue;
                }
                else
                {
                    hAfter = problem.GetPairsOptimalCost(pairId / 2, check1, check2);

                    if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
                    {
                        if (hBefore != 0)
                            pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                        else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference  
                        // TODO:do we need to substruct both of the agents arraival time? do we need to change the if? DT
                            pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[pairId].arrivalTime - allAgentsState[pairId + 1].arrivalTime + 1); //DT NOT SURE!
                        else
                            pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction] = 0; // This is a WAIT move at the goal.
                    }
                    else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
                    {
                        if (hBefore == 0 && hAfter == 0)
                            pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction] = 0; // This is a WAIT move at the goal.
                        else
                            pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                    }
                    pairsMaxLegalDeltaF = Math.Max(pairsMaxLegalDeltaF, pairsDeltaFs[pairId/2][(int)check1.direction, (int)check2.direction]);
                }
            }
            }

            if (pairsMaxLegalDeltaF == -1) // No legal action for this agent, so no legal children exist for this node
            {
                this.maxDeltaF = 0; // Can't make it negative without widening the field.
                break;
            }

            this.maxDeltaF += (byte) pairsMaxLegalDeltaF;
        }

        if (allAgentsState.Length % 2 != 0){ // DT last agent in uneven case
            int agentNum = allAgentsState.Length - 1;
            int i = allAgentsState.Length / 2;
            hBefore = problem.GetSingleAgentOptimalCost(allAgentsState[agentNum]);
                
            int singleAgentMaxLegalDeltaF = -1;

            foreach (TimedMove check in allAgentsState[agentNum].lastMove.GetNextMoves())
            {
                if (isValid(check, noMoves, this.makespan + 1, agentNum, this, this) == false)  // Is this move by itself invalid because of constraints or obstacles
                {
                        pairsDeltaFs[i][(int)check.direction, 0] = byte.MaxValue;
                }
                else
                {
                    hAfter = problem.GetSingleAgentOptimalCost(allAgentsState[agentNum].agent.agentNum, check);

                    if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
                    {
                        if (hBefore != 0)
                            pairsDeltaFs[i][(int)check.direction, 0] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                        else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
                            pairsDeltaFs[i][(int)check.direction, 0] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                        else
                            pairsDeltaFs[i][(int)check.direction, 0] = 0; // This is a WAIT move at the goal.
                    }
                    else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
                    {
                        if (hBefore == 0 && hAfter == 0)
                            pairsDeltaFs[i][(int)check.direction, 0] = 0; // This is a WAIT move at the goal.
                        else
                            pairsDeltaFs[i][(int)check.direction, 0] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                    }
                    singleAgentMaxLegalDeltaF = Math.Max(singleAgentMaxLegalDeltaF, pairsDeltaFs[i][(int)check.direction, 0]);
                }
            }

            if (singleAgentMaxLegalDeltaF == -1) // No legal action for this agent, so no legal children exist for this node
            {
                this.maxDeltaF = 0; // Can't make it negative without widening the field.
            
            }
            else
                this.maxDeltaF += (byte) singleAgentMaxLegalDeltaF;
        }

        pairsFLookup = new DeltaFAchievable[(int)Math.Ceiling((double)allAgentsState.Length/2)][];
        for (int i = 0; i < pairsFLookup.Length; i++)
        {
            pairsFLookup[i] = new DeltaFAchievable[this.maxDeltaF + 1]; // Towards the last agents most of the row will be wasted (the last one can do delta F of 0 or 1),
                                                                    // but it's easier than fiddling with array sizes
        }
    }

    /// <summary>
    /// Returns whether all possible f values were generated from this node already.
    /// Assumes calcSingleAgentDeltaFs was called earlier.
    /// </summary>
    /// <returns></returns>
    public bool hasMoreChildren()
    {
        return this.targetDeltaF <= this.maxDeltaF;
    }
        
    public bool IsAlreadyExpanded()
    {
        return alreadyExpanded;
    }

    /// <summary>
    /// Assumes calcSingleAgentDeltaFs was called earlier.
    /// </summary>
    /// <param name="id"></param> agenNum if isPair=false, pairId otherwise
    /// <returns></returns>
    public bool hasChildrenForCurrentDeltaF(bool isPair, int id=0)
    {
        return isPair ? pairsExistsChildForF(id, this.remainingDeltaF) : existsChildForF(id, this.remainingDeltaF);
    }

    /// <summary>
    /// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
    /// </summary>
    /// <param name="agentNum"></param>
    /// <param name="remainingTargetDeltaF"></param>
    /// <returns></returns>
    protected bool existsChildForF(int agentNum, ushort remainingTargetDeltaF)
    {
        // Stopping conditions:
        if (agentNum == allAgentsState.Length)
        {
            if (remainingTargetDeltaF == 0)
                return true;
            return false;
        }
            
        if (fLookup[agentNum][remainingTargetDeltaF] != DeltaFAchievable.NOT_YET_COMPUTED) // Answer known (arrays are initialized to zero).
        {
            return fLookup[agentNum][remainingTargetDeltaF] == DeltaFAchievable.YES; // Return known answer.
        }

        // Recursive actions:
        for (int direction = 0; direction < Constants.NUM_ALLOWED_DIRECTIONS; direction++)
        {
            if (singleAgentDeltaFs[agentNum][direction] > remainingTargetDeltaF) // Small optimization - no need to make the recursive
                                                                                    // call just to request a negative target from it and
                                                                                    // get false (because we assume the heuristic function
                                                                                    // is consistent)
                continue;
            if (existsChildForF(agentNum + 1, (byte)(remainingTargetDeltaF - singleAgentDeltaFs[agentNum][direction])))
            {
                fLookup[agentNum][remainingTargetDeltaF] = DeltaFAchievable.YES;
                return true;
            }
        }
        fLookup[agentNum][remainingTargetDeltaF] = DeltaFAchievable.NO;
        return false;
    }


     /// <summary>
    /// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
    /// </summary>
    /// <param name="pairId"></param>
    /// <param name="remainingTargetDeltaF"></param>
    /// <returns></returns>
    protected bool pairsExistsChildForF(int pairId, ushort remainingTargetDeltaF)
    {
        // Stopping conditions:
        int stopping_cond = allAgentsState.Length % 2 == 0 ? allAgentsState.Length - 1 : allAgentsState.Length - 2;
        if (pairId >= stopping_cond)
        {
            if(allAgentsState.Length % 2 != 0 & pairId == allAgentsState.Length - 1)
                return pairsLastAgentExistsChildForF(allAgentsState.Length - 1, remainingTargetDeltaF);
            if (remainingTargetDeltaF == 0)
                return true;
            return false;
        }
            
        if (pairsFLookup[pairId / 2][remainingTargetDeltaF] != DeltaFAchievable.NOT_YET_COMPUTED) // Answer known (arrays are initialized to zero).
        {
            return pairsFLookup[pairId / 2][remainingTargetDeltaF] == DeltaFAchievable.YES; // Return known answer.
        }

        // Recursive actions:
        for (int direction1 = 0; direction1 < Constants.NUM_ALLOWED_DIRECTIONS; direction1++)
        {
            for (int direction2 = 0; direction2 < Constants.NUM_ALLOWED_DIRECTIONS; direction2++)
        {
            if (pairsDeltaFs[pairId / 2][direction1, direction2] > remainingTargetDeltaF) // Small optimization - no need to make the recursive
                                                                                    // call just to request a negative target from it and
                                                                                    // get false (because we assume the heuristic function
                                                                                    // is consistent)
                continue;
            if (pairsExistsChildForF(pairId + 2, (byte)(remainingTargetDeltaF - pairsDeltaFs[pairId / 2][direction1, direction2])))
            {
                pairsFLookup[pairId / 2][remainingTargetDeltaF] = DeltaFAchievable.YES;
                return true;
            }
        }
        }
        pairsFLookup[pairId / 2][remainingTargetDeltaF] = DeltaFAchievable.NO;
        return false;
    }

    /// DT 
    protected bool pairsLastAgentExistsChildForF(int agenNum, ushort remainingTargetDeltaF){
        if (pairsFLookup[agenNum / 2][remainingTargetDeltaF] != DeltaFAchievable.NOT_YET_COMPUTED) // Answer known (arrays are initialized to zero).
        {
            return pairsFLookup[agenNum / 2][remainingTargetDeltaF] == DeltaFAchievable.YES; // Return known answer.
        }

        // Recursive actions:
        for (int direction = 0; direction < Constants.NUM_ALLOWED_DIRECTIONS; direction++)
        {               
            if (pairsDeltaFs[agenNum / 2][direction, 0] > remainingTargetDeltaF) // Small optimization - no need to make the recursive
                                                                                    // call just to request a negative target from it and
                                                                                    // get false (because we assume the heuristic function
                                                                                    // is consistent)
                continue;
            if (pairsExistsChildForF(agenNum + 1, (byte)(remainingTargetDeltaF - pairsDeltaFs[agenNum / 2][direction, 0])))
            {
                pairsFLookup[agenNum / 2][remainingTargetDeltaF] = DeltaFAchievable.YES;
                return true;
            }
        
        }
        pairsFLookup[agenNum / 2][remainingTargetDeltaF] = DeltaFAchievable.NO;
        return false;

    }

    /// <summary>
    /// An agent was moved between calculating the singleAgentDeltaFs and this call.
    /// Using the data that describes its delta F potential before the move.
    /// </summary>
    /// <param name="agentNum"></param>
    public void UpdateRemainingDeltaF(bool isPair, int agentNum) {
        if (this.remainingDeltaF == ushort.MaxValue)
            Trace.Assert(false,
                            $"Remaining deltaF is ushort.MaxValue, a reserved value with special meaning. agentNum={agentNum}");

        byte lastMoveDeltaF = 0;
        if(isPair){//DT
            if(allAgentsState.Length % 2 != 0 & agentNum == allAgentsState.Length - 1)  // the last agent when no even agents 
                lastMoveDeltaF = this.pairsDeltaFs[agentNum / 2][(int)this.allAgentsState[agentNum].lastMove.direction, 0];
            else if(agentNum % 2 != 0) //only after we have the step for both agents in the pair
                lastMoveDeltaF = this.pairsDeltaFs[agentNum / 2][(int)this.allAgentsState[agentNum -1].lastMove.direction, (int)this.allAgentsState[agentNum].lastMove.direction];
            
        }
        else 
        lastMoveDeltaF = this.singleAgentDeltaFs[agentNum][(int)this.allAgentsState[agentNum].lastMove.direction];

        if (lastMoveDeltaF != byte.MaxValue && this.remainingDeltaF >= lastMoveDeltaF)
            this.remainingDeltaF -= lastMoveDeltaF;
        else
            this.remainingDeltaF = ushort.MaxValue; // Either because last move was illegal or because the delta F from the last move was more than the entire remaining delta F budget
    }

    /// <summary>
    /// For fully expanded nodes.
    /// Notice ClearExpansionData does a similar thing, but for different reasons.
    /// </summary>
    public override void Clear()
    {
        this.alreadyExpanded = false; // Enables reopening
        // The following info could be reused when reopening the node, saving the time it takes to generate it,
        // but reopening a node is rare in our domain, and the memory saved can be significant
        this.fLookup = null; // Save a lot of memory
        this.singleAgentDeltaFs = null; // Save some more memory
        this.targetDeltaF = 0;
        this.remainingDeltaF = 0;
        //this.h -= this.hBonus; // Reset the h
        //this.hBonus = 0;
    }

    public override int f
    {
        get
        {
            return Math.Max(this.g + this.h,
                            this.g + this.sic + this.targetDeltaF);
        }
    }
}
