using System;
using System.IO;
using System.Linq;

namespace mapf;

/// <summary>
/// This class forms a wrapper around problem.GetSingleAgentOptimalCost().
/// It represents the single shortest path heuristic, precomputed for every agent.
/// </summary>
[Serializable]
class SumPairsCosts : PDB
{
    /// <summary>
    /// Since this class simply refers via a table-lookup to the globally
    /// available problem.GetSingleAgentOptimalCost class, we incur no memory.
    /// </summary>
    /// <returns>0 by definition.</returns>
    public override UInt64 estimateSize()
    {
        return 0;
    }

    /// <summary>
    /// The building function for this class doesn't do anything because we
    /// are simply wrapping the functionality of the problem.GetSingleAgentOptimalCost
    /// class.
    /// </summary>
    public override void build() {}

    /// <summary>
    /// Returns the heuristic estimate.
    /// </summary>
    /// <param name="s">The current state.</param>
    /// <returns>The PDB entry for the given state.</returns>
    public override uint h(WorldState s)
    {
        return h(s, this.problem);
    }

    public static uint h(WorldState s, ProblemInstance instance)
    {
        uint nHeuristic = 0;
        int nn = instance.GetNumOfAgents();
        //TODO:here we could check if nn is even (we consider even only)
        for(int i=0; i< nn; i = i+2) // todo for each pair state
        {
            AgentState state_agent1 = s.allAgentsState[i];
            AgentState state_agent2 = s.allAgentsState[i + 1];
            Console.WriteLine((uint)instance.GetPairsOptimalCost(i/2, state_agent1, state_agent2));
            Console.WriteLine('_');
            nHeuristic += (uint)instance.GetPairsOptimalCost(i/2, state_agent1, state_agent2); // todo get pair optimal cost
        }
        Console.WriteLine(nHeuristic);
        return nHeuristic; // if we make - nHeuristic the problem gets solved... we have heuristic problem DT
    }

    public override string ToString()
    {
        return "SIC";
    }

    /// <summary>
    /// Prints header of statistics of a single run to the given output. 
    /// </summary>
    public override void OutputStatisticsHeader(TextWriter output) { }

    /// <summary>
    /// Prints statistics of a single run to the given output.
    /// </summary>
    public override void OutputStatistics(TextWriter output) { }

    public override int NumStatsColumns
    {
        get
        {
            return 0;
        }
    }

    /// <summary>
    /// Clears statistics.
    /// </summary>
    public override void ClearStatistics() { }

    public override void ClearAccumulatedStatistics() { }
    public override void AccumulateStatistics() { }
    public override void OutputAccumulatedStatistics(TextWriter output) { }
}

/// <summary>
/// This class forms a wrapper around problem.GetSingleAgentOptimalCost().
/// It represents the single shortest path heuristic for makespan, precomputed for every agent.
/// </summary>
[Serializable]
class MaxPairsCosts : PDB
{
    /// <summary>
    /// Since this class simply refers via a table-lookup to the globally
    /// available problem.GetSingleAgentOptimalCost class, we incur no memory.
    /// </summary>
    /// <returns>0 by definition.</returns>
    public override UInt64 estimateSize()
    {
        return 0;
    }

    /// <summary>
    /// The building function for this class doesn't do anything because we
    /// are simply wrapping the functionality of the problem.GetSingleAgentOptimalCost
    /// class.
    /// </summary>
    public override void build() { }

    /// <summary>
    /// Returns the heuristic estimate.
    /// </summary>
    /// <param name="s">The current state.</param>
    /// <returns>The PDB entry for the given state.</returns>
    public override uint h(WorldState s)
    {
        return h(s, this.problem);
    }

    public static uint h(WorldState s, ProblemInstance instance)
    {
        uint maxHeuristic = 0;
        int pairAgentIndexWithMaxEstimate = 0;
        int nn = instance.GetNumOfAgents();
        for(int i=0; i< nn; i = i+2) {
            AgentState state_agent1 = s.allAgentsState[i];
            AgentState state_agent2 = s.allAgentsState[i + 1];
            uint heuristic = (uint)instance.GetPairsOptimalCost(i/2, state_agent1, state_agent2); // todo get pair optimal cost
            if (heuristic > maxHeuristic)
            {
                maxHeuristic = heuristic;
                pairAgentIndexWithMaxEstimate = i;
            }

        }


        if (s.GetType() == typeof(WorldStateWithOD))
        {
            var sWithOD = (WorldStateWithOD) s;
            if (sWithOD.agentTurn != 0 && sWithOD.agentTurn <= pairAgentIndexWithMaxEstimate)
                maxHeuristic--;  // Make the F of nodes non-decreasing. Otherwise the child node where the agent
                                    // with the max estimate finally moves, and moves along its shortest path to the
                                    // goal (decreasing the heuristic), will have a lower F than its parent (because
                                    // the cost of the node is already updated after the first agent moves). 
        }
            
        return maxHeuristic;
    }

    public override string ToString()
    {
        return "MIC";
    }

    /// <summary>
    /// Prints header of statistics of a single run to the given output. 
    /// </summary>
    public override void OutputStatisticsHeader(TextWriter output) { }

    /// <summary>
    /// Prints statistics of a single run to the given output.
    /// </summary>
    public override void OutputStatistics(TextWriter output) { }

    public override int NumStatsColumns
    {
        get
        {
            return 0;
        }
    }

    /// <summary>
    /// Clears statistics.
    /// </summary>
    public override void ClearStatistics() { }

    public override void ClearAccumulatedStatistics() { }
    public override void AccumulateStatistics() { }
    public override void OutputAccumulatedStatistics(TextWriter output) { }
}
