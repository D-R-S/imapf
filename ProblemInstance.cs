﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;

namespace mapf;

/// <summary>
/// This class represents a cooperative pathfinding problem instance. This includes:
/// - The grid in which the agents are located
/// - An array of initial state for every agent.
/// </summary>
public class ProblemInstance
{
    /// <summary>
    /// Delimiter used for export/import purposes
    /// </summary>
    private static readonly char EXPORT_DELIMITER = ',';

    /// <summary>
    /// This contains extra data of this problem instance (used for special problem instances, e.g. subproblems of a bigger problem instance).
    /// </summary>
    public IDictionary<string, object> parameters;

    /// <summary>
    /// Contains true at [x][y] if cell (x,y) is an obstacle
    /// </summary>
    public bool[][] grid;

    /// <summary>
    /// We keep a reference to the array of agents in the original problem.
    /// This will only change when IndependenceDetection's algorithm determines in another
    /// iteration that a new set of agents must be jointly planned due
    /// to their mutual conflicts.
    /// </summary>
    public AgentState[] agents;

    /// <summary>
    /// This is a matrix that contains the cost of the optimal path to the goal of every agent from any point in the grid.
    /// The first dimension of the matrix is the number of agents.
    /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
    /// </summary>
    public int[][] singleAgentOptimalCosts;


    /// <summary> //DT
    /// This is a matrix that contains the cost of the optimal path to the goal of every agent pair
    // from any point in the grid.
    /// The first dimension of the matrix is the number of agent pairs.
    /// The second dimension of the matrix is the cardinality of the location of agent 1 from which we want the shortest path (for the pairs).
    /// The third dimension of the matrix is the cardinality of the location of agent 2 from which we want the shortest path (for the pairs).

    /// </summary>
    public int[,,] pairsOptimalCosts; // DT#1 maybe change back to two dim and encode pair state with cardinalty fcn


    /// <summary>
    /// The time it took to compute the shortest paths.
    /// </summary>
    public double shortestPathComputeTime;

    /// <summary>
    /// This is a matrix that contains the best move towards the goal of every agent from any point in the grid.
    /// The first dimension of the matrix is the number of agents.
    /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
    /// </summary>
    public Move[][] singleAgentOptimalMoves;

    public uint numObstacles;
    public uint numLocations;
        
    /// <summary>
    /// This field is used to identify an instance when running a set of experiments
    /// </summary>
    public int instanceId;
        
    /// <summary>
    /// Enumerates all of the empty spots in the grid. The indices
    /// correspond directly to those used in the grid, where the major
    /// index corresponds to the x-axis and the minor index corresponds to
    /// the y-axis. If there are obstacles, it's more space-efficient to store
    /// data for each non-empty spot.
    /// </summary>
    public int[,] cardinality; // DT#1 maybe instead of having a tensor use pair state encoding for pair cardenality

    public string gridName;
    public string instanceName;

    public ProblemInstance(IDictionary<string, object> parameters = null)
    {
        if (parameters != null)
            this.parameters = parameters;
        else
            this.parameters = new Dictionary<string, object>();
    }

 
    /// <summary>
    /// Create a subproblem of this problem instance, in which only part of the agents are regarded.
    /// </summary>
    /// <param name="selectedAgents">The selected agent states that will be the root of the subproblem.</param>
    /// <returns></returns>
    public ProblemInstance Subproblem(AgentState[] selectedAgents)
    {
        // Notice selected agents may actually be a completely different set of agents.
        // Not copying instance id. This isn't the same problem.
        ProblemInstance subproblemInstance = new ProblemInstance(this.parameters);
        subproblemInstance.Init(selectedAgents, this.grid, (int)this.numObstacles, (int)this.numLocations, this.cardinality);
        subproblemInstance.singleAgentOptimalCosts = this.singleAgentOptimalCosts; // Each subproblem knows every agent's single shortest paths so this.singleAgentOptimalCosts[agent_num] would easily work
        subproblemInstance.singleAgentOptimalMoves = this.singleAgentOptimalMoves;
        subproblemInstance.gridName = this.gridName;
        subproblemInstance.instanceName = this.instanceName;
        subproblemInstance.instanceId = this.instanceId;
        return subproblemInstance;
    }

    /// <summary>
    /// Initialize the members of this object, such that the given agent states are the start state of this instance.
    /// </summary>
    /// <param name="agentStartStates"></param>
    /// <param name="grid"></param>
    /// <param name="nObstacles"></param>
    /// <param name="nLocations"></param>
    /// <param name="cardinality"></param>
    public void Init(AgentState[] agentStartStates, bool[][] grid, int nObstacles=-1,
                        int nLocations=-1, int[,] cardinality=null)
    {
        agents = agentStartStates;
        this.grid = grid;
            
        if (nObstacles == -1)
            numObstacles = (uint)grid.Sum(row => row.Count(x => x));
        else
            numObstacles = (uint)nObstacles;

        if (nLocations == -1)
            numLocations = ((uint)(grid.Length * grid[0].Length)) - numObstacles;
        else
            numLocations = (uint)nLocations;
            
        if (cardinality == null)
            PrecomputeCardinality();
        else
            this.cardinality = cardinality;
    }
        
    /// <summary>
    /// Compute the shortest path to the goal of every agent in the problem instance, from every location in the grid.
    /// Current implementation is a simple breadth-first search from every location in the graph.
    /// </summary>
    public void ComputeSingleAgentShortestPaths()
    {
        Debug.WriteLine("Computing the single agent shortest path for all agents...");
        Stopwatch watch = Stopwatch.StartNew();
        double startTime = watch.Elapsed.TotalMilliseconds;
        //return; // Add for generator

        this.singleAgentOptimalCosts = new int[this.GetNumOfAgents()][];
        this.singleAgentOptimalMoves = new Move[this.GetNumOfAgents()][];

        for (int agentId = 0; agentId < this.GetNumOfAgents(); agentId++)
        {
            // Run a single source shortest path algorithm from the _goal_ of the agent
            var shortestPathLengths = new int[this.numLocations];
            var optimalMoves = new Move[this.numLocations];
            for (int i = 0; i < numLocations; i++)
                shortestPathLengths[i] = -1;
            var openlist = new Queue<AgentState>();

            // Create initial state
            var agentStartState = this.agents[agentId];
            var agent = agentStartState.agent;
            var goalState = new AgentState(agent.Goal.x, agent.Goal.y, -1, -1, agentId);
            int goalIndex = this.GetCardinality(goalState.lastMove);
            shortestPathLengths[goalIndex] = 0;
            optimalMoves[goalIndex] = new Move(goalState.lastMove);
            openlist.Enqueue(goalState);

            while (openlist.Count > 0)
            {
                AgentState state = openlist.Dequeue();

                // Generate child states
                foreach (TimedMove aMove in state.lastMove.GetNextMoves())
                {
                    if (IsValid(aMove))
                    {
                        int entry = cardinality[aMove.x, aMove.y];
                        // If move will generate a new or better state - add it to the queue
                        if ((shortestPathLengths[entry] == -1) || (shortestPathLengths[entry] > state.g + 1))
                        {
                            var childState = new AgentState(state);
                            childState.MoveTo(aMove);
                            shortestPathLengths[entry] = childState.g;
                            optimalMoves[entry] = new Move(aMove.GetOppositeMove());
                            openlist.Enqueue(childState);
                        }
                    }
                }

            }

            int start = this.GetCardinality(agentStartState.lastMove);
            if (shortestPathLengths[start] == -1)
            {
                throw new Exception($"Unsolvable instance! Agent {agentId} cannot reach its goal");
                // Note instances can still be unsolvable if this isn't reached. E.g. this corridor:
                // s1-g2-g1-s2
            }

            this.singleAgentOptimalCosts[agentId] = shortestPathLengths;
            this.singleAgentOptimalMoves[agentId] = optimalMoves;
        }
        double endTime = watch.Elapsed.TotalMilliseconds;
        this.shortestPathComputeTime = endTime - startTime;
    }

   /// <summary> DT main function - 
    /// input: original instance
    /// a - split instances into pair instances
    /// b - Compute the shortest path to the goal of every agent pair in the problem instance,
    /// from every pair location in the grid. for all pair: A*(h1,pair-instance)
    /// TODO 
    /// output: h2 matrix for all pairs
    /// </summary>
    public void ComputePairsShortestPaths()
    {
        //Debug.WriteLine("Computing the shortest path for all agent starting locations for all pairs ...");
        Stopwatch watch = Stopwatch.StartNew();
        double startTime = watch.Elapsed.TotalMilliseconds;

        uint mm = this.numLocations;
        this.pairsOptimalCosts = new int[this.GetNumOfAgents()/2,mm,mm];
        for (int ii = 0; ii < this.GetNumOfAgents()/2; ii++)
            for (int jj = 0; jj < mm; jj++)
                for (int kk = 0; kk < mm; kk++)
                    this.pairsOptimalCosts[ii,jj,kk] = -1;




        for (int pairId = 0; pairId < this.GetNumOfAgents(); pairId = pairId + 2)
        {
            ProblemInstance pair_instance = create_pair_instance(pairId);

            for (int a1_i = 0; a1_i < grid.Length; ++a1_i) // TODO MAYBE kick out illegal start state pairs DT
                for (int a1_j = 0; a1_j < grid[a1_i].Length; ++a1_j)
                {
                    if (grid[a1_i][a1_j])
                        continue;
                
                    pair_instance.agents[0].lastMove.x = a1_i;
                    pair_instance.agents[0].lastMove.y = a1_j;
                    int locationCardinalityA1 = this.cardinality[a1_i, a1_j];
                    
                    for (int a2_i = 0; a2_i < grid.Length; ++a2_i)
                        for (int a2_j = 0; a2_j < grid[a2_i].Length; ++a2_j)
                        {
                            if (grid[a2_i][a2_j])
                                continue;

                            pair_instance.agents[1].lastMove.x = a2_i;
                            pair_instance.agents[1].lastMove.y = a2_j;
                            int locationCardinalityA2 = this.cardinality[a2_i, a2_j];
                            if (locationCardinalityA1 == locationCardinalityA2)
                                continue;
                                
                            Run runner = new Run();  // instantiates stuff unnecessarily
                            runner.startTime = runner.ElapsedMillisecondsTotal();
                            IHeuristicCalculator<WorldState> lowLevelHeuristic = new SumIndividualCosts();
                            List<uint> agentList = Enumerable.Range(0, pair_instance.agents.Length).Select(x=> (uint)x).ToList(); // FIXME: Must the heuristics really receive a list of uints?
                            lowLevelHeuristic.Init(pair_instance, agentList); // = h1(instance i) = lowLevelHeurisitc(i)
                            ISolver solver = new EPEA_Star(lowLevelHeuristic);   // solve pair instances to get h2       
                            solver.Setup(pair_instance, runner);
                            bool solved = solver.Solve();
                            
                            this.pairsOptimalCosts[pairId/2,locationCardinalityA1,locationCardinalityA2] = solver.GetSolutionCost();

                        }

                }
        }

            
    }


    public ProblemInstance create_pair_instance(int pairID){
        ProblemInstance pair_instance = ProblemInstance.Import(Directory.GetCurrentDirectory()+"/Instances/Instance-DT-10", isPair:2);
        // remove all agents that are not in the current pair
        AgentState[] current_pair_state = new AgentState[2];
        current_pair_state[0] = this.agents[pairID];
        current_pair_state[0].agent.agentNum = 0;
        current_pair_state[1] = this.agents[pairID+1];
        current_pair_state[1].agent.agentNum = 1;
        pair_instance.agents = current_pair_state;
        pair_instance.ComputeSingleAgentShortestPaths();

        return pair_instance;
    }
    /// <summary>
    /// Returns the length of the shortest path between a given coordinate and the goal location of the given agent.
    /// </summary>
    /// <param name="agentNum"></param>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <returns>The length of the shortest path from x,y to the goal of the agent.</returns>
    public int GetSingleAgentOptimalCost(int agentNum, int x, int y)
    {
        return this.singleAgentOptimalCosts[agentNum][this.cardinality[x, y]];
    }

    /// <summary>
    /// Returns the length of the shortest path between a given coordinate and the goal location of the given agent.
    /// </summary>
    /// <param name="agentNum"></param>
    /// <param name="move"></param>
    /// <returns>The length of the shortest path from x,y to the goal of the agent.</returns>
    public int GetSingleAgentOptimalCost(int agentNum, Move move)
    {
        return this.singleAgentOptimalCosts[agentNum][this.cardinality[move.x, move.y]];
    }

    /// <summary>
    /// Returns the length of the shortest path between a given agent's location and the goal of that agent.
    /// </summary>
    /// <param name="agentState"></param>
    /// <returns>The length of the shortest path between a given agent's location and the goal of that agent</returns>
    public int GetSingleAgentOptimalCost(AgentState agentState)
    {
        int locationCardinality = this.cardinality[agentState.lastMove.x, agentState.lastMove.y];
        return this.singleAgentOptimalCosts[agentState.agent.agentNum][locationCardinality];
    }

        /// <summary>
    /// Returns the length of the shortest path between a given agent's location and the goal of that agent.
    /// </summary>
    /// <param name="agentState"></param>
    /// <returns>The length of the shortest path between a given agent's location and the goal of that agent</returns>
    public int GetPairsOptimalCost(int pair,AgentState agent1State, AgentState agent2State)
    {
        int locationCardinality1 = this.cardinality[agent1State.lastMove.x, agent1State.lastMove.y];
        int locationCardinality2 = this.cardinality[agent2State.lastMove.x, agent2State.lastMove.y];
        Console.WriteLine(locationCardinality1);
        Console.WriteLine('-');
        Console.WriteLine(locationCardinality2);
        Console.WriteLine('-');
        Console.WriteLine(pair);
        Console.WriteLine('-');
        Console.WriteLine(this.pairsOptimalCosts[pair,locationCardinality1,locationCardinality2]);
        if (this.pairsOptimalCosts[pair,locationCardinality1,locationCardinality2] == -1)
            return 0;

        Console.WriteLine('-');
        Console.WriteLine(this.pairsOptimalCosts[pair,locationCardinality1,locationCardinality2]);
        return this.pairsOptimalCosts[pair,locationCardinality1,locationCardinality2];
    }

    /// <summary>
    /// Returns the optimal move towards the goal of the given agent. Move isn't necessarily unique.
    /// </summary>
    /// <param name="agentState"></param>
    /// <returns></returns>
    public Move GetSingleAgentOptimalMove(AgentState agentState)
    {
        int locationCardinality = this.cardinality[agentState.lastMove.x, agentState.lastMove.y];
        return this.singleAgentOptimalMoves[agentState.agent.agentNum][locationCardinality];
    }

    /// <summary>
    /// Note: The returned plan wasn't constructed considering a CAT, so it's possible there's an alternative plan with the same cost and less collisions.
    /// </summary>
    /// <param name="agentState"></param>
    /// <returns>An optimal plan for the agent, ignoring all others</returns>
    public SinglePlan GetSingleAgentOptimalPlan(AgentState agentState)
    {
        LinkedList<Move> moves = new LinkedList<Move>();
        int agentNum = agentState.agent.agentNum;
        TimedMove current = agentState.lastMove; // The starting position
        int time = current.time;

        while (true)
        {
            moves.AddLast(current);

            if (agentState.agent.Goal.Equals(current))
                break;

            // Get next optimal move
            time++;
            Move optimal = this.singleAgentOptimalMoves[agentNum][this.GetCardinality(current)];
            current = new TimedMove(optimal, time);
        }

        return new SinglePlan(moves, agentNum);
    }

    /// <summary>
    /// Utility function that returns the number of agents in this problem instance.
    /// </summary>
    public int GetNumOfAgents()
    {
        return this.agents.Length;
    }

    /// <summary>
    /// Utility function that returns the x dimension of the grid
    /// </summary>
    public int GetMaxX()
    {
        return this.grid.Length;
    }

    /// <summary>
    /// Utility function that returns the y dimension of the grid
    /// </summary>
    public int GetMaxY()
    {
        return this.grid[0].Length;
    }

    private static bool[][] readMapFile(string mapFilePath)
    {
        using (TextReader input = new StreamReader(mapFilePath))
        {
            // Read grid dimensions
            string line = input.ReadLine();
            if (line == "type octile")
                return readBenchmarkMap(input, line);
            else
                return readLironMap(input, line);
        }
    }

    private static bool[][] readBenchmarkMap(TextReader input, string line)
    {
        bool[][] grid;
        string[] lineParts;
        int maxX, maxY;
        // Read grid dimensions
        line = input.ReadLine();
        lineParts = line.Split(' ');
        Trace.Assert(lineParts.Length == 2);
        Trace.Assert(lineParts[0].Equals("height"));
        maxY = int.Parse(lineParts[1]);  // The height is the number of rows
        line = input.ReadLine();
        lineParts = line.Split(' ');
        Trace.Assert(lineParts.Length == 2);
        Trace.Assert(lineParts[0].Equals("width"));
        maxX = int.Parse(lineParts[1]);  // The width is the number of columns
        grid = new bool[maxY][];

        line = input.ReadLine();
        Trace.Assert(line.StartsWith("map"));

        char cell;
        // Read grid
        for (int i = 0; i < maxY; i++)
        {
            grid[i] = new bool[maxX];
            line = input.ReadLine();
            for (int j = 0; j < maxX; j++)
            {
                cell = line[j];
                if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                    grid[i][j] = true;
                else
                    grid[i][j] = false;
            }
        }
        return grid;
    }


    private static bool[][] readLironMap(TextReader input, string line) {
        string[] lineParts;
        lineParts = line.Split(',');
        int maxX = int.Parse(lineParts[0]);
        int maxY = int.Parse(lineParts[1]);
        bool[][] grid = new bool[maxX][];
        char cell;
        // Read grid
        for (int i = 0; i < maxX; i++)
        {
            grid[i] = new bool[maxY];
            line = input.ReadLine();
            for (int j = 0; j < maxY; j++)
            {
                cell = line[j];
                if (cell == '1')
                    grid[i][j] = true;
                else
                    grid[i][j] = false;
            }
        }
        return grid;
    }

    /// <summary>
    /// Imports a problem instance from a given file
    /// </summary>
    /// <param name="filePath"></param>
    /// <param name="mapFilePath"></param>
    /// <returns></returns>
    public static ProblemInstance Import(string filePath, string mapFilePath = null, int isPair = 0)
    {
        if (filePath.EndsWith(".agents"))
        {
            string fileNameWithoutExtension = Path.GetFileNameWithoutExtension(filePath);
            int instanceId = 0;
            try
            {
                instanceId = int.Parse(filePath.Split('_').Last());
            }
            catch (Exception) {}
            string mapfileNameWithoutExtension;
            if (mapFilePath == null)
            {
                var lastIndexOf_ = fileNameWithoutExtension.LastIndexOf('_');
                Trace.Assert(lastIndexOf_ >= 0);
                mapfileNameWithoutExtension = fileNameWithoutExtension.Substring(0, length: lastIndexOf_) + ".map";  // Passing a length parameter is like specifying a non-inclusive end index
                mapFilePath = Path.Combine(Path.GetDirectoryName(filePath), "..", "..", "..", "maps", mapfileNameWithoutExtension);
            }
            else
            {
                mapfileNameWithoutExtension = Path.GetFileNameWithoutExtension(mapFilePath);
            }

            bool[][] grid = readMapFile(mapFilePath);

            string line;
            string[] lineParts;
            AgentState[] states;
            using (TextReader input = new StreamReader(filePath))
            {
                // Read the number of agents
                line = input.ReadLine();
                int numOfAgents = int.Parse(line);
                if (numOfAgents > Constants.MAX_AGENTS)
                {
                    Console.WriteLine($"Only reading first {Constants.MAX_AGENTS} agents from the scenario");
                    numOfAgents = Constants.MAX_AGENTS;
                }

                // Read the agents' start and goal states
                states = new AgentState[numOfAgents];
                AgentState state;
                Agent agent;
                int goalX;
                int goalY;
                int startX;
                int startY;
                for (int i = 0; i < numOfAgents; i++)
                {
                    line = input.ReadLine();
                    lineParts = line.Split(EXPORT_DELIMITER);
                    goalX = int.Parse(lineParts[0]);
                    goalY = int.Parse(lineParts[1]);
                    startX = int.Parse(lineParts[2]);
                    startY = int.Parse(lineParts[3]);
                    agent = new Agent(goalX, goalY, i);
                    state = new AgentState(startX, startY, agent);
                    states[i] = state;
                }
            }

            // Generate the problem instance
            ProblemInstance instance = new ProblemInstance();
            instance.Init(states, grid);
            instance.instanceId = instanceId;
            instance.gridName = mapfileNameWithoutExtension;
            instance.instanceName = fileNameWithoutExtension + ".agents";
            instance.ComputeSingleAgentShortestPaths();
            return instance;
        }
        else if (filePath.EndsWith(".scen"))
        {
            string fileNameWithoutExtension = Path.GetFileNameWithoutExtension(filePath);
            int instanceId = int.Parse(fileNameWithoutExtension.Split('-').Last());
            string mapfileName;
            if (mapFilePath == null)
            {
                var lastIndexOfDash = fileNameWithoutExtension.LastIndexOf('-');
                Trace.Assert(lastIndexOfDash >= 0, "No '-' found in scenario file name?!");
                mapfileName = fileNameWithoutExtension.Substring(0, length: lastIndexOfDash);  // Passing a length parameter is like specifying a non-inclusive end index
                if (mapfileName.EndsWith(".map") == false)  // New format - remove the "-even"/"-random" and add ".map" to get the map file name
                {
                    lastIndexOfDash = mapfileName.LastIndexOf('-');
                    Trace.Assert(lastIndexOfDash >= 0);
                    mapfileName = mapfileName.Substring(0, length: lastIndexOfDash) + ".map";
                    // Handle Omri's annoying naming:
                    while (mapfileName.Contains("cross") || mapfileName.Contains("inside") || mapfileName.Contains("outside") || 
                            mapfileName.Contains("swap") || mapfileName.Contains("tight"))
                    {
                        lastIndexOfDash = mapfileName.LastIndexOf('-');
                        Trace.Assert(lastIndexOfDash >= 0);
                        mapfileName = mapfileName.Substring(0, length: lastIndexOfDash) + ".map";
                    }
                }
                mapFilePath = Path.Combine(Path.GetDirectoryName(filePath), "..", "..", "maps", mapfileName);
            }

            // DT the case where we have a instance 
            else
                mapfileName = Path.GetFileNameWithoutExtension(mapFilePath);


            bool[][] grid = readMapFile(mapFilePath);

            string line;
            string[] lineParts;
            List<AgentState> stateList = new List<AgentState>();
            using (TextReader input = new StreamReader(filePath))
            {
                // Read the format version number
                line = input.ReadLine();
                lineParts = line.Split(' ');
                Trace.Assert(lineParts[0].Equals("version"));
                int version = int.Parse(lineParts[1]);
                Trace.Assert(version == 1, "Only version 1 is currently supported");

                // Read the agents' start and goal states
                AgentState state;
                Agent agent;
                int agentNum = 0;
                int block;
                int goalX;
                int goalY;
                int startX;
                int startY;
                string mapFileNameRow;
                int mapRows;
                int mapCols;
                double optimalCost;  // Assuming diagonal moves are allowed and cost sqrt(2)
                while (true)
                {
                    line = input.ReadLine();
                    if (string.IsNullOrWhiteSpace(line))
                        break;
                    lineParts = line.Split('\t');
                    block = int.Parse(lineParts[0]);
                    mapFileNameRow = lineParts[1];
                    Trace.Assert((mapfileName == mapFileNameRow) || (mapfileName == mapFileNameRow + ".map"), "Row's map name doesn't match map's name");  // Second option is for Omri's scenarios
                    mapRows = int.Parse(lineParts[3]);
                    Trace.Assert(mapRows == grid.Length, "Row's number of grid rows doesn't match map's");
                    mapCols = int.Parse(lineParts[2]);
                    Trace.Assert(mapCols == grid[0].Length, "Row's number of grid columns doesn't match map's");

                    // Read in the start and goal coordinates.
                    // Note that at first glance, https://movingai.com/benchmarks/formats.html seems to indicate a reverse order of for Y,X,
                    // but the maps *height* is indicated as y, which translates to the number of rows here, so each coordinate is given to us
                    // as (column,row) and we invert it.
                    startY = int.Parse(lineParts[4]);
                    startX = int.Parse(lineParts[5]);
                    if (grid[startX][startY])
                        throw new Exception($"Agent {agentNum} start location ({startX},{startY}) is on an obstacle");
                    goalY = int.Parse(lineParts[6]);
                    goalX = int.Parse(lineParts[7]);
                    if (grid[goalX][goalY])
                        throw new Exception($"Agent {agentNum} goal location ({goalX},{goalY}) is on an obstacle");
                    optimalCost = double.Parse(lineParts[8]);
                    agent = new Agent(goalX, goalY, agentNum);
                    state = new AgentState(startX, startY, agent);
                    stateList.Add(state);
                    agentNum++;
                }
            }

            // Generate the problem instance
            ProblemInstance instance = new ProblemInstance();
            instance.Init(stateList.ToArray(), grid);
            instance.instanceId = instanceId;
            instance.gridName = mapfileName;
            instance.instanceName = Path.GetFileName(filePath);
            //instance.ComputeSingleAgentShortestPaths();  // FIXME: Uncomment this hack later
            return instance;
        }
        else  // Combined map and scenario, no suffix
        {
            using (TextReader input = new StreamReader(filePath))
            {
                string[] lineParts;
                string line;
                int instanceId = 0;
                string gridName = "Random Grid"; // The default

                line = input.ReadLine();
                if (line.StartsWith("Grid:") == false)
                {
                    lineParts = line.Split(',');
                    instanceId = int.Parse(lineParts[0]);
                    if (lineParts.Length > 1)
                        gridName = lineParts[1];
                    line = input.ReadLine();
                }

                // First/second line is Grid:
                Trace.Assert(line.StartsWith("Grid:"));

                // Read grid dimensions
                line = input.ReadLine();
                lineParts = line.Split(',');
                int maxX = int.Parse(lineParts[0]);
                int maxY = int.Parse(lineParts[1]);
                bool[][] grid = new bool[maxX][];
                // Read grid
                char cell;
                for (int i = 0; i < maxX; i++)
                {
                    grid[i] = new bool[maxY];
                    line = input.ReadLine();
                    for (int j = 0; j < maxY; j++)
                    {
                        cell = line[j];
                        if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                            grid[i][j] = true;
                        else
                            grid[i][j] = false;
                    }
                }

                // Next line is Agents:
                line = input.ReadLine();
                Trace.Assert(line.StartsWith("Agents:"));

                // Read the number of agents
                line = input.ReadLine();
                int numOfAgents = int.Parse(line);

                // Read the agents' start and goal states
                AgentState[] states = new AgentState[numOfAgents];
                AgentState state;
                Agent agent;
                int agentNum;
                int goalX;
                int goalY;
                int startX;
                int startY;
                for (int i = 0; i < numOfAgents; i++)
                {
                    line = input.ReadLine();
                    lineParts = line.Split(EXPORT_DELIMITER);
                    agentNum = int.Parse(lineParts[0]);
                    goalX = int.Parse(lineParts[1]);
                    goalY = int.Parse(lineParts[2]);
                    startX = int.Parse(lineParts[3]);
                    startY = int.Parse(lineParts[4]);
                    agent = new Agent(goalX, goalY, agentNum);
                    state = new AgentState(startX, startY, agent);
                    states[i] = state;
                }

                // Generate the problem instance 
                ProblemInstance instance = new ProblemInstance();
                instance.Init(states, grid);
                instance.instanceId = instanceId;
                instance.gridName = gridName;
                instance.instanceName = Path.GetFileNameWithoutExtension(filePath);
                if (isPair == 1){ // DT
                    instance.ComputePairsShortestPaths();
                    instance.ComputeSingleAgentShortestPaths(); 
                    // for (int i = 0; i < instance.GetNumOfAgents(); i = i+2){ // DT dumb sanity check
                    //     for(int c1=0; c1 < 4; c1++){
                    //         for(int c2=0; c2 < ; c2++){

                    //             int h1 = instance.singleAgentOptimalCosts[i][c1] + instance.singleAgentOptimalCosts[i+1][c2];
                    //             int h2 = instance.pairsOptimalCosts[i/2, c1, c2];
                    //             if(h1 - h2 < 0){
                    //                 Console.WriteLine("h2 is worse than h1 - as it should be!");
                    //             }
                                


                    //         }
                    //     }

                    // }
                        

                }
                else if (isPair == 0)
                    instance.ComputeSingleAgentShortestPaths();
                
                return instance;
            }
        }
    }

    /// <summary>
    /// Exports a problem instance to a file. The format depends on the suffix of the given filename.
    /// </summary>
    /// <param name="fileName"></param>
    /// <param name="mapFileName">For including in .scen file format data</param>
    public void Export(string fileName, string mapFileName = null)
    {
        TextWriter output = new StreamWriter(Path.Combine(Directory.GetCurrentDirectory(), "Instances", fileName));

        if (fileName.EndsWith(".scen"))
        {
            output.WriteLine("version 1");

            if (mapFileName == null)
                throw new Exception("Map file name needed for .scen format");

            foreach (var agentState in this.agents)
            {
                // Output all agent as block 1, with optimal cost -1
                output.WriteLine($"{1}\t{mapFileName}\t{grid[0].Length}\t{grid.Length}\t{agentState.lastMove.y}\t{agentState.lastMove.x}\t{agentState.agent.Goal.y}\t{agentState.agent.Goal.x}\t{-1}");
            }
        }
        else if (fileName.EndsWith(".agents"))
        {
            output.WriteLine(this.GetNumOfAgents());

            foreach (var agentState in this.agents)
            {
                output.WriteLine($"{agentState.agent.Goal.x},{agentState.agent.Goal.y},{agentState.lastMove.x},{agentState.lastMove.x}");
            }
        }
        else
        {
            // Output the instance ID
            output.WriteLine($"{this.instanceId},{this.gridName}");

            // Output the grid
            output.WriteLine("Grid:");
            output.WriteLine($"{this.grid.Length},{this.grid[0].Length}");

            for (int i = 0; i < this.grid.Length; i++)
            {
                for (int j = 0; j < this.grid[0].Length; j++)
                {
                    if (this.grid[i][j] == true)
                        output.Write('@');
                    else
                        output.Write('.');

                }
                output.WriteLine();
            }
            // Output the agents state
            output.WriteLine("Agents:");
            output.WriteLine(this.agents.Length);
            AgentState state;
            for (int i = 0; i < this.agents.Length; i++)
            {
                state = this.agents[i];
                output.WriteLine($"{state.agent.agentNum}{EXPORT_DELIMITER}{state.agent.Goal.x}{EXPORT_DELIMITER}{state.agent.Goal.y}{EXPORT_DELIMITER}{state.lastMove.x}{EXPORT_DELIMITER}{state.lastMove.y}");
            }
        }
        output.Flush();
        output.Close();
    }

    /// <summary>
    /// Given an agent located at the nth location on our board that is
    /// not occupied by an obstacle, we return n.
    /// </summary>
    /// <param name="move">An agent's current location.</param>
    /// <returns>n, where the agent is located at the nth non-obstacle
    /// location in our grid.</returns>
    public int GetCardinality(Move move)
    {
        return cardinality[move.x, move.y];
    }
        
    private void PrecomputeCardinality()
    {
        cardinality = new int[grid.Length, grid[0].Length];
        int maxCardinality = 0;
        for (uint i = 0; i < grid.Length; ++i)
            for (uint j = 0; j < grid[i].Length; ++j)
            {
                if (grid[i][j])
                    cardinality[i, j] = -1;
                else
                    cardinality[i, j] = maxCardinality++;
            }
    }

    /// <summary>
    /// Check if the tile is valid, i.e. in the grid and without an obstacle.
    /// NOT checking the direction. A Move could be declared valid even if it came to an edge tile from outside the grid!
    /// </summary>
    /// <param name="aMove"></param>
    /// <returns>True if the given location is a valid grid location with no obstacles</returns>
    public bool IsValid(Move aMove)
    {
        return IsValidTile(aMove.x, aMove.y);
    }

    /// <summary>
    /// Also checks if the move is illegal
    /// </summary>
    /// <param name="toCheck"></param>
    /// <returns></returns>
    public bool IsValid(TimedMove toCheck)
    {
        if (IsValidTile(toCheck.x, toCheck.y) == false)
            return false;

        return true;
    }

    public bool IsValidTile(int x, int y)
    {
        if (x < 0 || x >= GetMaxX())
            return false;
        if (y < 0 || y >= GetMaxY())
            return false;
        return !grid[x][y];
    }

    public override string ToString()
    {
        return $"Problem instance name:{instanceName} #Agents:{agents.Length}, GridCells:{numLocations}, #Obstacles:{numObstacles}";
    }
}
