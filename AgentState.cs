﻿using System;
using System.Collections.Generic;

namespace CPF_experiment
{
    [Serializable] public class AgentState : IComparable<IBinaryHeapItem>, IBinaryHeapItem
    {
        public int pos_X;
        public int pos_Y;
        public int h;
        public Agent agent;
        public int arrivalTime;
        public int currentStep;
        public sbyte direction;
        private int binaryHeapIndex;
        public int potentialConflicts;
        public ushort potentialConflictsID;
        [NonSerialized] public AgentState prev;

        public AgentState(int pos_X, int pos_Y, Agent agent)
        {
            this.pos_X = pos_X;
            this.pos_Y = pos_Y;
            direction = 0;
            this.agent = agent;
        }

        public AgentState(int startX, int startY, int goalX, int goalY, int agentId)
            : this(startX, startY, new Agent(goalX, goalY, agentId))
        {}

        public AgentState(AgentState copy)
        {
            this.pos_X = copy.pos_X;
            this.pos_Y = copy.pos_Y;
            this.agent = copy.agent;
            this.h = copy.h;
            this.arrivalTime = copy.arrivalTime;
            this.direction = copy.direction;
            this.currentStep = copy.currentStep;
        }

        public void swapCurrentWithGoal()
        {
            int nTemp = pos_X;
            pos_X = agent.Goal_X;
            agent.Goal_X = nTemp;
            nTemp = pos_Y;
            pos_Y = agent.Goal_Y;
            agent.Goal_Y = nTemp;
        }

        /// <summary>
        /// Actually moves the agent and recalculates its heuristic.
        /// Does NOT recalculates the heuristic!
        /// </summary>
        public void move(int direction)
        {
            int deltaX = WorldState.operators[direction, 0];
            int deltaY = WorldState.operators[direction, 1];
            pos_X += deltaX;
            pos_Y += deltaY;
            currentStep += 1;
            this.direction = (sbyte)direction;

            // If performed a non STAY move and reached the agent's goal - store the arrival time
            if (((deltaX != 0) || (deltaY != 0)) && (this.atGoal()))
                this.arrivalTime = currentStep;
        }

        /// <summary>
        /// Checks if the agent is at its goal location
        /// </summary>
        /// <returns>True if the agent has reached its goal location</returns>
        public bool atGoal()
        {
            return ((this.pos_X == this.agent.Goal_X) && (this.pos_Y == this.agent.Goal_Y));
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        public int getIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        public void setIndexInHeap(int index) { binaryHeapIndex = index; }

        /// <summary>
        /// Checks pos_X, pos_Y, agent and, if CBS_LocalConflicts.isDnC, the currentStep too.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            AgentState that = (AgentState)obj;
            
            if (CBS_LocalConflicts.isDnC == true)
                if (this.currentStep != that.currentStep)
                    return false;

            if (this.pos_X == that.pos_X && this.pos_Y == that.pos_Y && this.agent.Equals(that.agent))
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Uses pos_X, pos_Y and agent.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                return this.agent.GetHashCode() + (this.pos_X * 5) + (this.pos_Y * 7);
            }
        }

        public LinkedList<Move> GetMove()
        {
            LinkedList<Move> ans = new LinkedList<Move>();
            ans.AddFirst(new Move(pos_X, pos_Y, direction));
            return ans;
        }

        public int CompareTo(IBinaryHeapItem other)
        {
            AgentState that = (AgentState)other;
            if (this.h + this.currentStep < that.h + that.currentStep)
                return -1;
            if (this.h + this.currentStep > that.h + that.currentStep)
                return 1;



            if (this.potentialConflictsID < that.potentialConflictsID)
                return -1;
            if (this.potentialConflictsID > that.potentialConflictsID)
                return 1;


            if (this.potentialConflicts < that.potentialConflicts)
                return -1;
            if (this.potentialConflicts > that.potentialConflicts)
                return 1;



            if (this.currentStep < that.currentStep)
                return 1;
            if (this.currentStep > that.currentStep)
                return -1;
            return 0;
        }

        public override string ToString()
        {
            return " step-" + currentStep + " position (" + pos_X + "," + pos_Y + ")";
        }
    }

    ///<summary>
    /// Compares two AgentStates according to their AgentNum
    /// This method is used for Rtrevor
    ///</summary>
    class AgentsNumsComparator : IComparer<AgentState>
    {
        public int Compare(AgentState x, AgentState y)
        {
            if (x.agent.agentNum > y.agent.agentNum)
                return 1;
            else if (x.agent.agentNum < y.agent.agentNum)
                return -1;
            else
                return 0;
        }
    }
}
