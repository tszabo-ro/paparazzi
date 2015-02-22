/*
 * Copyright (C) T. Szabo
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/BehaviorTrees/BTStructure.c"
 * @author T. Szabo
 * Definition for BehaviorTree nodes
 */

#include "BTStructure.h"
#include <stdio.h>


BehaviorTree theBehaviorTree;
BTWorkspace   theBehaviorTreeWorkspaceworkspace;

bool tickNode(void *node, BTNodeType nodeType, BTWorkspace *workspace)
{
  switch (nodeType)
  {
    case tBTSelect:
      {
        BTComposite *thisNode = (BTComposite*)node;
        for (int childIndex = 0; childIndex < BT_MAX_NUM_NODES; ++childIndex)
        {
          if ((thisNode->childrenTypes[childIndex] == tBTEmpty) || (thisNode->children[childIndex] == NULL))
            continue;
            
          if (tickNode(thisNode->children[childIndex], thisNode->childrenTypes[childIndex], workspace) == true)
            return true;
        }
        
        return false;
      }
      break;
    case tBTSequence:
      {
        BTComposite *thisNode = (BTComposite*)node;
        for (int childIndex = 0; childIndex < BT_MAX_NUM_NODES; ++childIndex)
        {
          if ((thisNode->childrenTypes[childIndex] == tBTEmpty) || (thisNode->children[childIndex] == NULL))
            continue;
            
          if (tickNode(thisNode->children[childIndex], thisNode->childrenTypes[childIndex], workspace) == false)
            return false;
        }
        
        return true;
      }
      break;
    case tBTConditionC:
      {
        BTConditionC *thisNode = (BTConditionC*)node;
        if (thisNode->compareTypeMoreThan)
        {
          if (workspace->wpData[thisNode->wpDataIndex] > thisNode->compareTo)
            return true;
        }
        else
        {
          if (workspace->wpData[thisNode->wpDataIndex] < thisNode->compareTo)
            return true;
        }
      }
      break;
    case tBTConditionP:
      {
        BTConditionP *thisNode = (BTConditionP*)node;
        if (thisNode->compareTypeMoreThan)
        {
          if (workspace->wpData[thisNode->wpDataIndex] > workspace->wpData[thisNode->compareTo])
            return true;
        }
        else
        {
          if (workspace->wpData[thisNode->wpDataIndex] < workspace->wpData[thisNode->compareTo])
            return true;
        }
      }
      case tBTSet:
      {
        BTSet *thisNode = (BTSet*)node;
        if (thisNode->setTypeAbsolute)
          workspace->wpData[BTWORKSPACE_NUM_INS + thisNode->wpDataIndex] = thisNode->value;
        else
          workspace->wpData[BTWORKSPACE_NUM_INS + thisNode->wpDataIndex] += thisNode->value;
          
        return true;
      }
      case tBTSetProportional:
      {
        BTSetProportional *thisNode = (BTSetProportional*)node;
        float val = thisNode->G*workspace->wpData[thisNode->wpDataIndex] + thisNode->O;
        
        if (val > thisNode->uLim)
          val = thisNode->uLim;
        if (val < thisNode->lLim)
          val = thisNode->lLim;
          
        workspace->wpData[BTWORKSPACE_NUM_INS + thisNode->wpDataTo] = val;
        
        return true;
      }
      break;
      default:
        perror("Unhandled BTNode type!");
  }
  return false;
}
bool tickBT(BehaviorTree *tree, BTWorkspace *workspace)
{
  return tickNode(tree->rootNode, tree->rootType, workspace);
}
