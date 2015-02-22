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
 * @file "modules/BehaviorTrees/BTStructure.h"
 * @author T. Szabo
 * Definition for BehaviorTree nodes
 */

#ifndef BTSTRUCTURE_H
#define BTSTRUCTURE_H

#include "inttypes.h"
#include <stdbool.h>
#include "math.h"

#define BTWORKSPACE_NUM_INS 1
#define BTWORKSPACE_NUM_PAR 0
#define BTWORKSPACE_NUM_OUT 2
#define BT_MAX_NUM_NODES    5

typedef enum {tBTSelect, tBTSequence, tBTConditionC, tBTConditionP, tBTSet, tBTSetProportional, tBTEmpty} BTNodeType;

typedef struct BehaviorTreeStrtuct
{
  void        *rootNode;
  BTNodeType  rootType;
  
} BehaviorTree;

typedef struct BTWorkspaceStruct
{
  float wpData[BTWORKSPACE_NUM_INS + BTWORKSPACE_NUM_PAR + BTWORKSPACE_NUM_OUT];
} BTWorkspace;

typedef struct BTCompositeStruct
{
  void        *children[BT_MAX_NUM_NODES];
  BTNodeType  childrenTypes[BT_MAX_NUM_NODES];
} BTComposite;
typedef struct BTConditionCStruct
{
  uint8_t     wpDataIndex;
  float       compareTo;
  bool        compareTypeMoreThan;
} BTConditionC;
typedef struct BTConditionPStruct
{
  uint8_t     wpDataIndex;
  uint8_t     compareTo;
  bool        compareTypeMoreThan;
} BTConditionP;
typedef struct BTSetStruct
{
  uint8_t     wpDataIndex;
  float       value;
  bool        setTypeAbsolute;
} BTSet;

typedef struct BTSetProportionalStruct
{
    int       wpDataIndex;
    int       wpDataTo;
    float     G;
    float     O;
    
    float     lLim;
    float     uLim;
    
} BTSetProportional;

extern BehaviorTree theBehaviorTree;
extern BTWorkspace   theBehaviorTreeWorkspace;

extern void initBT(BehaviorTree *BT);
extern bool tickBT(BehaviorTree *tree, BTWorkspace *workspace);
extern bool tickNode(void *node, BTNodeType nodeType, BTWorkspace *workspace);
#endif
