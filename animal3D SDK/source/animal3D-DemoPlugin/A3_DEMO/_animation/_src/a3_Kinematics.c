/*
	Copyright 2011-2025 Daniel S. Buckstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
	animal3D SDK: Minimal 3D Animation Framework
	By Daniel S. Buckstein
	
	a3_Kinematics.c
	Implementation of kinematics solvers.
*/

#include "../a3_Kinematics.h"


//-----------------------------------------------------------------------------

// single FK helpers
static inline void a3kinematicsSolveForwardSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// T[this_object] = T[parent_object] * T[this_local]
	a3real4x4Product(
		hierarchyState->objectSpace->hpose_base[index].transformMat.m,		// Result: this node object-space.
		hierarchyState->objectSpace->hpose_base[parentIndex].transformMat.m,// Left-hand: parent node object-space.
		hierarchyState->localSpace->hpose_base[index].transformMat.m		// Right-hand: this node local space.
	);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
}
static inline void a3kinematicsSolveForwardRoot(const a3_HierarchyState* hierarchyState, const a3ui32 index)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// T[root_object] = T[root_local]
	hierarchyState->objectSpace->hpose_base[index] = hierarchyState->localSpace->hpose_base[index];

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
}

// partial FK solver
a3i32 a3kinematicsSolveForwardPartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// implement forward kinematics algorithm: 
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- object matrix = parent object matrix * local matrix
		//		- else
		//			- copy local matrix to object matrix
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

		const a3_HierarchyNode* itr = hierarchyState->hierarchy->nodes + firstIndex;
		const a3_HierarchyNode* const end = itr + nodeCount;
		for (; itr < end; ++itr)
		{
			if (itr->parentIndex >= 0)
				a3kinematicsSolveForwardSingle(hierarchyState, itr->index, itr->parentIndex);
			else
				a3kinematicsSolveForwardRoot(hierarchyState, itr->index);
		}
		return (a3i32)(end - itr);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
	return -1;
}


//-----------------------------------------------------------------------------

// single IK helpers
static inline void a3kinematicsSolveInverseSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	a3real4x4Product(hierarchyState->localSpace->hpose_base[index].transformMat.m,
		hierarchyState->objectSpaceInv->hpose_base[parentIndex].transformMat.m,
		hierarchyState->objectSpace->hpose_base[index].transformMat.m);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}
static inline void a3kinematicsSolveInverseRoot(const a3_HierarchyState* hierarchyState, const a3ui32 index)
{
	//-----------------------------------------------------------------------------
	//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
	//-----------------------------------------------------------------------------

	hierarchyState->localSpace->hpose_base[index] = hierarchyState->objectSpace->hpose_base[index];

	//-----------------------------------------------------------------------------
	//****END-TO-DO-PROJECT-3
	//-----------------------------------------------------------------------------
}


// partial IK solver
a3i32 a3kinematicsSolveInversePartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// implement inverse kinematics algorithm: 
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- local matrix = inverse parent object matrix * object matrix
		//		- else
		//			- copy object matrix to local matrix
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3ui32 i;
		for (i = firstIndex; i < nodeCount; ++i)
		{
			if (hierarchyState->hierarchy->nodes[i].parentIndex < 0)
			{
				// we are root
				a3kinematicsSolveInverseRoot(hierarchyState, hierarchyState->hierarchy->nodes[i].index);
			}
			else
			{
				// we are not root
				a3kinematicsSolveInverseSingle(hierarchyState,
					hierarchyState->hierarchy->nodes[i].index,
					hierarchyState->hierarchy->nodes[i].parentIndex);
			}
		}

		//-----------------------------------------------------------------------------
		//****END-TO-DO-PROJECT-3
		//-----------------------------------------------------------------------------
	}
	return -1;
}



//-----------------------------------------------------------------------------

void a3kinematicsUpdateHierarchyStateFK(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		// FK pipeline
		//	-> concatenate base pose
		//	-> convert poses to local-space matrices
		//	-> perform recursive FK
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3hierarchyPoseConcat(activeHS->localSpace,	// local: goal to calculate
			activeHS->animPose,						// holds current sample pose
			baseHS->localSpace,						// holds base pose (animPose is all identity poses)
			activeHS->hierarchy->numNodes);
		a3hierarchyPoseConvert(activeHS->localSpace,
			activeHS->hierarchy->numNodes,
			poseGroup->channel,
			poseGroup->order);
		a3kinematicsSolveForward(activeHS);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
}

void a3kinematicsUpdateHierarchyStateIK(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		// IK pipeline
		//	-> perform recursive IK
		//	-> restore local-space matrices to poses
		//	-> deconcatenate base pose
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3kinematicsSolveInverse(activeHS); // Finally, do FK algorithm
		a3hierarchyPoseRestore(
			activeHS->localSpace, // GOAL: convert local pose description to matrix
			activeHS->hierarchy->numNodes,
			poseGroup->channel,
			poseGroup->order
		);
		a3hierarchyPoseDeconcat(
			activeHS->animPose, // GOAL: local pose = total of base and delta
			activeHS->localSpace, // delta pose (from clip controller interpolation)
			baseHS->localSpace, // precomputed base pose
			activeHS->hierarchy->numNodes
		);

		//-----------------------------------------------------------------------------
		//****END-TO-DO-PROJECT-3
		//-----------------------------------------------------------------------------
	}
}


void a3kinematicsUpdateHierarchyStateSkin(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS)
{
	if (activeHS->hierarchy == baseHS->hierarchy)
	{
		// FK pipeline extended for skinning and other applications
		//	-> update local-space inverse matrices
		//	-> update object-space inverse matrices
		//	-> update transform from base to current
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3hierarchyStateUpdateLocalInverse(activeHS);
		a3hierarchyStateUpdateObjectInverse(activeHS);
		a3hierarchyStateUpdateObjectBindToCurrent(activeHS, baseHS);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
}


//-----------------------------------------------------------------------------

// helper to resolve single-joint IK after solver
static void a3kinematicsResolvePostIK(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup,
	a3ui32 const nodeIndex, a3real4x4 const j2obj)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// post-IK resolution for single affected joint
	//	-> reassign resolved transform to object-space
	a3real4x4SetReal4x4(activeHS->objectSpace->hpose_base[nodeIndex].transformMat.m, j2obj);

	//	-> compute object-space inverse matrix
	a3real4x4TransformInverse(activeHS->objectSpaceInv->hpose_base[nodeIndex].transformMat.m, j2obj); // might need to use a3real4x4TransformInverse

	//	-> compute local-space matrix
	a3kinematicsSolveInverseSingle(activeHS, nodeIndex, activeHS->hierarchy->nodes[nodeIndex].parentIndex);

	//	-> restore local-space matrix to pose
	a3spatialPoseRestore(activeHS->localSpace->hpose_base + nodeIndex, *poseGroup->channel, *poseGroup->order);

	//	-> deconcatenate base pose
	a3spatialPoseDeconcat(activeHS->animPose->hpose_base + nodeIndex, activeHS->localSpace->hpose_base + nodeIndex, baseHS->localSpace->hpose_base + nodeIndex);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}

void a3kinematicsUpdateLookAtIK(a3_HierarchyState const* sceneGraphState,
	a3_HierarchyState* activeHS, a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup,
	a3ui32 const sceneGraphIndex_hierarchyObj, a3ui32 const sceneGraphIndex_effector,
	a3ui32 const hierarchyObjIndex_affected, a3_Basis const basis_hierarchyObj, a3_Basis const basis_affected)
{
	a3mat3 m_hierarchyObj, m_affected;
	if (!a3basisToMat3(m_hierarchyObj.m, basis_hierarchyObj))
		return;
	if (!a3basisToMat3(m_affected.m, basis_affected))
		return;

	if ((!sceneGraphState || !activeHS || !baseHS || !poseGroup) ||
		(activeHS->hierarchy != baseHS->hierarchy) ||
		(activeHS->hierarchy != poseGroup->hierarchy))
		return;
	
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// First step:
	// transform everything into the space of the skeleton/hierarchy
	// -> look at target

	a3real4x4* worldToJointSpace = &sceneGraphState->localSpaceInv->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;

	// Put effector pos in joint space
	a3vec4 effectorPosInJ;
	a3real4ProductTransform(effectorPosInJ.v, sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector].transformMat.v3.v, *worldToJointSpace);

	//a3mat4 rigSpace = sceneGraphState->localSpace->hpose_base
	//a3real3x3MakeLookAt

	// Main step:
	// solver: build an orthonormal basis (Joint-to-object)
	//	1. direction basis = target - joint position
	a3vec3 jointPos = activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.xyz;
	a3vec3 directionBasis;
	a3real3Diff(directionBasis.v, effectorPosInJ.v, jointPos.v);

	//	2. side basis = known up x direction basis
	a3vec3 up = {0, 1, 0};
	a3vec3 sideBasis;
	a3real3Cross(&sideBasis.x, &up.x, &directionBasis.x);


	//	3. up basis = direction basis x side basis
	a3vec3 upBasis;
	a3real3Cross(&upBasis.x, &directionBasis.x, &sideBasis.x);
	
	//	4. normalize all
	a3real3Normalize(&directionBasis.x);
	a3real3Normalize(&sideBasis.x);
	a3real3Normalize(&upBasis.x);

	// Make transform
	a3mat4 lookAt;

	// -> Basis vectors
	a3real4Set(lookAt.v0.v, sideBasis.x, sideBasis.y, sideBasis.z, 0);
	a3real4Set(lookAt.v1.v, upBasis.x, upBasis.y, upBasis.z, 0);
	a3real4Set(lookAt.v2.v, directionBasis.x, directionBasis.y, directionBasis.z, 0);

	// -> Translation
	a3real4Set(lookAt.v3.v, jointPos.x, jointPos.y, jointPos.z, 1);

	// Last step:
	// resolve every affected joint:
	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected, lookAt.m);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}

void a3kinematicsUpdateLimbIK(a3_HierarchyState const* sceneGraphState,
	a3_HierarchyState* activeHS, a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup,
	a3ui32 const sceneGraphIndex_hierarchyObj, a3ui32 const sceneGraphIndex_effector_end, a3ui32 const sceneGraphIndex_constraint,
	a3ui32 const hierarchyObjIndex_affected_end, a3ui32 const hierarchyObjIndex_affected_hinge, a3ui32 const hierarchyObjIndex_affected_base,
	a3_Basis const basis_hierarchyObj, a3_Basis const basis_affected_end, a3_Basis const basis_affected_hinge, a3_Basis const basis_affected_base)
{
	a3mat3 m_hierarchyObj, m_affected_end, m_affected_hinge, m_affected_base;
	if (!a3basisToMat3(m_hierarchyObj.m, basis_hierarchyObj))
		return;
	if (!a3basisToMat3(m_affected_end.m, basis_affected_end))
		return;
	if (!a3basisToMat3(m_affected_hinge.m, basis_affected_hinge))
		return;
	if (!a3basisToMat3(m_affected_base.m, basis_affected_base))
		return;

	if ((!sceneGraphState || !activeHS || !baseHS || !poseGroup) ||
		(activeHS->hierarchy != baseHS->hierarchy) ||
		(activeHS->hierarchy != poseGroup->hierarchy))
		return;
	
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// First step:
	// transform everything into the space of the skeleton/hierarchy
	a3real4x4* worldToJointSpace = &sceneGraphState->localSpaceInv->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;

	// -> wrist effector
	a3vec4 effectorPosInJ;
	a3real4ProductTransform(effectorPosInJ.v, sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector_end].transformMat.v3.v, *worldToJointSpace);

	// -> pole vector constraint
	a3vec4 constraintPosInJ;
	a3real4ProductTransform(constraintPosInJ.v, sceneGraphState->localSpace->hpose_base[sceneGraphIndex_constraint].transformMat.v3.v, *worldToJointSpace);

	// Main step:
	// solve joint-object for end, hinge, base
	//	-> end position*
	a3vec4 wristPosInJ;
	a3real4ProductTransform(wristPosInJ.v, activeHS->localSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v3.v, *worldToJointSpace);

	//  -> hinge position*
	a3vec4 elbowPosInJ;
	a3real4ProductTransform(elbowPosInJ.v, activeHS->localSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3.v, *worldToJointSpace);

	//	-> base position
	a3vec3 basePosInJ = activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.xyz;

	// 1. base joint to end effector vector (and distance)
	a3vec3 baseToEnd;
	a3real3Diff(baseToEnd.v, effectorPosInJ.v, basePosInJ.v);
	a3f32 baseToEndDist = a3real3Distance(effectorPosInJ.v, basePosInJ.v);

	// 2. base joint to pole vector constraint
	a3vec3 baseToConstraint;
	a3real3Diff(baseToConstraint.v, constraintPosInJ.v, basePosInJ.v);

	// 3. plane normal = base-to-pole x base-to-end
	a3vec3 planeNormal;
	a3real3Cross(&planeNormal.x, &baseToConstraint.x, &baseToEnd.x);
	a3real3Normalize(&planeNormal.x);

	// 4. geometric (Heron's formula) or algebraic (law of cosines)
	a3real area, s;

	// Get distances for limbs
	a3real baseToHingeDist = a3real3Distance(basePosInJ.v, elbowPosInJ.v);
	a3real hingeToEndDist = a3real3Distance(elbowPosInJ.v, effectorPosInJ.v);

	// Heron's formula to solve for area
	s = (a3real)0.5 * (baseToEndDist + baseToHingeDist + hingeToEndDist);
	area = s * ((s - baseToEndDist) * (s - baseToHingeDist) * (s - hingeToEndDist));
	area = a3sqrtf(area);

	// Solve for side lengths of triangle to new elbow pos
	a3real height = (2 * area) / baseToEndDist;
	a3real distToMiddle = (baseToHingeDist * baseToHingeDist) - (height * height);
	distToMiddle = a3sqrtf(distToMiddle);

	//	-> solves elbow position
	// direction of base to effector
	a3vec3 baseNormalized = baseToEnd;
	a3real3Normalize(baseNormalized.v);

	// direction to elbow from D
	a3vec3 heightNormalized;
	a3real3CrossUnit(heightNormalized.v, planeNormal.v, baseNormalized.v);

	// Make vectors in directions with calulated lengths
	a3vec3 distToMiddleVec, heightVec;
	a3real3ProductS(distToMiddleVec.v, baseNormalized.v, distToMiddle);
	a3real3ProductS(heightVec.v, heightNormalized.v, height);

	// Add vectors to get new elbow pos
	a3vec3 newElbowPosInJ = basePosInJ;
	a3real3Add(newElbowPosInJ.v, distToMiddleVec.v);
	a3real3Add(newElbowPosInJ.v, heightVec.v);

	// 5. "look at" solves shoulder and elbow rotations
	// Geometric solution in slides

	// Make new transform for base joint
	//	-> make basis vectors
	a3vec3 directionBasis, upBasis, sideBasis;
	a3real3Diff(directionBasis.v, newElbowPosInJ.v, basePosInJ.v);
	upBasis = planeNormal;
	a3real3Cross(sideBasis.v, directionBasis.v, upBasis.v);

	// -> normalize basis vectors
	a3real3Normalize(&directionBasis.x);
	a3real3Normalize(&upBasis.x);
	a3real3Normalize(&sideBasis.x);	

	// -> make transform matrix
	a3mat4 baseLookAt;
	a3real4Set(baseLookAt.v0.v, sideBasis.x, sideBasis.y, sideBasis.z, 0);
	a3real4Set(baseLookAt.v1.v, upBasis.x, upBasis.y, upBasis.z, 0);
	a3real4Set(baseLookAt.v2.v, directionBasis.x, directionBasis.y, directionBasis.z, 0);

	// -> translation
	a3real4Set(baseLookAt.v3.v, basePosInJ.x, basePosInJ.y, basePosInJ.z, 1);

	// Resolve base
	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_base, baseLookAt.m);


	// Make new transform for elbow joint
	//	-> make basis vectors
	a3real3Diff(directionBasis.v, effectorPosInJ.v, newElbowPosInJ.v);
	upBasis = planeNormal;
	a3real3Cross(sideBasis.v, directionBasis.v, upBasis.v);

	// -> normalize basis vectors
	a3real3Normalize(&directionBasis.x);
	a3real3Normalize(&upBasis.x);
	a3real3Normalize(&sideBasis.x);

	// -> make transform matrix
	a3mat4 elbowLookAt;
	a3real4Set(elbowLookAt.v0.v, sideBasis.x, sideBasis.y, sideBasis.z, 0);
	a3real4Set(elbowLookAt.v1.v, upBasis.x, upBasis.y, upBasis.z, 0);
	a3real4Set(elbowLookAt.v2.v, directionBasis.x, directionBasis.y, directionBasis.z, 0);

	// -> translation
	a3real4Set(elbowLookAt.v3.v, newElbowPosInJ.x, newElbowPosInJ.y, newElbowPosInJ.z, 1);

	//a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_hinge, elbowLookAt.m);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}


//-----------------------------------------------------------------------------
