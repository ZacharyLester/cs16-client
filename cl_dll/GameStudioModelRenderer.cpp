/***
*
*	Copyright (c) 1996-2002, Valve LLC. All rights reserved.
*
*	This product contains software technology licensed from Id
*	Software, Inc. ("Id Technology").  Id Technology (c) 1996 Id Software, Inc.
*	All Rights Reserved.
*
*   Use, distribution, and modification of this source code and/or resulting
*   object code is restricted to non-commercial enhancements to products from
*   Valve LLC.  All other use, distribution, or modification is prohibited
*   without written permission from Valve LLC.
*
****/

// Big thanks to Chicken Fortress developers
// for this code.
#include <assert.h>
#include "hud.h"
#include "cl_util.h"
#include "const.h"
#include "com_model.h"
#include "studio.h"
#include "entity_state.h"
#include "cl_entity.h"
#include "dlight.h"
#include "triangleapi.h"

#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <math.h>

#include "studio_util.h"
#include "r_studioint.h"

#include "StudioModelRenderer.h"
#include "GameStudioModelRenderer.h"
#include "pm_defs.h"
#include "camera.h"
#include "eventscripts.h"

#define ANIM_WALK_SEQUENCE 3
#define ANIM_JUMP_SEQUENCE 6
#define ANIM_SWIM_1 8
#define ANIM_SWIM_2 9
#define ANIM_FIRST_DEATH_SEQUENCE 101
#define ANIM_LAST_DEATH_SEQUENCE 159
#define ANIM_FIRST_EMOTION_SEQUENCE 198
#define ANIM_LAST_EMOTION_SEQUENCE 207

//ragdoll stuff start
#include <cstring>
#include <cmath>
#include <algorithm>

static const RagdollBoneDesc k_BoneDescs[RB_COUNT] = {
	{ RB_PELVIS,     "Bip01 Pelvis",    "Bip01 Pelvis",     RB_COUNT,       20.0f, 6.0f, 0.0f },
	{ RB_SPINE,      "Bip01 Spine",     "Bip01 Spine1",     RB_PELVIS,       6.0f, 5.0f, 0.5f },
	{ RB_SPINE1,     "Bip01 Spine1",    "Bip01 Head",       RB_SPINE,        5.0f, 5.0f, 0.5f },
	{ RB_HEAD,       "Bip01 Head",      "Bip01 Neck",       RB_SPINE1,       4.0f, 4.5f, 0.5f },
	{ RB_UPPER_ARM_L,"Bip01 L UpperArm","Bip01 L Forearm",  RB_SPINE1,       2.0f, 3.0f, 0.5f },

	{ RB_FOREARM_L, "Bip01 L Forearm", "Bip01 L Hand", RB_UPPER_ARM_L, 3.0f, 2.5f, 0.5f },

	{ RB_UPPER_ARM_R,"Bip01 R UpperArm","Bip01 R Forearm",  RB_SPINE1,       2.0f, 3.0f, 0.5f },

	{ RB_FOREARM_R, "Bip01 R Forearm", "Bip01 R Hand", RB_UPPER_ARM_R, 3.0f, 2.5f, 0.5f },

	{ RB_THIGH_L,    "Bip01 L Thigh",   "Bip01 L Calf",     RB_PELVIS,       4.0f, 3.5f, 0.4f },
	{ RB_CALF_L,     "Bip01 L Calf",    "Bip01 L Foot",     RB_THIGH_L,      2.5f, 3.0f, 0.4f },
	{ RB_THIGH_R,    "Bip01 R Thigh",   "Bip01 R Calf",     RB_PELVIS,       4.0f, 3.5f, 0.4f },
	{ RB_CALF_R,     "Bip01 R Calf",    "Bip01 R Foot",     RB_THIGH_R,      2.5f, 3.0f, 0.4f },
	{ RB_FOOT_L,     "Bip01 L Foot",    "Bip01 L Toe0",     RB_CALF_L,       1.0f, 2.0f, 0.3f },
	{ RB_FOOT_R,     "Bip01 R Foot",    "Bip01 R Toe0",     RB_CALF_R,       1.0f, 2.0f, 0.3f },
};

CRagdollWorld &CRagdollWorld::Get()
{
	static CRagdollWorld s_instance;
	return s_instance;
}

void CRagdollWorld::Init()
{
	if (m_world)
		return;

	m_config     = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_config);
	m_broadphase = new btDbvtBroadphase();
	m_solver     = new btSequentialImpulseConstraintSolver();
	m_world      = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_config);

	m_world->setGravity(btVector3(0, 0, -800.0f * GU_TO_M));
}

void CRagdollWorld::NotifyMapChanged()
{
	if (m_world && m_worldBody)
	{
		m_world->removeRigidBody(m_worldBody);
		delete m_worldBody->getMotionState();
		delete m_worldBody;	m_worldBody = nullptr;
	}
	delete m_worldShape;	m_worldShape = nullptr;
	delete m_worldMesh;	m_worldMesh = nullptr;

	m_currentMapName[0] = '\0';

	CRagdollManager::Get().RemoveAllRagdolls();
}

void CRagdollWorld::Shutdown()
{
	if (m_world && m_worldBody)
	{
		m_world->removeRigidBody(m_worldBody);
		delete m_worldBody->getMotionState();
		delete m_worldBody;	m_worldBody  = nullptr;
	}
	delete m_worldShape;	m_worldShape = nullptr;
	delete m_worldMesh;		m_worldMesh  = nullptr;

	delete m_world;     m_world      = nullptr;
	delete m_solver;    m_solver     = nullptr;
	delete m_broadphase;m_broadphase = nullptr;
	delete m_dispatcher;m_dispatcher = nullptr;
	delete m_config;    m_config     = nullptr;
}

void CRagdollWorld::Step(float dt)
{
	if (!m_world)
		return;

	m_world->stepSimulation(dt, 4, 1.0f / 120.0f);
}

void CRagdollWorld::EnsureWorldCollision()
{
	if (!m_world) return;

	model_t *worldModel = IEngineStudio.GetModelByIndex(1);
	if (!worldModel || worldModel->type != mod_brush)
		return;

	const char *mapName = worldModel->name;

	if (m_worldBody && strncmp(m_currentMapName, mapName, sizeof(m_currentMapName)) == 0)
		return;

	if (m_worldBody)
	{
		m_world->removeRigidBody(m_worldBody);
		delete m_worldBody->getMotionState();
		delete m_worldBody;	m_worldBody = nullptr;
	}
	delete m_worldShape;	m_worldShape = nullptr;
	delete m_worldMesh;	m_worldMesh = nullptr;

	strncpy(m_currentMapName, mapName, sizeof(m_currentMapName) - 1);

	m_worldMesh = new btTriangleMesh(/*use32bitIndices=*/true);

	int triCount = 0;

	for (int i = 0; i < worldModel->numsurfaces; i++)
	{
		msurface_t *surf = worldModel->surfaces + i;

		if (surf->flags & (SURF_DRAWTURB | SURF_DRAWSKY | SURF_UNDERWATER))
			continue;

		glpoly_t *poly = surf->polys;
		if (!poly)
			continue;

		float *v0 = poly->verts[0];

		for (int j = 1; j < poly->numverts - 1; j++)
		{
			float *v1 = poly->verts[j];
			float *v2 = poly->verts[j + 1];

			btVector3 p0(v0[0] * GU_TO_M, v0[1] * GU_TO_M, v0[2] * GU_TO_M);
			btVector3 p1(v1[0] * GU_TO_M, v1[1] * GU_TO_M, v1[2] * GU_TO_M);
			btVector3 p2(v2[0] * GU_TO_M, v2[1] * GU_TO_M, v2[2] * GU_TO_M);

			m_worldMesh->addTriangle(p0, p1, p2, /*removeDuplicateVertices=*/false);
			triCount++;
		}
	}

	if (triCount == 0)
	{
		delete m_worldMesh;
		m_worldMesh  = nullptr;
		m_worldShape = nullptr;

		btCollisionShape *fallback = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
		btRigidBody::btRigidBodyConstructionInfo ci(0, new btDefaultMotionState(), fallback);
		m_worldBody = new btRigidBody(ci);
		m_world->addRigidBody(m_worldBody, 1, 2);
		return;
	}

	//btBvhTriangleMeshShape builds an AABB tree for fast ray/shape queries.
	//trueflag tells it to build the BVH immediately.
	m_worldShape = new btBvhTriangleMeshShape(m_worldMesh, /*buildBvh=*/true);

	btRigidBody::btRigidBodyConstructionInfo ci(
		0,//mass 0 = static
		new btDefaultMotionState(),
		m_worldShape
	);
	ci.m_friction    = 0.8f;
	ci.m_restitution = 0.0f;

	m_worldBody = new btRigidBody(ci);
	//CF_STATIC_OBJECT tells the broadphase this never moves
	m_worldBody->setCollisionFlags(
		m_worldBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT
	);

	m_world->addRigidBody(m_worldBody, 1, 2);
	//gEngfuncs.Con_Printf("cl ragdoll: bsp collision built (%d triangles)\n", triCount);
}


CRagdollManager &CRagdollManager::Get()
{
	static CRagdollManager s_instance;
	return s_instance;
}

void CRagdollManager::Update(float dt)
{
	CRagdollWorld::Get().Step(dt);

	btDiscreteDynamicsWorld* world = CRagdollWorld::Get().GetWorld();
	if (!world) return;

	float now = gEngfuncs.GetClientTime();
	for (auto it = m_ragdolls.begin(); it != m_ragdolls.end(); )
	{
		Ragdoll& rd = it->second;
		float age = now - rd.deathTime;

		for (int slot = 0; slot < RB_COUNT; slot++)
		{
			if (slot != RB_PELVIS   &&
			    slot != RB_THIGH_L  && slot != RB_THIGH_R &&
			    slot != RB_CALF_L   && slot != RB_CALF_R  &&
			    slot != RB_FOOT_L   && slot != RB_FOOT_R)
				continue;

			RagdollBody& part = rd.parts[slot];
			if (!part.body || !part.shape) continue;

			btCapsuleShape* capsule = (btCapsuleShape*)part.shape;
			float r  = capsule->getRadius();
			float hh = capsule->getHalfHeight();

			btTransform xform = part.body->getWorldTransform();
			btVector3 org = xform.getOrigin();

			btVector3 localTop(0,  hh + r, 0);
			btVector3 localBot(0, -(hh + r), 0);
			btVector3 worldTop = xform * localTop;
			btVector3 worldBot = xform * localBot;
			float bottomZ = btMin(worldTop.z(), worldBot.z());

			float ox = org.x() * M_TO_GU;
			float oy = org.y() * M_TO_GU;
			float oz = org.z() * M_TO_GU;

			float traceStart[3] = { ox, oy, oz };
			float traceEnd[3]   = { ox, oy, oz - 64.0f };

			pmtrace_t tr;
			gEngfuncs.pEventAPI->EV_SetUpPlayerPrediction(false, true);
			gEngfuncs.pEventAPI->EV_PushPMStates();
			gEngfuncs.pEventAPI->EV_SetSolidPlayers(-1);
			gEngfuncs.pEventAPI->EV_SetTraceHull(2);
			gEngfuncs.pEventAPI->EV_PlayerTrace(traceStart, traceEnd,
				PM_STUDIO_IGNORE | PM_GLASS_IGNORE, -1, &tr);
			gEngfuncs.pEventAPI->EV_PopPMStates();

			if (!tr.startsolid && tr.fraction < 1.0f)
			{
				float floorBt = tr.endpos[2] * GU_TO_M;
				if (bottomZ < floorBt)
				{
					org.setZ(org.z() + (floorBt - bottomZ));
					xform.setOrigin(org);
					part.body->setWorldTransform(xform);
					part.body->getMotionState()->setWorldTransform(xform);

					btVector3 vel = part.body->getLinearVelocity();
					if (vel.z() < 0) vel.setZ(0);
					part.body->setLinearVelocity(vel);
					part.body->activate();
				}
			}
		}

		if (age > 10.0f)
		{
			int idx = it->first;
			++it;
			RemoveRagdoll(idx);
		}
		else ++it;
	}
}

Vector CRagdollManager::GetRagdollOrigin(int entityIndex)
{
	auto it = m_ragdolls.find(entityIndex);
	if (it == m_ragdolls.end()) return Vector(0,0,0);

	Ragdoll &rd = it->second;
	if (!rd.parts[RB_PELVIS].body) return Vector(0,0,0);

	btTransform t;
	rd.parts[RB_PELVIS].motionState->getWorldTransform(t);
	btVector3 pos = t.getOrigin();

	return Vector(
		pos.x() * M_TO_GU,
		pos.y() * M_TO_GU,
		pos.z() * M_TO_GU
	);
}

void CRagdollManager::ApplyImpulse(int entityIndex, const Vector &hitPos, const Vector &dir, float strength)
{
	auto it = m_ragdolls.find(entityIndex);
	if (it == m_ragdolls.end() || !it->second.active) return;

	Ragdoll &rd = it->second;

	btVector3 impulse(
		dir.x * strength * GU_TO_M,
		dir.y * strength * GU_TO_M,
		dir.z * strength * GU_TO_M
	);

	btVector3 hitBt(
		hitPos.x * GU_TO_M,
		hitPos.y * GU_TO_M,
		hitPos.z * GU_TO_M
	);

	// find the closest body to the hit position
	float bestDist = 1e9f;
	btRigidBody *bestBody = nullptr;

	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		if (!rd.parts[slot].body) continue;

		btVector3 bodyPos = rd.parts[slot].body->getWorldTransform().getOrigin();
		float dist = (bodyPos - hitBt).length();

		if (dist < bestDist)
		{
			bestDist = dist;
			bestBody = rd.parts[slot].body;
		}
	}

	if (bestBody)
	{
		bestBody->activate();
		bestBody->applyImpulse(impulse, hitBt - bestBody->getWorldTransform().getOrigin());
	}
}

void CRagdollManager::PushFromPlayers()
{
	cl_entity_t *local = gEngfuncs.GetLocalPlayer();
	if (!local) return;

	for (auto &[ragIdx, rd] : m_ragdolls)
	{
		if (!rd.active || !rd.parts[RB_PELVIS].body) continue;

		btVector3 pelvisPos = rd.parts[RB_PELVIS].body->getWorldTransform().getOrigin();
		Vector ragOrigin(
			pelvisPos.x() * M_TO_GU,
			pelvisPos.y() * M_TO_GU,
			pelvisPos.z() * M_TO_GU
		);

		Vector delta = ragOrigin - Vector(local->curstate.origin);
		float dist = delta.Length();
		if ( dist < 1.0f ) continue;

		Vector pushDir = delta * ( 1.0f / dist ); 
		pushDir.z += 0.3f;

		float strength = (1.0f - dist / 50.0f) * 80.0f;

		bool isInside = dist < 50.0f;
		if (isInside && !rd.playerWasInside)
		{
			int pushBodies[] = { RB_PELVIS, RB_SPINE };
			for ( int slot : pushBodies )
			{
				if ( !rd.parts[slot].body ) continue;
				rd.parts[slot].body->activate();
				rd.parts[slot].body->applyCentralImpulse( btVector3(
				pushDir.x * strength,
				pushDir.y * strength,
				pushDir.z * strength));
			}
		}
		rd.playerWasInside = isInside;
	}
}

static const int MAX_RAGDOLL_BONES = 128;

void CRagdollManager::SpawnRagdoll(int entityIndex, studiohdr_t *hdr, float bonetransform[][3][4], const float	*velocity)
{
	if (!hdr)
		return;

	CRagdollWorld::Get().Init();
	CRagdollWorld::Get().EnsureWorldCollision();
	btDiscreteDynamicsWorld *world = CRagdollWorld::Get().GetWorld();

	RemoveRagdoll(entityIndex);

	Ragdoll &rd    = m_ragdolls[entityIndex];
	rd.active      = true;

	rd.deathTime   = gEngfuncs.GetClientTime();
	rd.spawnTime = gEngfuncs.GetClientTime();

	mstudiobone_t *pbones = (mstudiobone_t *)((byte *)hdr + hdr->boneindex);

	for (int i = 0; i < 128; i++)
		rd.boneToSlot[i] = -1;

	//pass 1 : find the studiohdr bone index for each rigid body slot
	//strstr match so "Bip01 L Thigh" finds "Bip01 L Thigh" even on models
	//that append suffixes like "_twist"
	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		rd.parts[slot].slot       = (ERagdollBody)slot;
		rd.parts[slot].boneIndex  = -1;
		rd.parts[slot].pboneIndex = -1;

		for (int b = 0; b < hdr->numbones && b < MAX_RAGDOLL_BONES; b++)
		{
			if (rd.parts[slot].boneIndex < 0 &&
			    strstr(pbones[b].name, k_BoneDescs[slot].boneName))
				rd.parts[slot].boneIndex = b;

			if (rd.parts[slot].pboneIndex < 0 &&
			    strstr(pbones[b].name, k_BoneDescs[slot].pboneName))
				rd.parts[slot].pboneIndex = b;

			if (rd.parts[slot].boneIndex >= 0 && rd.parts[slot].pboneIndex >= 0)
				break;
		}
	}

	//pass 2: map every studiohdr bone to its nearest owning rigid body
	//first mark bones that directly own a slot, then walk the parent chain for
	//the rest so every bone ends up assigned to something
	for (int b = 0; b < hdr->numbones && b < MAX_RAGDOLL_BONES; b++)
	{
		for (int slot = 0; slot < RB_COUNT; slot++)
		{
			if (rd.parts[slot].boneIndex == b)
			{
				rd.boneToSlot[b] = slot;
				break;
			}
		}

		if (rd.boneToSlot[b] >= 0)
			continue;

		//not a direct owner, walk up until we find an ancestor that is
		for (int cur = pbones[b].parent; cur != -1; cur = pbones[cur].parent)
		{
			for (int slot = 0; slot < RB_COUNT; slot++)
			{
				if (rd.parts[slot].boneIndex == cur)
				{
					rd.boneToSlot[b] = slot;
					goto next_bone;
				}
			}
		}
		next_bone:;
	}

	//pass 3: create body
	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		const RagdollBoneDesc &desc = k_BoneDescs[slot];
		RagdollBody           &part = rd.parts[slot];

		if (part.boneIndex < 0) continue;

		btVector3 posNear = BoneMatToBt(bonetransform[part.boneIndex]).getOrigin();
		btVector3 posFar  = (part.pboneIndex >= 0)
		                    ? BoneMatToBt(bonetransform[part.pboneIndex]).getOrigin()
		                    : posNear;

		float halfH = 0.0f;
		btTransform xform = BuildCapsuleTransform(
			bonetransform[part.boneIndex], posNear, posFar,
			desc.pboneoffset, halfH);

		part.shape       = new btCapsuleShape(desc.radius * GU_TO_M, halfH * 2.0f);
		part.motionState = new btDefaultMotionState(xform);

		btVector3 inertia(0, 0, 0);
		part.shape->calculateLocalInertia(desc.mass, inertia);

		btRigidBody::btRigidBodyConstructionInfo ci(desc.mass, part.motionState, part.shape, inertia);

		if (desc.mass <= 2.0f)
		{
			ci.m_linearDamping  = 0.9f;
			ci.m_angularDamping = 0.99f;
		}
		else if (desc.mass <= 4.0f)
		{
			ci.m_linearDamping  = 0.7f;
			ci.m_angularDamping = 0.90f;
		}
		else
		{
			ci.m_linearDamping  = 0.5f;
			ci.m_angularDamping = 0.8f;
		}

		ci.m_friction    = 0.6f;
		ci.m_restitution = 0.1f;

		part.body = new btRigidBody(ci);
		part.body->setUserIndex(slot);
		part.body->setSleepingThresholds(0.5f * GU_TO_M, 0.05f);
		part.body->setDeactivationTime(0.3f);

		world->addRigidBody(part.body, 2, 1);
	}

	//pass 4: record each bone's fixed offset from its owning body
	//at spawn time the bodies sit exactly on their bone origins, so the offset
	//is just inv(bodyWorld) * boneWorld.  GetRagdollBones replays this each
	//frame: boneWorld = bodyWorld * boneLocalToBody[b].
	for (int b = 0; b < hdr->numbones && b < MAX_RAGDOLL_BONES; b++)
	{
		int slot = rd.boneToSlot[b];
		if (slot < 0 || slot >= RB_COUNT || !rd.parts[slot].body)
		{
			rd.boneLocalToBody[b].setIdentity();
			continue;
		}

		btTransform boneWorld = BoneMatToBt(bonetransform[b]);
		btTransform bodyWorld = rd.parts[slot].body->getWorldTransform();
		btTransform local = bodyWorld.inverseTimes(boneWorld);

		if (slot == RB_CALF_L || slot == RB_CALF_R)
		{
			btVector3 localOrigin = local.getOrigin();
			if (localOrigin.y() < 0)  //negative Y = below capsule center
				localOrigin.setY(localOrigin.y() * 0.5f); //reduce by half
			local.setOrigin(localOrigin);
		}

		rd.boneLocalToBody[b] = local;
	}

	//pass 5: ground clamp
	//trace downward from the pelvis to find the floor Z, then clamp all bodies
	bool wasCrouching = false;

	{
		btVector3 pelvisPos(0, 0, 0);
		if (rd.parts[RB_PELVIS].body)
			pelvisPos = rd.parts[RB_PELVIS].body->getWorldTransform().getOrigin();

		float ox = pelvisPos.x() * M_TO_GU;
		float oy = pelvisPos.y() * M_TO_GU;
		float oz = pelvisPos.z() * M_TO_GU;

		cl_entity_t *ent = gEngfuncs.GetEntityByIndex(entityIndex);

		if (ent)
		{
			float heightDiff = oz - ent->curstate.origin[2];
			wasCrouching = (heightDiff < 30.0f);
		}

		float traceStartZ = wasCrouching ? oz + 36.0f : oz;
		float traceStart[3] = { ox, oy, traceStartZ };
		float traceEnd[3]   = { ox, oy, traceStartZ - 512.0f };

		pmtrace_t tr;
		gEngfuncs.pEventAPI->EV_SetUpPlayerPrediction(false, true);
		gEngfuncs.pEventAPI->EV_PushPMStates();
			gEngfuncs.pEventAPI->EV_SetSolidPlayers(-1);
			gEngfuncs.pEventAPI->EV_SetTraceHull(2);

		/*gEngfuncs.pEventAPI->EV_PlayerTrace(traceStart, traceEnd,
			                                    PM_STUDIO_IGNORE | PM_GLASS_IGNORE,
			                                    -1, &tr);*/
//not really working..

		gEngfuncs.pEventAPI->EV_PlayerTrace(traceStart, traceEnd,
		PM_STUDIO_IGNORE | PM_GLASS_IGNORE,
		entityIndex,
		&tr);

		gEngfuncs.pEventAPI->EV_PopPMStates();

		float floorZ = (!tr.startsolid && tr.fraction < 1.0f)
		               ? tr.endpos[2]
		               : oz - 36.0f;

		float floorBt = floorZ * GU_TO_M;

		for (int slot = 0; slot < RB_COUNT; slot++)
		{
			btRigidBody *body = rd.parts[slot].body;
			if (!body) continue;

			btCapsuleShape *capsule = (btCapsuleShape *)rd.parts[slot].shape;
			float r  = capsule->getRadius();
			float hh = capsule->getHalfHeight();

			btTransform xform = body->getWorldTransform();
			btVector3 org = xform.getOrigin();

			btVector3 localTop (0,  hh, 0);
			btVector3 localBottom(0, -hh, 0);
			btVector3 worldTop = xform * localTop;
			btVector3 worldBottom = xform * localBottom;

			float bottomZ = btMin(worldTop.z(), worldBottom.z()) - r;

			if (bottomZ < floorBt)
			{
				org.setZ(org.z() + (floorBt - bottomZ));
				xform.setOrigin(org);
				body->setWorldTransform(xform);
				body->getMotionState()->setWorldTransform(xform);
			}

			btVector3 vel = body->getLinearVelocity();
			if (vel.z() > 0) vel.setZ(0);
			body->setLinearVelocity(vel);
		}

	}

	//pass 6: joint constraints
	BuildConstraints(rd, world);

	world->performDiscreteCollisionDetection();

	//pass 7: warmup simulation
	if (velocity && rd.parts[RB_SPINE1].body)
	{
		btVector3 hitDir(
		velocity[0] * GU_TO_M * 5.0f,
		velocity[1] * GU_TO_M * 5.0f,
		velocity[2] * GU_TO_M * 5.0f
		);

		rd.parts[RB_SPINE1].body->applyImpulse(hitDir, btVector3(0, 0, 0));
	}

	srand(entityIndex ^ (int)(gEngfuncs.GetClientTime() * 1000));
	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		if (!rd.parts[slot].body) continue;

		float dist = (rd.parts[slot].body->getWorldTransform().getOrigin() - rd.parts[RB_SPINE1].body->getWorldTransform().getOrigin()).length();
		float scale = 1.0f / (1.0f + dist * 5.0f);

		btVector3 angVel(
		((rand() % 200) - 100) * 0.01f * scale,
		((rand() % 200) - 100) * 0.01f * scale,
		((rand() % 200) - 100) * 0.01f * scale
		);

		rd.parts[slot].body->setAngularVelocity(angVel);
	}

	cl_entity_t *ent = gEngfuncs.GetEntityByIndex( entityIndex + 1 );
	if ( ent )
	{
		ent->curstate.colormap = 0;
		memset( ent->curstate.controller, 0, sizeof( ent->curstate.controller ) );
	}
}

struct JointLimit {
	ERagdollBody child;
	float swingSpan1; //radians, swing away from cone axis
	float swingSpan2;
	float twistSpan; //twist around cone axis
};

static const JointLimit k_JointLimits[] = {
	{ RB_SPINE,       0.25f, 0.15f, 0.10f },
	{ RB_SPINE1,      0.20f, 0.15f, 0.10f },

	{ RB_HEAD,        0.15f, 0.10f, 0.05f },

	{ RB_UPPER_ARM_L, 1.20f, 1.20f, 0.80f },

	{ RB_FOREARM_L,   0.90f, 0.40f, 0.15f },

	{ RB_UPPER_ARM_R, 1.20f, 1.20f, 0.80f },

	{ RB_FOREARM_R,   0.90f, 0.40f, 0.15f },

	{ RB_THIGH_L,     0.50f, 0.30f, 0.15f },
	{ RB_CALF_L,      0.60f, 0.05f, 0.05f },
	{ RB_THIGH_R,     0.50f, 0.30f, 0.15f },
	{ RB_CALF_R,      0.60f, 0.05f, 0.05f },
	{ RB_FOOT_L,      0.20f, 0.10f, 0.05f },
	{ RB_FOOT_R,      0.20f, 0.10f, 0.05f },
};


void CRagdollManager::BuildConstraints(Ragdoll &rd, btDiscreteDynamicsWorld *world)
{
	rd.constraintCount = 0;

	for (const JointLimit &jl : k_JointLimits)
	{
		ERagdollBody childSlot  = jl.child;
		ERagdollBody parentSlot = k_BoneDescs[childSlot].parentSlot;

		RagdollBody &child  = rd.parts[childSlot];
		RagdollBody &parent = rd.parts[parentSlot];

		if (!child.body || !parent.body)
			continue;

		//pivot location
		btVector3 pivotWorld = child.body->getWorldTransform().getOrigin();

		//convert pivot into each body's local space (translation only)
		btVector3 pivotInParentLocal =
			parent.body->getWorldTransform().inverse() * pivotWorld;
		btVector3 pivotInChildLocal =
			child.body->getWorldTransform().inverse()  * pivotWorld;

		//constraint frames
		btMatrix3x3 childBasis = child.body->getWorldTransform().getBasis();

		//express the child orientation in each body's local space
		btMatrix3x3 parentBasisInv = parent.body->getWorldTransform().getBasis().inverse();
		btMatrix3x3 frameRotInParent = parentBasisInv * childBasis;
		btMatrix3x3 frameRotInChild  = btMatrix3x3::getIdentity(); //child frame = child space

		btTransform frameInParent(frameRotInParent, pivotInParentLocal);
		btTransform frameInChild (frameRotInChild,  pivotInChildLocal);

		btConeTwistConstraint *ct = new btConeTwistConstraint(
			*parent.body, *child.body,
			frameInParent, frameInChild
		);

		ct->setLimit(jl.swingSpan1, jl.swingSpan2, jl.twistSpan,
			0.8f, //softness
			0.3f, //bias
			10.0f //relaxation
		);
		ct->setDamping(0.3f);//4.9

		world->addConstraint(ct, true);
		rd.constraints[rd.constraintCount++] = ct;
	}
}

bool CRagdollManager::GetRagdollBones(int entityIndex, studiohdr_t *hdr, float bonetransform[][3][4])
{
	auto it = m_ragdolls.find(entityIndex);
	if (it == m_ragdolls.end() || !it->second.active)
		return false;

	Ragdoll &rd = it->second;

	//snapshot each body's current world transform
	btTransform slotWorld[RB_COUNT];
	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		if (rd.parts[slot].body)
			rd.parts[slot].motionState->getWorldTransform(slotWorld[slot]);
		else
			slotWorld[slot].setIdentity();
	}

	//reconstruct every bone
	for (int b = 0; b < hdr->numbones && b < MAX_RAGDOLL_BONES; b++)
	{
		int slot = rd.boneToSlot[b];
		if (slot < 0 || slot >= RB_COUNT)
			continue;

		btTransform boneWorld = slotWorld[slot] * rd.boneLocalToBody[b];
		BtToBoneMat(boneWorld, bonetransform[b]);
	}

	return true;
}

void CRagdollManager::RemoveRagdoll(int entityIndex)
{
	auto it = m_ragdolls.find(entityIndex);
	if (it == m_ragdolls.end())
		return;

	Ragdoll &rd  = it->second;
	btDiscreteDynamicsWorld *world = CRagdollWorld::Get().GetWorld();

	if (world)
	{
		for (int i = 0; i < rd.constraintCount; i++)
		{
			world->removeConstraint(rd.constraints[i]);
			delete rd.constraints[i];
			rd.constraints[i] = nullptr;
		}

		for (int slot = 0; slot < RB_COUNT; slot++)
		{
			RagdollBody &part = rd.parts[slot];
			if (part.body)
			{
				world->removeRigidBody(part.body);
				delete part.body;
				part.body = nullptr;
			}

			if (part.motionState)
			{
				delete part.motionState;
				part.motionState = nullptr;
			}

			if (part.shape)
			{
				delete part.shape;
				part.shape = nullptr;
			}
		}
	}

	m_ragdolls.erase(it);
}

void CRagdollManager::RemoveAllRagdolls()
{
	std::vector<int> keys;
	for (auto &kv : m_ragdolls)
	keys.push_back(kv.first);

	for (int idx : keys)
	RemoveRagdoll(idx);
}

bool CRagdollManager::HasRagdoll(int entityIndex) const
{
	auto it = m_ragdolls.find(entityIndex);
	return it != m_ragdolls.end() && it->second.active;
}
//ragdoll stuff end

CGameStudioModelRenderer g_StudioRenderer;

int g_rseq;
int g_gaitseq;
vec3_t g_clorg;
vec3_t g_clang;

void CounterStrike_GetSequence(int *seq, int *gaitseq)
{
	*seq = g_rseq;
	*gaitseq = g_gaitseq;
}

void CounterStrike_GetOrientation(float *o, float *a)
{
	VectorCopy(g_clorg, o);
	VectorCopy(g_clang, a);
}

float g_flStartScaleTime;
int iPrevRenderState;
int iRenderStateChanged;

engine_studio_api_t IEngineStudio;

static client_anim_state_t g_state;
static client_anim_state_t g_clientstate;

CGameStudioModelRenderer::CGameStudioModelRenderer(void)
{
	m_bLocal = false;
}

mstudioanim_t *CGameStudioModelRenderer::LookupAnimation(mstudioseqdesc_t *pseqdesc, int index)
{
	mstudioanim_t *panim = NULL;

	panim = StudioGetAnim(m_pRenderModel, pseqdesc);

	if (index < 0)
		return panim;

	if (index > (pseqdesc->numblends - 1))
		return panim;

	panim += index * m_pStudioHeader->numbones;
	return panim;
}

void CGameStudioModelRenderer::StudioSetupBones(void)
{
	int i;
	double f;

	mstudiobone_t *pbones;
	mstudioseqdesc_t *pseqdesc;
	mstudioanim_t *panim;

	static float pos[MAXSTUDIOBONES][3];
	static vec4_t q[MAXSTUDIOBONES];
	float bonematrix[3][4];

	static float pos2[MAXSTUDIOBONES][3];
	static vec4_t q2[MAXSTUDIOBONES];
	static float pos3[MAXSTUDIOBONES][3];
	static vec4_t q3[MAXSTUDIOBONES];
	static float pos4[MAXSTUDIOBONES][3];
	static vec4_t q4[MAXSTUDIOBONES];

	if (!m_pCurrentEntity->player)
	{
		CStudioModelRenderer::StudioSetupBones();
		return;
	}

	if (m_pCurrentEntity->curstate.sequence >= m_pStudioHeader->numseq)
		m_pCurrentEntity->curstate.sequence = 0;

	pseqdesc = (mstudioseqdesc_t *)((byte *)m_pStudioHeader + m_pStudioHeader->seqindex) + m_pCurrentEntity->curstate.sequence;
	panim = StudioGetAnim(m_pRenderModel, pseqdesc);

	f = StudioEstimateFrame(pseqdesc);

	if (m_pPlayerInfo->gaitsequence == ANIM_WALK_SEQUENCE)
	{
		if (m_pCurrentEntity->curstate.blending[0] <= 26)
		{
			m_pCurrentEntity->curstate.blending[0] = 0;
			m_pCurrentEntity->latched.prevseqblending[0] = m_pCurrentEntity->curstate.blending[0];
		}
		else
		{
			m_pCurrentEntity->curstate.blending[0] -= 26;
			m_pCurrentEntity->latched.prevseqblending[0] = m_pCurrentEntity->curstate.blending[0];
		}
	}

	if (pseqdesc->numblends == 9)
	{
		float s = m_pCurrentEntity->curstate.blending[0];
		float t = m_pCurrentEntity->curstate.blending[1];

		if (s <= 127.0)
		{
			s = (s * 2.0);

			if (t <= 127.0)
			{
				t = (t * 2.0);

				StudioCalcRotations(pos, q, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 1);
				StudioCalcRotations(pos2, q2, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 3);
				StudioCalcRotations(pos3, q3, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 4);
				StudioCalcRotations(pos4, q4, pseqdesc, panim, f);
			}
			else
			{
				t = 2.0 * (t - 127.0);

				panim = LookupAnimation(pseqdesc, 3);
				StudioCalcRotations(pos, q, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 4);
				StudioCalcRotations(pos2, q2, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 6);
				StudioCalcRotations(pos3, q3, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 7);
				StudioCalcRotations(pos4, q4, pseqdesc, panim, f);
			}
		}
		else
		{
			s = 2.0 * (s - 127.0);

			if (t <= 127.0)
			{
				t = (t * 2.0);

				panim = LookupAnimation(pseqdesc, 1);
				StudioCalcRotations(pos, q, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 2);
				StudioCalcRotations(pos2, q2, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 4);
				StudioCalcRotations(pos3, q3, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 5);
				StudioCalcRotations(pos4, q4, pseqdesc, panim, f);
			}
			else
			{
				t = 2.0 * (t - 127.0);

				panim = LookupAnimation(pseqdesc, 4);
				StudioCalcRotations(pos, q, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 5);
				StudioCalcRotations(pos2, q2, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 7);
				StudioCalcRotations(pos3, q3, pseqdesc, panim, f);
				panim = LookupAnimation(pseqdesc, 8);
				StudioCalcRotations(pos4, q4, pseqdesc, panim, f);
			}
		}

		s /= 255.0;
		t /= 255.0;

		StudioSlerpBones(q, pos, q2, pos2, s);
		StudioSlerpBones(q3, pos3, q4, pos4, s);
		StudioSlerpBones(q, pos, q3, pos3, t);
	}
	else
	{
		StudioCalcRotations(pos, q, pseqdesc, panim, f);
	}

	if (m_fDoInterp && m_pCurrentEntity->latched.sequencetime && (m_pCurrentEntity->latched.sequencetime + 0.2 > m_clTime) && (m_pCurrentEntity->latched.prevsequence < m_pStudioHeader->numseq))
	{
		static float pos1b[MAXSTUDIOBONES][3];
		static vec4_t q1b[MAXSTUDIOBONES];
		float s = m_pCurrentEntity->latched.prevseqblending[0];
		float t = m_pCurrentEntity->latched.prevseqblending[1];

		pseqdesc = (mstudioseqdesc_t *)((byte *)m_pStudioHeader + m_pStudioHeader->seqindex) + m_pCurrentEntity->latched.prevsequence;
		panim = StudioGetAnim(m_pRenderModel, pseqdesc);

		if (pseqdesc->numblends == 9)
		{
			if (s <= 127.0)
			{
				s = (s * 2.0);

				if (t <= 127.0)
				{
					t = (t * 2.0);

					StudioCalcRotations(pos1b, q1b, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 1);
					StudioCalcRotations(pos2, q2, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 3);
					StudioCalcRotations(pos3, q3, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 4);
					StudioCalcRotations(pos4, q4, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
				}
				else
				{
					t = 2.0 * (t - 127.0);

					panim = LookupAnimation(pseqdesc, 3);
					StudioCalcRotations(pos1b, q1b, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 4);
					StudioCalcRotations(pos2, q2, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 6);
					StudioCalcRotations(pos3, q3, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 7);
					StudioCalcRotations(pos4, q4, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
				}
			}
			else
			{
				s = 2.0 * (s - 127.0);

				if (t <= 127.0)
				{
					t = (t * 2.0);

					panim = LookupAnimation(pseqdesc, 1);
					StudioCalcRotations(pos1b, q1b, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 2);
					StudioCalcRotations(pos2, q2, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 4);
					StudioCalcRotations(pos3, q3, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 5);
					StudioCalcRotations(pos4, q4, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
				}
				else
				{
					t = 2.0 * (t - 127.0);

					panim = LookupAnimation(pseqdesc, 4);
					StudioCalcRotations(pos1b, q1b, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 5);
					StudioCalcRotations(pos2, q2, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 7);
					StudioCalcRotations(pos3, q3, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
					panim = LookupAnimation(pseqdesc, 8);
					StudioCalcRotations(pos4, q4, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
				}
			}

			s /= 255.0;
			t /= 255.0;

			StudioSlerpBones(q1b, pos1b, q2, pos2, s);
			StudioSlerpBones(q3, pos3, q4, pos4, s);
			StudioSlerpBones(q1b, pos1b, q3, pos3, t);
		}
		else
		{
			StudioCalcRotations(pos1b, q1b, pseqdesc, panim, m_pCurrentEntity->latched.prevframe);
		}

		s = 1.0 - (m_clTime - m_pCurrentEntity->latched.sequencetime) / 0.2;
		StudioSlerpBones(q, pos, q1b, pos1b, s);
	}
	else
	{
		m_pCurrentEntity->latched.prevframe = f;
	}

	pbones = (mstudiobone_t *)((byte *)m_pStudioHeader + m_pStudioHeader->boneindex);

	if (m_pPlayerInfo && (m_pCurrentEntity->curstate.sequence < ANIM_FIRST_DEATH_SEQUENCE || m_pCurrentEntity->curstate.sequence > ANIM_LAST_DEATH_SEQUENCE) && (m_pCurrentEntity->curstate.sequence < ANIM_FIRST_EMOTION_SEQUENCE || m_pCurrentEntity->curstate.sequence > ANIM_LAST_EMOTION_SEQUENCE) && m_pCurrentEntity->curstate.sequence != ANIM_SWIM_1 && m_pCurrentEntity->curstate.sequence != ANIM_SWIM_2)
	{
		int copy = 1;

		if (m_pPlayerInfo->gaitsequence >= m_pStudioHeader->numseq)
			m_pPlayerInfo->gaitsequence = 0;

		pseqdesc = (mstudioseqdesc_t *)((byte *)m_pStudioHeader + m_pStudioHeader->seqindex ) + m_pPlayerInfo->gaitsequence;

		panim = StudioGetAnim(m_pRenderModel, pseqdesc);
		StudioCalcRotations(pos2, q2, pseqdesc, panim, m_pPlayerInfo->gaitframe);

		for (i = 0; i < m_pStudioHeader->numbones; i++)
		{
			if (!strcmp(pbones[i].name, "Bip01 Spine"))
				copy = 0;
			else if (!strcmp(pbones[pbones[i].parent].name, "Bip01 Pelvis"))
				copy = 1;

			if (copy)
			{
				memcpy(pos[i], pos2[i], sizeof(pos[i]));
				memcpy(q[i], q2[i], sizeof(q[i]));
			}
		}
	}

	for (i = 0; i < m_pStudioHeader->numbones; i++)
	{
		QuaternionMatrix(q[i], bonematrix);

		bonematrix[0][3] = pos[i][0];
		bonematrix[1][3] = pos[i][1];
		bonematrix[2][3] = pos[i][2];

		if (pbones[i].parent == -1)
		{
			if (IEngineStudio.IsHardware())
			{
				ConcatTransforms((*m_protationmatrix), bonematrix, (*m_pbonetransform)[i]);
				MatrixCopy((*m_pbonetransform)[i], (*m_plighttransform)[i]);
			}
			else
			{
				ConcatTransforms((*m_paliastransform), bonematrix, (*m_pbonetransform)[i]);
				ConcatTransforms((*m_protationmatrix), bonematrix, (*m_plighttransform)[i]);
			}

			StudioFxTransform(m_pCurrentEntity, (*m_pbonetransform)[i]);
		}
		else
		{
			ConcatTransforms((*m_pbonetransform)[pbones[i].parent], bonematrix, (*m_pbonetransform)[i]);
			ConcatTransforms((*m_plighttransform)[pbones[i].parent], bonematrix, (*m_plighttransform)[i]);
		}
	}
}

void CGameStudioModelRenderer::StudioEstimateGait(entity_state_t *pplayer)
{
	float dt;
	vec3_t est_velocity;

	dt = (m_clTime - m_clOldTime);
	dt = max(0.0, dt);
	dt = min(1.0, dt);

	if (dt == 0 || m_pPlayerInfo->renderframe == m_nFrameCount)
	{
		m_flGaitMovement = 0;
		return;
	}

	if (m_fGaitEstimation)
	{
		VectorSubtract(m_pCurrentEntity->origin, m_pPlayerInfo->prevgaitorigin, est_velocity);
		VectorCopy(m_pCurrentEntity->origin, m_pPlayerInfo->prevgaitorigin);
		m_flGaitMovement = est_velocity.Length();

		if (dt <= 0 || m_flGaitMovement / dt < 5)
		{
			m_flGaitMovement = 0;
			est_velocity[0] = 0;
			est_velocity[1] = 0;
		}
	}
	else
	{
		VectorCopy(pplayer->velocity, est_velocity);
		m_flGaitMovement = est_velocity.Length() * dt;
	}

	if (est_velocity[1] == 0 && est_velocity[0] == 0)
	{
		float flYawDiff = m_pCurrentEntity->angles[YAW] - m_pPlayerInfo->gaityaw;
		flYawDiff = flYawDiff - (int)(flYawDiff / 360) * 360;

		if (flYawDiff > 180)
			flYawDiff -= 360;

		if (flYawDiff < -180)
			flYawDiff += 360;

		if (dt < 0.25)
			flYawDiff *= dt * 4;
		else
			flYawDiff *= dt;

		m_pPlayerInfo->gaityaw += flYawDiff;
		m_pPlayerInfo->gaityaw = m_pPlayerInfo->gaityaw - (int)(m_pPlayerInfo->gaityaw / 360) * 360;

		m_flGaitMovement = 0;
	}
	else
	{
		m_pPlayerInfo->gaityaw = (atan2(est_velocity[1], est_velocity[0]) * 180 / M_PI);

		if (m_pPlayerInfo->gaityaw > 180)
			m_pPlayerInfo->gaityaw = 180;

		if (m_pPlayerInfo->gaityaw < -180)
			m_pPlayerInfo->gaityaw = -180;
	}
}

void CGameStudioModelRenderer::StudioPlayerBlend(mstudioseqdesc_t *pseqdesc, int *pBlend, float *pPitch)
{
	float range = 45.0;

	*pBlend = (*pPitch * 3);

	if (*pBlend <= -range)
		*pBlend = 255;
	else if (*pBlend >= range)
		*pBlend = 0;
	else
		*pBlend = 255 * (range - *pBlend) / (2 * range);

	*pPitch = 0;
}

void CGameStudioModelRenderer::CalculatePitchBlend(entity_state_t *pplayer)
{
	mstudioseqdesc_t *pseqdesc;
	int iBlend;

	pseqdesc = (mstudioseqdesc_t *)((byte *)m_pStudioHeader + m_pStudioHeader->seqindex) + m_pCurrentEntity->curstate.sequence;

	StudioPlayerBlend(pseqdesc, &iBlend, &m_pCurrentEntity->angles[PITCH]);

	m_pCurrentEntity->latched.prevangles[PITCH] = m_pCurrentEntity->angles[PITCH];
	m_pCurrentEntity->curstate.blending[1] = iBlend;
	m_pCurrentEntity->latched.prevblending[1] = m_pCurrentEntity->curstate.blending[1];
	m_pCurrentEntity->latched.prevseqblending[1] = m_pCurrentEntity->curstate.blending[1];
}

void CGameStudioModelRenderer::CalculateYawBlend(entity_state_t *pplayer)
{
	float flYaw;

	StudioEstimateGait(pplayer);

	flYaw = m_pCurrentEntity->angles[YAW] - m_pPlayerInfo->gaityaw;
	flYaw = fmod(flYaw, 360.0f);

	if (flYaw < -180)
		flYaw = flYaw + 360;
	else if (flYaw > 180)
		flYaw = flYaw - 360;

	float maxyaw = 120.0;

	if (flYaw > maxyaw)
	{
		m_pPlayerInfo->gaityaw = m_pPlayerInfo->gaityaw - 180;
		m_flGaitMovement = -m_flGaitMovement;
		flYaw = flYaw - 180;
	}
	else if (flYaw < -maxyaw)
	{
		m_pPlayerInfo->gaityaw = m_pPlayerInfo->gaityaw + 180;
		m_flGaitMovement = -m_flGaitMovement;
		flYaw = flYaw + 180;
	}

	float blend_yaw = (flYaw / 90.0) * 128.0 + 127.0;

	blend_yaw = 255.0 - bound( 0.0, blend_yaw, 255.0 );

	m_pCurrentEntity->curstate.blending[0] = (int)(blend_yaw);
	m_pCurrentEntity->latched.prevblending[0] = m_pCurrentEntity->curstate.blending[0];
	m_pCurrentEntity->latched.prevseqblending[0] = m_pCurrentEntity->curstate.blending[0];

	m_pCurrentEntity->angles[YAW] = m_pPlayerInfo->gaityaw;

	if (m_pCurrentEntity->angles[YAW] < -0)
		m_pCurrentEntity->angles[YAW] += 360;

	m_pCurrentEntity->latched.prevangles[YAW] = m_pCurrentEntity->angles[YAW];
}

void CGameStudioModelRenderer::StudioProcessGait(entity_state_t *pplayer)
{
	mstudioseqdesc_t *pseqdesc;

	CalculateYawBlend(pplayer);
	CalculatePitchBlend(pplayer);


	pseqdesc = (mstudioseqdesc_t *)((byte *)m_pStudioHeader + m_pStudioHeader->seqindex) + pplayer->gaitsequence;

	if (pseqdesc->linearmovement[0] > 0)
		m_pPlayerInfo->gaitframe += (m_flGaitMovement / pseqdesc->linearmovement[0]) * pseqdesc->numframes;
	else
	{
		float dt = bound( 0.0, (m_clTime - m_clOldTime), 1.0 );
		m_pPlayerInfo->gaitframe += pseqdesc->fps * dt * m_pCurrentEntity->curstate.framerate;
	}

	m_pPlayerInfo->gaitframe = m_pPlayerInfo->gaitframe - (int)(m_pPlayerInfo->gaitframe / pseqdesc->numframes) * pseqdesc->numframes;

	if (m_pPlayerInfo->gaitframe < 0)
		m_pPlayerInfo->gaitframe += pseqdesc->numframes;
}

void CGameStudioModelRenderer::SavePlayerState(entity_state_t *pplayer)
{
	client_anim_state_t *st;
	cl_entity_t *ent = IEngineStudio.GetCurrentEntity();

	if (!ent)
		return;

	st = &g_state;

	st->angles = ent->curstate.angles;
	st->origin = ent->curstate.origin;

	st->realangles = ent->angles;

	st->sequence = ent->curstate.sequence;
	st->gaitsequence = pplayer->gaitsequence;
	st->animtime = ent->curstate.animtime;
	st->frame = ent->curstate.frame;
	st->framerate = ent->curstate.framerate;

	memcpy(st->blending, ent->curstate.blending, 2);
	memcpy(st->controller, ent->curstate.controller, 4);

	st->lv = ent->latched;
}

void GetSequenceInfo(void *pmodel, client_anim_state_t *pev, float *pflFrameRate, float *pflGroundSpeed)
{
	studiohdr_t *pstudiohdr;
	pstudiohdr = (studiohdr_t *)pmodel;

	if (!pstudiohdr)
		return;

	mstudioseqdesc_t *pseqdesc;

	if (pev->sequence >= pstudiohdr->numseq)
	{
		*pflFrameRate = 0.0;
		*pflGroundSpeed = 0.0;
		return;
	}

	pseqdesc = (mstudioseqdesc_t *)((byte *)pstudiohdr + pstudiohdr->seqindex) + (int)pev->sequence;

	if (pseqdesc->numframes > 1)
	{
		*pflFrameRate = 256 * pseqdesc->fps / (pseqdesc->numframes - 1);
		*pflGroundSpeed = sqrt(pseqdesc->linearmovement[0] * pseqdesc->linearmovement[0] + pseqdesc->linearmovement[1] * pseqdesc->linearmovement[1] + pseqdesc->linearmovement[2] * pseqdesc->linearmovement[2]);
		*pflGroundSpeed = *pflGroundSpeed * pseqdesc->fps / (pseqdesc->numframes - 1);
	}
	else
	{
		*pflFrameRate = 256.0;
		*pflGroundSpeed = 0.0;
	}
}

int GetSequenceFlags(void *pmodel, client_anim_state_t *pev)
{
	studiohdr_t *pstudiohdr;
	pstudiohdr = (studiohdr_t *)pmodel;

	if (!pstudiohdr || pev->sequence >= pstudiohdr->numseq)
		return 0;

	mstudioseqdesc_t *pseqdesc;
	pseqdesc = (mstudioseqdesc_t *)((byte *)pstudiohdr + pstudiohdr->seqindex) + (int)pev->sequence;

	return pseqdesc->flags;
}

float StudioFrameAdvance(client_anim_state_t *st, float framerate, float flInterval)
{
	if (flInterval == 0.0)
	{
		flInterval = (gEngfuncs.GetClientTime() - st->animtime);

		if (flInterval <= 0.001)
		{
			st->animtime = gEngfuncs.GetClientTime();
			return 0.0;
		}
	}

	if (!st->animtime)
		flInterval = 0.0;

	st->frame += flInterval * framerate * st->framerate;
	st->animtime = gEngfuncs.GetClientTime();

	if (st->frame < 0.0 || st->frame >= 256.0)
	{
		if (st->m_fSequenceLoops)
			st->frame -= (int)(st->frame / 256.0) * 256.0;
		else
			st->frame = (st->frame < 0.0) ? 0 : 255;

		st->m_fSequenceFinished = TRUE;
	}

	return flInterval;
}

void CGameStudioModelRenderer::SetupClientAnimation(entity_state_t *pplayer)
{
	static double oldtime;
	double curtime, dt;

	client_anim_state_t *st;
	float fr, gs;

	cl_entity_t *ent = IEngineStudio.GetCurrentEntity();

	if (!ent)
		return;

	curtime = gEngfuncs.GetClientTime();
	dt = bound( 0.0, (curtime - oldtime), 1.0 );

	oldtime = curtime;
	st = &g_clientstate;

	st->framerate = 1.0;

	int oldseq = st->sequence;
	CounterStrike_GetSequence(&st->sequence, &st->gaitsequence);
	CounterStrike_GetOrientation((float *)&st->origin, (float *)&st->angles);
	VectorCopy(st->angles, st->realangles);

	if (st->sequence != oldseq)
	{
		st->frame = 0.0;
		st->lv.prevsequence = oldseq;
		st->lv.sequencetime = st->animtime;

		memcpy(st->lv.prevseqblending, st->blending, 2);
		memcpy(st->lv.prevcontroller, st->controller, 4);
	}

	void *pmodel = (studiohdr_t *)IEngineStudio.Mod_Extradata(ent->model);

	if( !pmodel )
		return;


	GetSequenceInfo(pmodel, st, &fr, &gs);
	st->m_fSequenceLoops = ((GetSequenceFlags(pmodel, st) & STUDIO_LOOPING) != 0);
	StudioFrameAdvance(st, fr, dt);

	ent->angles = st->realangles;

	ent->curstate.angles = st->angles;
	ent->curstate.origin = st->origin;

	ent->curstate.sequence = st->sequence;
	pplayer->gaitsequence = st->gaitsequence;
	ent->curstate.animtime = st->animtime;
	ent->curstate.frame = st->frame;
	ent->curstate.framerate = st->framerate;

	memcpy(ent->curstate.blending, st->blending, 2);
	memcpy(ent->curstate.controller, st->controller, 4);

	ent->latched = st->lv;
}

void CGameStudioModelRenderer::RestorePlayerState(entity_state_t *pplayer)
{
	client_anim_state_t *st;
	cl_entity_t *ent = IEngineStudio.GetCurrentEntity();

	if (!ent)
		return;

	st = &g_clientstate;

	st->angles = ent->curstate.angles;
	st->origin = ent->curstate.origin;

	st->realangles = ent->angles;

	st->sequence = ent->curstate.sequence;
	st->gaitsequence = pplayer->gaitsequence;
	st->animtime = ent->curstate.animtime;
	st->frame = ent->curstate.frame;
	st->framerate = ent->curstate.framerate;

	memcpy(st->blending, ent->curstate.blending, 2);
	memcpy(st->controller, ent->curstate.controller, 4);

	st->lv = ent->latched;

	st = &g_state;

	ent->angles = st->realangles;

	ent->curstate.angles = st->angles;
	ent->curstate.origin = st->origin;

	ent->curstate.sequence = st->sequence;
	pplayer->gaitsequence = st->gaitsequence;
	ent->curstate.animtime = st->animtime;
	ent->curstate.frame = st->frame;
	ent->curstate.framerate = st->framerate;

	memcpy(ent->curstate.blending, st->blending, 2);
	memcpy(ent->curstate.controller, st->controller, 4);

	ent->latched = st->lv;
}

int CGameStudioModelRenderer::StudioDrawPlayer(int flags, entity_state_t *pplayer)
{
	int iret = 0;
	bool isLocalPlayer = false;

	m_pplayer = pplayer;

	if (m_bLocal && IEngineStudio.GetCurrentEntity() == gEngfuncs.GetLocalPlayer())
		isLocalPlayer = true;

	if (isLocalPlayer)
	{
		SavePlayerState(pplayer);
		SetupClientAnimation(pplayer);
	}

	iret = _StudioDrawPlayer(flags, pplayer);

	if (isLocalPlayer)
		RestorePlayerState(pplayer);

	if( m_pCvarShadows->value != 0.0f )
	{
		Vector chestpos;

		for( int i = 0; i < m_nCachedBones; i++ )
		{
			if( !strcmp(m_nCachedBoneNames[i], "Bip01 Spine3") )
			{
				chestpos.x = m_rgCachedBoneTransform[i][0][3];
				chestpos.y = m_rgCachedBoneTransform[i][1][3];
				chestpos.z = m_rgCachedBoneTransform[i][2][3];
				StudioDrawShadow(chestpos, 20.0f);
				break;
			}
		}
	}

	m_pplayer = NULL;

	return iret;
}

bool WeaponHasAttachments(entity_state_t *pplayer)
{
	studiohdr_t *modelheader = NULL;
	model_t *pweaponmodel;

	if (!pplayer)
		return false;

	pweaponmodel = IEngineStudio.GetModelByIndex(pplayer->weaponmodel);
	modelheader = (studiohdr_t *)IEngineStudio.Mod_Extradata(pweaponmodel);

	if( !modelheader )
		return false;

	return (modelheader->numattachments != 0);
}

int CGameStudioModelRenderer::_StudioDrawPlayer(int flags, entity_state_t *pplayer)
{
	m_pCurrentEntity = IEngineStudio.GetCurrentEntity();

	IEngineStudio.GetTimes(&m_nFrameCount, &m_clTime, &m_clOldTime);
	IEngineStudio.GetViewInfo(m_vRenderOrigin, m_vUp, m_vRight, m_vNormal);
	IEngineStudio.GetAliasScale(&m_fSoftwareXScale, &m_fSoftwareYScale);

	m_nPlayerIndex = pplayer->number - 1;

	if (m_nPlayerIndex < 0 || m_nPlayerIndex >= gEngfuncs.GetMaxClients())
		return 0;

	/*m_pRenderModel = IEngineStudio.SetupPlayerModel(m_nPlayerIndex);

	if (m_pRenderModel == NULL)
		return 0;*/

	extra_player_info_t *pExtra = g_PlayerExtraInfo + pplayer->number;

	if( gHUD.cl_minmodels && gHUD.cl_minmodels->value )
	{
		int team = pExtra->teamnumber;
		if( team == TEAM_TERRORIST )
		{
			// set leet if model isn't valid
			int modelIdx = gHUD.cl_min_t && BIsValidTModelIndex(gHUD.cl_min_t->value) ? gHUD.cl_min_t->value : 1;

			m_pRenderModel = gEngfuncs.CL_LoadModel( sPlayerModelFiles[ modelIdx ], NULL );
		}
		else if( team == TEAM_CT )
		{
			if( pExtra->vip )
				m_pRenderModel = gEngfuncs.CL_LoadModel( sPlayerModelFiles[3], NULL );
			else
			{
				// set gign, if model isn't valud
				int modelIdx = gHUD.cl_min_ct && BIsValidCTModelIndex(gHUD.cl_min_ct->value) ? gHUD.cl_min_ct->value : 2;

				m_pRenderModel = gEngfuncs.CL_LoadModel( sPlayerModelFiles[ modelIdx ], NULL );
			}
		}
	}
	else
	{
		m_pRenderModel = IEngineStudio.SetupPlayerModel( m_nPlayerIndex );
	}

	if( !m_pRenderModel )
	{
		return 0;
	}

	m_pStudioHeader = (studiohdr_t *)IEngineStudio.Mod_Extradata(m_pRenderModel);

	if( !m_pStudioHeader )
		return 0;

	IEngineStudio.StudioSetHeader(m_pStudioHeader);
	IEngineStudio.SetRenderModel(m_pRenderModel);

	if (m_pCurrentEntity->curstate.sequence >= m_pStudioHeader->numseq)
		m_pCurrentEntity->curstate.sequence = 0;

	if (pplayer->sequence >= m_pStudioHeader->numseq)
		pplayer->sequence = 0;

	if (m_pCurrentEntity->curstate.gaitsequence >= m_pStudioHeader->numseq)
		m_pCurrentEntity->curstate.gaitsequence = 0;

	if (pplayer->gaitsequence >= m_pStudioHeader->numseq)
		pplayer->gaitsequence = 0;

	if (pplayer->gaitsequence)
	{
		vec3_t orig_angles(m_pCurrentEntity->angles);
		m_pPlayerInfo = IEngineStudio.PlayerInfo(m_nPlayerIndex);

		StudioProcessGait(pplayer);

		m_pPlayerInfo->gaitsequence = pplayer->gaitsequence;
		m_pPlayerInfo = NULL;

		StudioSetUpTransform(0);
		m_pCurrentEntity->angles = orig_angles;
	}
	else
	{
		m_pCurrentEntity->curstate.controller[0] = 127;
		m_pCurrentEntity->curstate.controller[1] = 127;
		m_pCurrentEntity->curstate.controller[2] = 127;
		m_pCurrentEntity->curstate.controller[3] = 127;
		m_pCurrentEntity->latched.prevcontroller[0] = m_pCurrentEntity->curstate.controller[0];
		m_pCurrentEntity->latched.prevcontroller[1] = m_pCurrentEntity->curstate.controller[1];
		m_pCurrentEntity->latched.prevcontroller[2] = m_pCurrentEntity->curstate.controller[2];
		m_pCurrentEntity->latched.prevcontroller[3] = m_pCurrentEntity->curstate.controller[3];

		m_pPlayerInfo = IEngineStudio.PlayerInfo(m_nPlayerIndex);

		CalculatePitchBlend(pplayer);
		CalculateYawBlend(pplayer);

		m_pPlayerInfo->gaitsequence = 0;
		StudioSetUpTransform(0);
	}

	if (flags & STUDIO_RENDER)
	{
		(*m_pModelsDrawn)++;
		(*m_pStudioModelCount)++;

		if (m_pStudioHeader->numbodyparts == 0)
			return 1;
	}

	m_pPlayerInfo = IEngineStudio.PlayerInfo(m_nPlayerIndex);

	StudioSetupBones();
	StudioSaveBones();

	m_pPlayerInfo->renderframe = m_nFrameCount;
	m_pPlayerInfo = NULL;

	if (flags & STUDIO_EVENTS && (!(flags & STUDIO_RENDER) || !pplayer->weaponmodel || !WeaponHasAttachments(pplayer)))
	{
		StudioCalcAttachments();
		IEngineStudio.StudioClientEvents();

		if (m_pCurrentEntity->index > 0)
		{
			cl_entity_t *ent = gEngfuncs.GetEntityByIndex(m_pCurrentEntity->index);
			memcpy(ent->attachment, m_pCurrentEntity->attachment, sizeof(vec3_t) * 4);
		}
	}

	if (flags & STUDIO_RENDER)
	{
		alight_t lighting;
		vec3_t dir;

		lighting.plightvec = dir;

		IEngineStudio.StudioDynamicLight(m_pCurrentEntity, &lighting);
		IEngineStudio.StudioEntityLight(&lighting);
		IEngineStudio.StudioSetupLighting(&lighting);

		m_pPlayerInfo = IEngineStudio.PlayerInfo(m_nPlayerIndex);
		m_nTopColor = m_pPlayerInfo->topcolor;

		if (m_nTopColor < 0)
			m_nTopColor = 0;

		if (m_nTopColor > 360)
			m_nTopColor = 360;

		m_nBottomColor = m_pPlayerInfo->bottomcolor;

		if (m_nBottomColor < 0)
			m_nBottomColor = 0;

		if (m_nBottomColor > 360)
			m_nBottomColor = 360;

		IEngineStudio.StudioSetRemapColors(m_nTopColor, m_nBottomColor);

		StudioRenderModel(dir);
		m_pPlayerInfo = NULL;

		if (pplayer->weaponmodel)
		{
			studiohdr_t *saveheader = m_pStudioHeader;
			cl_entity_t saveent = *m_pCurrentEntity;

			model_t *pweaponmodel = IEngineStudio.GetModelByIndex(pplayer->weaponmodel);

			m_pStudioHeader = (studiohdr_t *)IEngineStudio.Mod_Extradata(pweaponmodel);
			if( !m_pStudioHeader )
				return 0;

			IEngineStudio.StudioSetHeader(m_pStudioHeader);

			StudioMergeBones(pweaponmodel);

			IEngineStudio.StudioSetupLighting(&lighting);

			StudioRenderModel(dir);

			StudioCalcAttachments();

			if (m_pCurrentEntity->index > 0)
				memcpy(saveent.attachment, m_pCurrentEntity->attachment, sizeof(vec3_t) * m_pStudioHeader->numattachments);

			*m_pCurrentEntity = saveent;
			m_pStudioHeader = saveheader;
			IEngineStudio.StudioSetHeader(m_pStudioHeader);

			if (flags & STUDIO_EVENTS)
				IEngineStudio.StudioClientEvents();
		}
	}

	return 1;
}


void CGameStudioModelRenderer::StudioFxTransform(cl_entity_t *ent, float transform[3][4])
{
	switch (ent->curstate.renderfx)
	{
	case kRenderFxDistort:
	case kRenderFxHologram:
	{
		if (Com_RandomLong(0, 49) == 0)
		{
			int axis = Com_RandomLong(0, 1);

			if (axis == 1)
				axis = 2;

			VectorScale( transform[axis], gEngfuncs.pfnRandomFloat( 1, 1.484 ), transform[axis] );
		}
		else if (Com_RandomLong(0, 49) == 0)
		{
			float offset;

			offset = gEngfuncs.pfnRandomFloat(-10, 10);
			transform[Com_RandomLong(0, 2)][3] += offset;
		}

		break;
	}

	case kRenderFxExplode:
	{
		if (iRenderStateChanged)
		{
			g_flStartScaleTime = m_clTime;
			iRenderStateChanged = FALSE;
		}

		float flTimeDelta = m_clTime - g_flStartScaleTime;

		if (flTimeDelta > 0)
		{
			float flScale = 0.001;

			if (flTimeDelta <= 2.0)
				flScale = 1.0 - (flTimeDelta / 2.0);

			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
					transform[i][j] *= flScale;
			}
		}

		break;
	}
	}
}

void R_StudioInit(void)
{
	g_StudioRenderer.Init();
}

int R_StudioDrawPlayer(int flags, entity_state_t *pplayer)
{
#ifndef NDEBUG
	if( g_StudioRenderer.m_pCvarDebug->value >= 8 )
	{
			cl_entity_t *pCurrentEntity = IEngineStudio.GetCurrentEntity();
			int ret = 0;

			if( pCurrentEntity )
			{
					cvar_t *drawEntites = g_StudioRenderer.m_pCvarDrawEntities;
					g_StudioRenderer.m_pCvarDebug->value -= 8;
					g_StudioRenderer.m_pCvarDrawEntities = g_StudioRenderer.m_pCvarDebug;

					// first draw interpolated
					ret = g_StudioRenderer.StudioDrawPlayer( flags, pplayer );

					// then draw non-interpolated
					/*{
						Vector saveOrigin = pCurrentEntity->origin;
						int savefx = pCurrentEntity->curstate.renderfx;
						int saveamt = pCurrentEntity->curstate.renderamt;
						color24 savecolor = pCurrentEntity->curstate.rendercolor;

						pCurrentEntity->curstate.renderfx = kRenderFxGlowShell;
						pCurrentEntity->curstate.rendercolor.r = 0;
						pCurrentEntity->curstate.rendercolor.g = 0;
						pCurrentEntity->curstate.rendercolor.b = 255;
						pCurrentEntity->curstate.renderamt = 255;
						pCurrentEntity->origin = pCurrentEntity->curstate.origin;

						g_StudioRenderer.StudioDrawPlayer( flags, pplayer );
						pCurrentEntity->origin = saveOrigin;
						pCurrentEntity->curstate.renderfx = savefx;
						pCurrentEntity->curstate.renderamt   = saveamt;
						pCurrentEntity->curstate.rendercolor = savecolor;
					}

					// then draw non-interpolated
					{
						Vector saveOrigin = pCurrentEntity->origin;
						int savefx = pCurrentEntity->curstate.renderfx;
						int saveamt = pCurrentEntity->curstate.renderamt;
						color24 savecolor = pCurrentEntity->curstate.rendercolor;

						pCurrentEntity->curstate.renderfx = kRenderFxGlowShell;
						pCurrentEntity->curstate.rendercolor.r = 255;
						pCurrentEntity->curstate.rendercolor.g = 0;
						pCurrentEntity->curstate.rendercolor.b = 0;
						pCurrentEntity->curstate.renderamt = 255;
						pCurrentEntity->origin = pCurrentEntity->prevstate.origin;

						g_StudioRenderer.StudioDrawPlayer( flags, pplayer );
						pCurrentEntity->origin = saveOrigin;
						pCurrentEntity->curstate.renderfx    = savefx;
						pCurrentEntity->curstate.renderamt   = saveamt;
						pCurrentEntity->curstate.rendercolor = savecolor;
					}*/


					g_StudioRenderer.m_pCvarDrawEntities = drawEntites;
					g_StudioRenderer.m_pCvarDebug->value += 8;
			}

			return ret;
	}
	else
#endif
	return g_StudioRenderer.StudioDrawPlayer(flags, pplayer);
}

int R_StudioDrawModel(int flags)
{
	return g_StudioRenderer.StudioDrawModel(flags);
}
// The simple drawing interface we'll pass back to the engine
r_studio_interface_t studio =
{
	STUDIO_INTERFACE_VERSION,
	R_StudioDrawModel,
	R_StudioDrawPlayer,
};

/*
====================
HUD_GetStudioModelInterface
Export this function for the engine to use the studio renderer class to render objects.
====================
*/
int DLLEXPORT HUD_GetStudioModelInterface( int version, struct r_studio_interface_s **ppinterface, struct engine_studio_api_s *pstudio )
{
	if ( version != STUDIO_INTERFACE_VERSION )
		return 0;

	// Point the engine to our callbacks
	*ppinterface = &studio;

	// Copy in engine helper functions
	IEngineStudio = *pstudio;

	// Initialize local variables, etc.
	R_StudioInit();

	// Success
	return 1;
}

