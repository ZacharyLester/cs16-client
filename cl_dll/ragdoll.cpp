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

#include "ragdoll.h"

extern engine_studio_api_t IEngineStudio;

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
	if (m_world) return;

	m_config     = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_config);
	m_broadphase = new btDbvtBroadphase();
	m_solver     = new btSequentialImpulseConstraintSolver();
	m_world      = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_config);
	m_world->setGravity(btVector3(0, 0, -800.0f * GU_TO_M));

	m_world->getSolverInfo().m_numIterations       = 20;
	m_world->getSolverInfo().m_erp                 = 0.8f;
	m_world->getSolverInfo().m_erp2                = 0.8f;
	m_world->getSolverInfo().m_splitImpulse        = true;
	m_world->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.002f;
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
	EnsureWorldCollision();
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

#ifndef SURF_PLANEBACK
#define SURF_PLANEBACK		0x02
#endif
#ifndef SURF_DRAWSKY
#define SURF_DRAWSKY		0x04
#endif
#ifndef SURF_DRAWTURB
#define SURF_DRAWTURB		0x10
#endif
#ifndef SURF_DRAWBACKGROUND
#define SURF_DRAWBACKGROUND	0x40
#endif
#ifndef SURF_UNDERWATER
#define SURF_UNDERWATER		0x80
#endif

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

	if (!worldModel->surfaces || !worldModel->surfedges ||
	    !worldModel->edges    || !worldModel->vertexes)
		return;

	m_worldMesh = new btTriangleMesh(true);

	int triCount = 0;

	for (int i = 0; i < worldModel->numsurfaces; i++)
	{
		msurface_t *surf = worldModel->surfaces + i;

		if (surf->flags & (SURF_DRAWTURB | SURF_DRAWSKY))
			continue;

		int firstedge = surf->firstedge;
		int numedges  = surf->numedges;

		if (numedges < 3)
			continue;

		if (firstedge < 0 || firstedge >= worldModel->numsurfedges)
			continue;
		if (firstedge + numedges > worldModel->numsurfedges)
			numedges = worldModel->numsurfedges - firstedge;
		if (numedges < 3)
			continue;

		auto GetSurfVert = [&](int edge_index) -> float*
		{
			int e    = worldModel->surfedges[firstedge + edge_index];
			int eIdx = (e >= 0) ? e : -e;

			if (eIdx >= worldModel->numedges)
				return nullptr;

			medge_t *edge    = &worldModel->edges[eIdx];
			int      vertIdx = (e >= 0) ? edge->v[0] : edge->v[1];

			if (vertIdx >= worldModel->numvertexes)
				return nullptr;

			return worldModel->vertexes[vertIdx].position;
		};

		float *v0 = GetSurfVert(0);
		if (!v0) continue;

		for (int j = 1; j < numedges - 1; j++)
		{
			float *v1 = GetSurfVert(j);
			float *v2 = GetSurfVert(j + 1);

			if (!v1 || !v2) continue;

			btVector3 p0(v0[0] * GU_TO_M, v0[1] * GU_TO_M, v0[2] * GU_TO_M);
			btVector3 p1(v1[0] * GU_TO_M, v1[1] * GU_TO_M, v1[2] * GU_TO_M);
			btVector3 p2(v2[0] * GU_TO_M, v2[1] * GU_TO_M, v2[2] * GU_TO_M);

			m_worldMesh->addTriangle(p0, p1, p2, false);
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

	m_worldShape = new btBvhTriangleMeshShape(m_worldMesh, true);
	m_worldShape->setMargin(0.01f);

	gEngfuncs.Con_Printf("ragdoll BSP: %d surfaces -> %d triangles\n",
		worldModel->numsurfaces, triCount);

	btRigidBody::btRigidBodyConstructionInfo ci(
		0,
		new btDefaultMotionState(),
		m_worldShape
	);
	ci.m_friction    = 0.8f;
	ci.m_restitution = 0.0f;

	m_worldBody = new btRigidBody(ci);
	m_worldBody->setCollisionFlags(
		m_worldBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT
	);

	m_world->addRigidBody(m_worldBody, 1, 2);
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

	for (auto it = m_ragdolls.begin(); it != m_ragdolls.end(); ++it)
	{
		int ragIdx    = it->first;
		Ragdoll &rd   = it->second;

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

void CRagdollManager::SpawnRagdoll(int entityIndex, studiohdr_t *hdr, float bonetransform[][3][4], const float *velocity)
{
	if (!hdr)
		return;

	CRagdollWorld::Get().Init();
	//CRagdollWorld::Get().EnsureWorldCollision();

	btDiscreteDynamicsWorld *world = CRagdollWorld::Get().GetWorld();

	RemoveRagdoll(entityIndex);

	Ragdoll &rd  = m_ragdolls[entityIndex];
	rd.active    = true;
	rd.deathTime = gEngfuncs.GetClientTime();
	rd.spawnTime = gEngfuncs.GetClientTime();

	mstudiobone_t *pbones = (mstudiobone_t *)((byte *)hdr + hdr->boneindex);

	for (int i = 0; i < 128; i++)
		rd.boneToSlot[i] = -1;

	//pass 1: find bone indices for each slot
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

	//pass 2: map every bone to its nearest owning rigid body slot
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

	//pass 3: create bodies but do NOT add to world yet
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
		part.shape->setMargin(0.01f);
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
		part.body->setCcdMotionThreshold(1.0f * GU_TO_M);
		part.body->setCcdSweptSphereRadius(desc.radius * GU_TO_M * 0.5f);
	}

	//pass 4: record bone local offsets from owning body
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
		btTransform local     = bodyWorld.inverseTimes(boneWorld);

		if (slot == RB_CALF_L || slot == RB_CALF_R)
		{
			btVector3 localOrigin = local.getOrigin();
			if (localOrigin.y() < 0)
				localOrigin.setY(localOrigin.y() * 0.5f);
			local.setOrigin(localOrigin);
		}

		rd.boneLocalToBody[b] = local;
	}

	//pass 5: ground clamp before adding to world
	{
		btVector3 pelvisPos(0, 0, 0);
		if (rd.parts[RB_PELVIS].body)
			pelvisPos = rd.parts[RB_PELVIS].body->getWorldTransform().getOrigin();

		float ox = pelvisPos.x() * M_TO_GU;
		float oy = pelvisPos.y() * M_TO_GU;
		float oz = pelvisPos.z() * M_TO_GU;

		bool wasCrouching = false;
		cl_entity_t *ent = gEngfuncs.GetEntityByIndex(entityIndex);
		if (ent)
		{
			float heightDiff = oz - ent->curstate.origin[2];
			wasCrouching = (heightDiff < 30.0f);
		}

		float traceStartZ    = wasCrouching ? oz + 36.0f : oz;
		float traceStart[3]  = { ox, oy, traceStartZ };
		float traceEnd[3]    = { ox, oy, traceStartZ - 512.0f };

		pmtrace_t tr;
		gEngfuncs.pEventAPI->EV_SetUpPlayerPrediction(false, true);
		gEngfuncs.pEventAPI->EV_PushPMStates();
		gEngfuncs.pEventAPI->EV_SetSolidPlayers(-1);
		gEngfuncs.pEventAPI->EV_SetTraceHull(2);
		gEngfuncs.pEventAPI->EV_PlayerTrace(traceStart, traceEnd,
			PM_STUDIO_IGNORE | PM_GLASS_IGNORE, entityIndex, &tr);
		gEngfuncs.pEventAPI->EV_PopPMStates();

		float floorZ  = (!tr.startsolid && tr.fraction < 1.0f) ? tr.endpos[2] : oz - 36.0f;
		float floorBt = floorZ * GU_TO_M;

		for (int slot = 0; slot < RB_COUNT; slot++)
		{
			btRigidBody *body = rd.parts[slot].body;
			if (!body) continue;

			btCapsuleShape *capsule = (btCapsuleShape *)rd.parts[slot].shape;
			float r  = capsule->getRadius();
			float hh = capsule->getHalfHeight();

			btTransform xform = body->getWorldTransform();
			btVector3   org   = xform.getOrigin();

			btVector3 worldTop    = xform * btVector3(0,  hh, 0);
			btVector3 worldBottom = xform * btVector3(0, -hh, 0);
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

	//pass 6: now add all bodies to world after they are correctly positioned
	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		if (rd.parts[slot].body)
			world->addRigidBody(rd.parts[slot].body, 2, 1);
	}

	//pass 7: constraints
	BuildConstraints(rd, world);

	world->performDiscreteCollisionDetection();

	//pass 8: death impulse
	if (velocity && rd.parts[RB_SPINE1].body)
	{
		btVector3 hitDir(
			velocity[0] * GU_TO_M * 5.0f,
			velocity[1] * GU_TO_M * 5.0f,
			velocity[2] * GU_TO_M * 5.0f
		);
		rd.parts[RB_SPINE1].body->applyImpulse(hitDir, btVector3(0, 0, 0));
	}

	//pass 9: random angular velocity for natural tumble
	srand(entityIndex ^ (int)(gEngfuncs.GetClientTime() * 1000));
	for (int slot = 0; slot < RB_COUNT; slot++)
	{
		if (!rd.parts[slot].body) continue;

		float dist  = (rd.parts[slot].body->getWorldTransform().getOrigin()
		               - rd.parts[RB_SPINE1].body->getWorldTransform().getOrigin()).length();
		float scale = 1.0f / (1.0f + dist * 5.0f);

		rd.parts[slot].body->setAngularVelocity(btVector3(
			((rand() % 200) - 100) * 0.01f * scale,
			((rand() % 200) - 100) * 0.01f * scale,
			((rand() % 200) - 100) * 0.01f * scale
		));
	}

	cl_entity_t *ent = gEngfuncs.GetEntityByIndex(entityIndex + 1);
	if (ent)
	{
		ent->curstate.colormap = 0;
		memset(ent->curstate.controller, 0, sizeof(ent->curstate.controller));
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
