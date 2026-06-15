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
#pragma once
#ifndef STUDIOMODELRENDERER_H
#define STUDIOMODELRENDERER_H

#include "studio.h"
#include "com_model.h"

class CStudioModelRenderer
{
public:
	CStudioModelRenderer(void);
	virtual ~CStudioModelRenderer(void);

public:
	virtual void Init(void);
	virtual int StudioDrawModel(int flags);
	virtual int StudioDrawPlayer(int flags, struct entity_state_s *pplayer);

public:
	virtual mstudioanim_t *StudioGetAnim(model_t *pSubModel, mstudioseqdesc_t *pseqdesc);
	virtual void StudioSetUpTransform(int trivial_accept);
	virtual void StudioSetupBones(void);
	virtual void StudioCalcAttachments(void);
	virtual void StudioSaveBones(void);
	virtual void StudioMergeBones(model_t *pSubModel);
	virtual float StudioEstimateInterpolant(void);
	virtual float StudioEstimateFrame(mstudioseqdesc_t *pseqdesc);
	virtual void StudioFxTransform(cl_entity_t *ent, float transform[3][4]);
	virtual void StudioSlerpBones(vec4_t q1[], float pos1[][3], vec4_t q2[], float pos2[][3], float s);
	virtual void StudioCalcBoneAdj(float dadt, float *adj, const byte *pcontroller1, const byte *pcontroller2, byte mouthopen);
	virtual void StudioCalcBoneQuaterion(int frame, float s, mstudiobone_t *pbone, mstudioanim_t *panim, float *adj, float *q);
	virtual void StudioCalcBonePosition(int frame, float s, mstudiobone_t *pbone, mstudioanim_t *panim, float *adj, float *pos);
	virtual void StudioCalcRotations(float pos[][3], vec4_t *q, mstudioseqdesc_t *pseqdesc, mstudioanim_t *panim, float f);
	virtual void StudioRenderModel(float *lightdir);
	virtual void StudioRenderFinal(void);
	virtual void StudioRenderFinal_Software(void);
	virtual void StudioRenderFinal_Hardware(void);
	virtual void StudioPlayerBlend(mstudioseqdesc_t *pseqdesc, int *pBlend, float *pPitch);
	virtual void StudioEstimateGait(entity_state_t *pplayer);
	virtual void StudioProcessGait(entity_state_t *pplayer);
	virtual void StudioSetShadowSprite(int idx);
	void StudioDrawShadow(Vector origin, float scale);


public:
	double m_clTime;
	double m_clOldTime;
	int m_fDoInterp;
	int m_iShadowSprite;
	int m_fGaitEstimation;
	int m_nFrameCount;
	cvar_t *m_pCvarHiModels;
	cvar_t *m_pCvarDeveloper;
	cvar_t *m_pCvarDrawEntities;
	cvar_t *m_pCvarShadows;
	cvar_t *m_pCvarDebug;
	cl_entity_t *m_pCurrentEntity;
	model_t *m_pRenderModel;
	player_info_t *m_pPlayerInfo;
	int m_nPlayerIndex;
	float m_flGaitMovement;
	studiohdr_t *m_pStudioHeader;
	mstudiobodyparts_t *m_pBodyPart;
	mstudiomodel_t *m_pSubModel;
	int m_nTopColor;
	int m_nBottomColor;
	model_t *m_pChromeSprite;
	int m_nCachedBones;
	char m_nCachedBoneNames[MAXSTUDIOBONES][32];
	float m_rgCachedBoneTransform[MAXSTUDIOBONES][3][4];
	float m_rgCachedLightTransform[MAXSTUDIOBONES][3][4];
	float m_fSoftwareXScale, m_fSoftwareYScale;
	float m_vUp[3];
	float m_vRight[3];
	float m_vNormal[3];
	float m_vRenderOrigin[3];
	int *m_pStudioModelCount;
	int *m_pModelsDrawn;
	float (*m_protationmatrix)[3][4];
	float (*m_paliastransform)[3][4];
	float (*m_pbonetransform)[MAXSTUDIOBONES][3][4];
	float (*m_plighttransform)[MAXSTUDIOBONES][3][4];
	entity_state_t *m_pplayer;
};

//ragdoll stuff start 
#include "btBulletDynamicsCommon.h"

#include <vector>
#include <memory>
#include <unordered_map>

static constexpr float GU_TO_M  = 1.0f / 52.0f;
static constexpr float M_TO_GU  = 52.0f;

enum ERagdollBody {
	RB_PELVIS = 0,
	RB_SPINE,
	RB_SPINE1,
	RB_HEAD,
	RB_UPPER_ARM_L,
	RB_FOREARM_L,
	RB_UPPER_ARM_R,
	RB_FOREARM_R,
	RB_THIGH_L,
	RB_CALF_L,
	RB_THIGH_R,
	RB_CALF_R,
	RB_FOOT_L,
	RB_FOOT_R,
	RB_COUNT
};

struct RagdollBoneDesc {
	ERagdollBody slot;
	const char *boneName;	 //"near" bone — owns this body, drives boneToSlot
	const char *pboneName; //"far" bone  — capsule axis points from near→far
	ERagdollBody parentSlot; //RB_COUNT means root (no parent constraint)
	float mass;
	float radius; //capsule radius, GoldSrc units
	float pboneoffset; //0=at boneName origin, 1=at pboneName origin
};

struct RagdollBody
{
	btRigidBody *body = nullptr;
	btCollisionShape *shape = nullptr;
	btDefaultMotionState *motionState = nullptr;
	int boneIndex = -1; //"near" bone index in studiohdr
	int pboneIndex = -1; //"far"  bone index in studiohdr
	ERagdollBody slot;
};

struct Ragdoll
{
	RagdollBody parts[RB_COUNT];
	btTypedConstraint *constraints[32] = {};
	int constraintCount = 0;
	bool active = false;
	float deathTime = 0.0f;
	int boneToSlot[128];
	btTransform boneLocalToBody[128];
	bool playerWasInside = false;
	float spawnTime;
};

class CRagdollWorld
{
public:
	static CRagdollWorld &Get();

	void Init();
	void Shutdown();
	void Step(float dt);

	void EnsureWorldCollision();

	btDiscreteDynamicsWorld *GetWorld() { return m_world; }

	void NotifyMapChanged();
private:
	btDefaultCollisionConfiguration	 *m_config = nullptr;
	btCollisionDispatcher *m_dispatcher = nullptr;
	btDbvtBroadphase *m_broadphase = nullptr;
	btSequentialImpulseConstraintSolver *m_solver = nullptr;
	btDiscreteDynamicsWorld *m_world = nullptr;

	btTriangleMesh	 *m_worldMesh	= nullptr;
	btBvhTriangleMeshShape *m_worldShape = nullptr;
	btRigidBody *m_worldBody = nullptr;
	char m_currentMapName[256] = {};
};

class CRagdollManager
{
public:
	static CRagdollManager &Get();

	void Update(float dt);

	void SpawnRagdoll(int entityIndex, studiohdr_t *hdr, float bonetransform[][3][4], const float *velocity);

	bool GetRagdollBones(int entityIndex, studiohdr_t *hdr, float bonetransform[][3][4]);

	Vector GetRagdollOrigin(int entityIndex);

	void RemoveRagdoll(int entityIndex);
	bool HasRagdoll(int entityIndex) const;
	void RemoveAllRagdolls();

	void ApplyImpulse(int entityIndex, const Vector &hitPos, const Vector &dir, float strength);
	void PushFromPlayers();

	void BuildConstraints(Ragdoll &rd, btDiscreteDynamicsWorld *world);

	std::unordered_map<int, Ragdoll> m_ragdolls;

	float GetRagdollSpawnTime( int playerIndex )
	{
		auto it = m_ragdolls.find( playerIndex );
		if ( it == m_ragdolls.end() ) return 0.0f;
		return it->second.spawnTime;
	}
};

inline btTransform BoneMatToBt(const float mat[3][4])
{
	btMatrix3x3 rot(
		mat[0][0], mat[0][1], mat[0][2],
		mat[1][0], mat[1][1], mat[1][2],
		mat[2][0], mat[2][1], mat[2][2]
	);
	btVector3 org(
		mat[0][3] * GU_TO_M,
		mat[1][3] * GU_TO_M,
		mat[2][3] * GU_TO_M
	);
	return btTransform(rot, org);
}

inline void BtToBoneMat(const btTransform &t, float mat[3][4])
{
	const btMatrix3x3 &r = t.getBasis();
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mat[i][j] = r[i][j];
	mat[0][3] = t.getOrigin().x() * M_TO_GU;
	mat[1][3] = t.getOrigin().y() * M_TO_GU;
	mat[2][3] = t.getOrigin().z() * M_TO_GU;
}

inline btTransform BuildCapsuleTransform(
	const float      boneMatNear[3][4],
	const btVector3 &posNear,
	const btVector3 &posFar,
	float            offset,
	float           &outHalfH)
{
	float segLen = (posFar - posNear).length();

	if (segLen > 1e-4f)
	{
		outHalfH = segLen * 0.5f;
	}
	else
	{
		outHalfH = 5.0f * GU_TO_M;
	}

	btVector3 centre = posNear.lerp(posFar, offset);

	btMatrix3x3 rot(
		boneMatNear[0][0], boneMatNear[0][1], boneMatNear[0][2],
		boneMatNear[1][0], boneMatNear[1][1], boneMatNear[1][2],
		boneMatNear[2][0], boneMatNear[2][1], boneMatNear[2][2]
	);

	return btTransform(rot, centre);
}

inline bool IsDyingSequence(studiohdr_t *hdr, int sequence)
{
	return sequence >= 101 && sequence <= 159;
}

//ragdoll stuff end
#endif
