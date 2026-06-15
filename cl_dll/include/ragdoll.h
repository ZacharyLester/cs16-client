#pragma once
#ifndef RAGDOLL_H
#define RAGDOLL_H

#include "studio.h"
#include "com_model.h"

//ragdoll stuff start 
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

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
	btCollisionShape	*m_worldShape = nullptr;
	btRigidBody *m_worldBody = nullptr;
	char m_currentMapName[256] = {};

public:
	bool m_studioReady = false;
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
