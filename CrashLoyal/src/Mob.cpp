#define _CRT_SECURE_NO_WARNINGS
#include "Mob.h"
#include "Mob_River.h"

#include <memory>
#include <limits>
#include <stdlib.h>
#include <stdio.h>
#include "Building.h"
#include "Waypoint.h"
#include "GameState.h"
#include "Point.h"


#include <stdint.h>
#include <string>
#if defined(_WIN32)
#include <objbase.h>
#endif


const int GUID_LEN = 64;


std::string Mob::previousUUID = "";


Mob::Mob()
	: pos(-10000.f, -10000.f)
	, nextWaypoint(NULL)
	, targetPosition(new Point)
	, state(MobState::Moving)
	, uuid(Mob::previousUUID)
	, attackingNorth(true)
	, health(-1)
	, targetLocked(false)
	, target(NULL)
	, lastAttackTime(0)
{
}

void Mob::Init(const Point& pos, bool attackingNorth)
{
	this->lastPosition = this->getPosition();
	health = GetMaxHealth();
	this->pos = pos;
	this->attackingNorth = attackingNorth;
	std::string str = uuidGenerator();
	this->uuid = str;
	findClosestWaypoint();
}

std::string Mob::uuidGenerator() {

	char buf[GUID_LEN] = { 0 };
	GUID guid;

	if (CoCreateGuid(&guid))
	{
		return std::move(std::string(""));
	}

	sprintf(buf,
		"%08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X",
		guid.Data1, guid.Data2, guid.Data3,
		guid.Data4[0], guid.Data4[1], guid.Data4[2],
		guid.Data4[3], guid.Data4[4], guid.Data4[5],
		guid.Data4[6], guid.Data4[7]);

	return std::move(std::string(buf));

}

std::shared_ptr<Point> Mob::getPosition() {
	return std::make_shared<Point>(this->pos);
 }

bool Mob::findClosestWaypoint() {
	std::shared_ptr<Waypoint> closestWP = GameState::waypoints[0];
#ifdef max
#undef max
#endif
	float smallestDist = std::numeric_limits<float>::max();

	for (std::shared_ptr<Waypoint> wp : GameState::waypoints) {
		//std::shared_ptr<Waypoint> wp = GameState::waypoints[i];
		// Filter out any waypoints that are "behind" us (behind is relative to attack dir
		// Remember y=0 is in the top left
		if (attackingNorth && wp->pos.y > this->pos.y) {
			continue;
		}
		else if ((!attackingNorth) && wp->pos.y < this->pos.y) {
			continue;
		}

		float dist = this->pos.dist(wp->pos);
		if (dist < smallestDist) {
			smallestDist = dist;
			closestWP = wp;
		}
	}
	std::shared_ptr<Point> newTarget = std::shared_ptr<Point>(new Point);
	this->targetPosition->x = closestWP->pos.x;
    this->targetPosition->y = closestWP->pos.y;
	this->nextWaypoint = closestWP;

	return true;
}

void Mob::moveTowards(std::shared_ptr<Point> moveTarget, double elapsedTime) {
	this->lastPosition = this->getPosition();
	Point movementVector;
	movementVector.x = moveTarget->x - this->pos.x;
	movementVector.y = moveTarget->y - this->pos.y;
	movementVector.normalize();
	movementVector *= (float)this->GetSpeed();
	movementVector *= (float)elapsedTime;
	pos += movementVector;
}

void Mob::moveTowardsVector(std::shared_ptr<Point> vector, double elapsedTime) {
	this->lastPosition = this->getPosition();
	Point movementVector;
	movementVector.x = vector->x;
	movementVector.y = vector->y;
	movementVector.normalize();
	movementVector *= (float)this->GetSpeed();
	movementVector *= (float)elapsedTime;
	this->pos += movementVector;
}

void Mob::findNewTarget() {
	// Find a new valid target to move towards and update this mob
	// to start pathing towards it

	if (!findAndSetAttackableMob()) { findClosestWaypoint(); }
}

// Have this mob start aiming towards the provided target
// TODO: impliment true pathfinding here
void Mob::updateMoveTarget(std::shared_ptr<Point> target) {
	this->targetPosition->x = target->x;
	this->targetPosition->y = target->y;
}

void Mob::updateMoveTarget(Point target) {
	this->targetPosition->x = target.x;
	this->targetPosition->y = target.y;
}


// Movement related
//////////////////////////////////
// Combat related

int Mob::attack(int dmg) {
	this->health -= dmg;
	return this->health;
	
}

//there should be many improvements of the enemyfinding.
//Now there are many unresonable enemyfinding;
bool Mob::findAndSetAttackableMob() {
	// Find an attackable target that's in the same quardrant as this Mob
	// If a target is found, this function returns true
	// If a target is found then this Mob is updated to start attacking it
	for (std::shared_ptr<Mob> otherMob : GameState::mobs) {
		if (otherMob->attackingNorth == this->attackingNorth) { continue; }
		
		/*bool imLeft = this->pos.x < (SCREEN_WIDTH / 2);
		bool otherLeft = otherMob->pos.x < (SCREEN_WIDTH / 2);

		bool imTop = this->pos.y < (SCREEN_HEIGHT / 2);
		bool otherTop = otherMob->pos.y < (SCREEN_HEIGHT / 2);
		if((imLeft == otherLeft) && (imTop == otherTop)) {*/
		else{
			// If we're in the same quardrant as the otherMob
			// Mark it as the new target
			if (checkIsInAttackRange(otherMob)) {
				this->setAttackTarget(otherMob);
				//this->targetMob = otherMob;
				return true;
			}
		}
	}

	float currentDistance = 0.f;
	float lastdistance = INFINITE;
	std::shared_ptr<Building> shortestMob;
	for (std::shared_ptr<Building> otherMob : GameState::buildings) {
		if (!otherMob->isNorthBuilding == this->attackingNorth) { continue; }
		else if (otherMob->isDead()) { continue; }
		else {
			float d1 = abs(otherMob->getPoint().x - this->getPosition()->x);
			float d2 = abs(otherMob->getPoint().y - this->getPosition()->y);
			currentDistance = d2 + d1;
			if (currentDistance < lastdistance) {
				shortestMob = otherMob;
				lastdistance = currentDistance;
			}
		}
	}
	if (shortestMob != nullptr) {
		if (checkIsInAttackRange(shortestMob)) {
			setAttackTarget(shortestMob);
			return true;
		}
	}
	return false;
}

bool Mob::checkIsInAttackRange(std::shared_ptr<Attackable> otherMob) {
	if (((this->getPosition()->y < 50.f && otherMob->getPosition()->y>50.f) || (this->getPosition()->y > 50.f && otherMob->getPosition()->y< 50.f))) {
		return false;
	}
	else {
		return true;
	}
}

// TODO Move this somewhere better like a utility class
int randomNumber(int minValue, int maxValue) {
	// Returns a random number between [min, max). Min is inclusive, max is not.
	return (rand() % maxValue) + minValue;
}

void Mob::setAttackTarget(std::shared_ptr<Attackable> newTarget) {
	this->state = MobState::Attacking;
	target = newTarget;
	
}

bool Mob::targetInRange() {
	float range = this->GetSize() / 2; // TODO: change this for ranged units
	float totalSize = range + target->GetSize() / 2 + 0.05f; //attack range should bigger than collider range
	return this->pos.insideOf(*(target->getPosition()), totalSize);
}
// Combat related
////////////////////////////////////////////////////////////
// Collisions

// PROJECT 3: 
//  1) return a vector of mobs that we're colliding with
//  2) handle collision with towers & river 
// I think there should be a class of collider, because now, i cannot add building, river and mob into a vector.
// if i change the everything to attacable, there is no function of moveToward.
// Plus, I think use three vectors to store different kinds of colliders is better.
// Now, the checkCollision only check the collision between mobs, the collision between buidings and rivers checked in processCollision
std::vector<std::shared_ptr<Mob>> Mob::checkCollision() {
	std::vector<std::shared_ptr<Mob>> collidingMobs;
	for (std::shared_ptr<Mob> otherMob : GameState::mobs) {
		// don't collide with yourself
		//std::cout << otherMob->getPosition()->y;
		if (this->sameMob(otherMob)) {
				
		}
		else {
			float averageOfSzie = (this->GetSize() + otherMob->GetSize()) / 2;
			if (abs(this->getPosition()->x - otherMob->getPosition()->x) < averageOfSzie && abs(this->getPosition()->y - otherMob->getPosition()->y) < averageOfSzie) {
				collidingMobs.push_back(otherMob);
			}
		}
		// PROJECT 3: YOUR CODE CHECKING FOR A COLLISION GOES HERE
	}
	
	return collidingMobs;
}

//check whether or not move to same direction
bool Mob::isMoveSamedir(std::shared_ptr<Mob> otherMob) {
	Point thisMoveDir;
	thisMoveDir.x = (this->getPosition()->x - this->lastPosition->x);
	thisMoveDir.y = (this->getPosition()->y - this->lastPosition->y);
	thisMoveDir.normalize();
	Point otherMoveDir;
	otherMoveDir.x =  (otherMob->getPosition()->x - otherMob->lastPosition->x);
	otherMoveDir.y = (otherMob->getPosition()->y - otherMob->lastPosition->y);
	otherMoveDir.normalize();
	if (sqrt(2) / 2 < ((double)thisMoveDir.x * (double)otherMoveDir.x + (double)thisMoveDir.y * (double)otherMoveDir.y) < 1) {
		return true;
	}
	return false;
}

//check "this" whether or not behind the otherMob
bool Mob::isBehind(std::shared_ptr<Mob> otherMob) {
	Point thisMoveDir;
	thisMoveDir.x = (this->getPosition()->x - this->lastPosition->x);
	thisMoveDir.y = (this->getPosition()->y - this->lastPosition->y);
	thisMoveDir.normalize();
	Point otherMoveDir;
	otherMoveDir.x = (otherMob->getPosition()->x - otherMob->lastPosition->x);
	otherMoveDir.y = (otherMob->getPosition()->y - otherMob->lastPosition->y);
	otherMoveDir.normalize();
	if (sqrt(2) / 2 < ((double)thisMoveDir.x * (double)otherMoveDir.x + (double)thisMoveDir.y * (double)otherMoveDir.y) < 1) {
		Point dif;
		dif.x = otherMob->getPosition()->x - this->getPosition()->x;
		dif.y = otherMob->getPosition()->y - this->getPosition()->y;
		if (thisMoveDir.x * dif.x + thisMoveDir.y * dif.y > 0) {
			return true;
		}
	}
	return false;
}

void Mob::processCollision(std::vector<std::shared_ptr<Mob>> otherMobs, double elapsedTime) {
	for (std::shared_ptr<Building> otherBuilding : GameState::buildings) {
		if (!otherBuilding->isDead()) {
			float averageOfSzie = (this->GetSize() + otherBuilding->GetSize()) / 2;
			if (abs(this->getPosition()->x - otherBuilding->getPosition()->x) < averageOfSzie && abs(this->getPosition()->y - otherBuilding->getPosition()->y) < averageOfSzie) {
				std::shared_ptr<Point> pos = std::shared_ptr<Point>(new Point( (this->getPosition()->x - otherBuilding->getPosition()->x),  (this->getPosition()->y - otherBuilding->getPosition()->y)));
				this->moveTowardsVector(pos, elapsedTime);
			}
		}
	}
	for (std::shared_ptr<Mob> river : GameState::rivers) {
		float averageOfSzie = (this->GetSize() + river->GetSize()) / 2;
		if (abs(this->getPosition()->x - river->getPosition()->x) < averageOfSzie && abs(this->getPosition()->y - river->getPosition()->y) < averageOfSzie) {
			std::shared_ptr<Point> pos = std::shared_ptr<Point>(new Point( (this->getPosition()->x - river->getPosition()->x), (this->getPosition()->y - river->getPosition()->y)));
			if (this->getPosition()->x <= 30 && this->getPosition()->y <= 50) {
				std::shared_ptr<Point> newTarget = std::shared_ptr<Point>(new Point);
				newTarget->x = 15.f;
				newTarget->y = 48.5f;
				this->moveTowards(newTarget, elapsedTime);
			}
			else if (this->getPosition()->x <= 30 && this->getPosition()->y > 50) {
				std::shared_ptr<Point> newTarget = std::shared_ptr<Point>(new Point);
				newTarget->x = 15.f;
				newTarget->y = 51.5f;
				this->moveTowards(newTarget, elapsedTime);
			}
			else if (this->getPosition()->x > 30 && this->getPosition()->y <= 50) {
				std::shared_ptr<Point> newTarget = std::shared_ptr<Point>(new Point);
				newTarget->x = 45.f;
				newTarget->y = 48.5f;
				this->moveTowards(newTarget, elapsedTime);
			}
			else {
				std::shared_ptr<Point> newTarget = std::shared_ptr<Point>(new Point);
				newTarget->x = 45.f;
				newTarget->y = 51.5f;
				this->moveTowards(newTarget, elapsedTime);
			}
			this->moveTowardsVector(pos, elapsedTime);
		}
		
	}
	if(!otherMobs.empty()){
		for (std::shared_ptr<Mob> otherMob : otherMobs) {
				if (otherMob->GetMass() > this->GetMass()) {
					std::shared_ptr<Point> pos = std::shared_ptr<Point>(new Point( (this->getPosition()->x - otherMob->getPosition()->x),  (this->getPosition()->y - otherMob->getPosition()->y)));
					this->moveTowardsVector(pos, elapsedTime);
				}
				else if (otherMob->GetMass() == this->GetMass()) {
					std::shared_ptr<Point> pos = std::shared_ptr<Point>(new Point( (otherMob->getPosition()->x - this->getPosition()->x),  (otherMob->getPosition()->y - this->getPosition()->y)));
					std::shared_ptr<Point> pos1 = std::shared_ptr<Point>(new Point( (this->getPosition()->x - otherMob->getPosition()->x),  (this->getPosition()->y - otherMob->getPosition()->y)));
					if (this->isMoveSamedir(otherMob)) {
						if (this->isBehind(otherMob)) {
							this->moveTowardsVector(pos1, elapsedTime);
						}
						else {
							otherMob->moveTowardsVector(pos, elapsedTime);
						}
					}
					else {
						this->moveTowardsVector(pos1, elapsedTime);
						otherMob->moveTowardsVector(pos, elapsedTime);
					}
				}
				else if (otherMob->GetMass() < this->GetMass()) {
					std::shared_ptr<Point> pos = std::shared_ptr<Point>(new Point( (otherMob->getPosition()->x - this->getPosition()->x),  (otherMob->getPosition()->y - this->getPosition()->y)));
					otherMob->moveTowardsVector(pos, elapsedTime);
			}
		}
	}
}

// Collisions
///////////////////////////////////////////////
// Procedures

void Mob::attackProcedure(double elapsedTime) {
	if(this->target == nullptr || this->target->isDead()) {
		this->targetLocked = false;
		this->target = nullptr;
		this->state = MobState::Moving;
		return;
	}
	std::vector<std::shared_ptr<Mob>> otherMobs = this->checkCollision();
		this->processCollision(otherMobs, elapsedTime);
		
	if (targetInRange()) {
		if (this->lastAttackTime >= this->GetAttackTime()) {
			// If our last attack was longer ago than our cooldown
			this->target->attack(this->GetDamage());
			this->lastAttackTime = 0; // lastAttackTime is incremented in the main update function
			return;
		}
		
	}
	else {
		// If the target is not in range, find new attackable target.
		this->findAndSetAttackableMob();
		this ->moveTowards(target->getPosition(), elapsedTime);
	}
}

void Mob::moveProcedure(double elapsedTime) {
	if (targetPosition) {
		// Fighting otherMob takes priority always
		this->findAndSetAttackableMob();

		this->moveTowards(targetPosition, elapsedTime);

		// Check for collisions
		if (this->nextWaypoint->pos.insideOf(this->pos, (this->GetSize() + WAYPOINT_SIZE))) {
			std::shared_ptr<Waypoint> trueNextWP = this->attackingNorth ?
				this->nextWaypoint->upNeighbor :
				this->nextWaypoint->downNeighbor;
			this->setNewWaypoint(trueNextWP);
		}

		// PROJECT 3: You should not change this code very much, but this is where your 
		// collision code will be called from
		std::vector<std::shared_ptr<Mob>> otherMobs = this->checkCollision();
		this->processCollision(otherMobs, elapsedTime);

	}
	else {
		// if targetPosition is nullptr
		this->findNewTarget();
	}
}

void Mob::update(double elapsedTime) {

	switch (this->state) {
	case MobState::Attacking:
		this->attackProcedure(elapsedTime);
		break;
	case MobState::Moving:
	default:
		this->moveProcedure(elapsedTime);
		break;
	}

	this->lastAttackTime += (float)elapsedTime;
}
