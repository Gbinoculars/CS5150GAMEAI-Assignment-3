#pragma once

#include "Mob.h"

class Mob_River : public Mob
{
public:
	virtual int GetMaxHealth() const { return 0; }
	virtual float GetSpeed() const { return 1.0f; }
	virtual float GetSize() const { return 3.0f; }
	virtual float GetMass() const { return INFINITY; }
	virtual int GetDamage() const { return 0; }
	virtual float GetAttackTime() const { return 0; }
	const char* GetDisplayLetter() const { return "A"; }
};
