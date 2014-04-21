#ifndef _GAZEBO_PHYSICS_PLUGIN_HH_
#define _GAZEBO_PHYSICS_PLUGIN_HH_

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*HELLO)(void);
//typedef PhysicsPlugin *(*CREATE)(void);

struct _PhysicsPlugin
{
  //CREATE create;
  HELLO hello;
};
typedef struct _PhysicsPlugin PhysicsPlugin;

//typedef PhysicsEngine *(*createEngine)();
// typedef int (*DestroyEngineFn)(PhysicsEngine *);

PhysicsPlugin *create_engine(void);

//CreateEngineFn createEngine;
//int destroy_engine(PhysicsEngine *_engine);

/*int resetEngine(PhysicsEngine *_engine);

int setSeed(PhysicsEngine *_engine, int32_t _seed);

&double getUpdatePeriod(PhysicsEngine *_engine);
double getMaxStepSize(PhysicsEngine *_engine);

int updateCollision(PhysicsEngine *_engine);
int updatePhysics(PhysicsEngine *_engine);

Link *createLink(PhysicsEngine *_engine);
*/

#ifdef __cplusplus
}
#endif

#endif
