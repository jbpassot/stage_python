/////////////////////////////////
// File: stest.c
// Desc: Stage library test program
// Created: 2004.9.15
// Author: Richard Vaughan <vaughan@sfu.ca>
// CVS: $Id: stest.cc,v 1.1 2008-01-15 01:29:10 rtv Exp $
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"
#include <boost/python.hpp>
using namespace boost::python;

class Robot {
public:
  Stg::ModelPosition *position;
  Stg::ModelRanger *ranger;
};

class Logic {
  Logic(const Logic &) {}
public:
  static int Callback(Stg::World *world, void *userarg)
  {
    Logic *lg = reinterpret_cast<Logic *>(userarg);

    lg->Tick(world);

    // never remove this call-back
    return 0;
  }

    static int MoveCallback(Stg::World *world, void *userarg)
    {

        PRINT_ERR("In move Callback\n");
        return 0;
    }


    explicit Logic(unsigned int popsize)
      : population_size(popsize), robots(new Robot[population_size])
  {
  }

  void connect(Stg::World *world)
  {
    // connect the first population_size robots to this controller
    for (unsigned int idx = 0; idx < population_size; idx++) {
      // the robots' models are named r0 .. r1999
      std::stringstream name;
      name << "r" << idx;

      // get the robot's model and subscribe to it
      Stg::ModelPosition *posmod =
          reinterpret_cast<Stg::ModelPosition *>(world->GetModel(name.str()));
      assert(posmod != 0);

      robots[idx].position = posmod;
      robots[idx].position->Subscribe();

      // get the robot's ranger model and subscribe to it
      Stg::ModelRanger *rngmod =
          reinterpret_cast<Stg::ModelRanger *>(robots[idx].position->GetChild("ranger:0"));
      assert(rngmod != 0);

      robots[idx].ranger = rngmod;
      robots[idx].ranger->Subscribe();
    }
      Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);

    world_gui->AddMoveCallback(Logic::MoveCallback, reinterpret_cast<void *>(this));
    // register with the world
    world->AddUpdateCallback(Logic::Callback, reinterpret_cast<void *>(this));
  }


  ~Logic() { delete[] robots; }
  void Tick(Stg::World * world) {
      //PRINT_ERR("Tick\n");
      Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);
      //world_gui->Redraw();
      //world_gui->NeedRedraw();
      double left_wheel_velocity_meter, right_wheel_velocity_meter, wheel_distance;
      left_wheel_velocity_meter = 0.3;
      right_wheel_velocity_meter = 0.;
      wheel_distance = 0.5;

      double brainos_forward_speed = (left_wheel_velocity_meter + right_wheel_velocity_meter) / 2.;
      double brainos_angular_speed = (left_wheel_velocity_meter - right_wheel_velocity_meter) / wheel_distance;


      brainos_forward_speed = world_gui->linear * 1.5;
      brainos_angular_speed = -world_gui->angular * 0.5;

      /*
      if (brainos_forward_speed != 0 || brainos_angular_speed != 0) {
          printf("Run for %d steps\n", 50);
          world_gui->UnpauseForNumSteps(50);
      }*/

      // the controllers parameters
    const double vspeed = 0.4; // meters per second
    const double wgain = 1.0; // turn speed gain
    const double safe_dist = 0.1; // meters
    const double safe_angle = 0.3; // radians

    //usleep(10000);

    // each robot has a group of ir sensors
    // each sensor takes one sample
    // the forward sensor is the middle sensor
    for (unsigned int idx = 0; idx < population_size; idx++) {
      Stg::ModelRanger *rgr = robots[idx].ranger;

      // compute the vector sum of the sonar ranges
      double dx = 0, dy = 0;

      // the range model has multiple sensors
      typedef std::vector<Stg::ModelRanger::Sensor>::const_iterator sensor_iterator;
      const std::vector<Stg::ModelRanger::Sensor> sensors = rgr->GetSensors();

      for (sensor_iterator sensor = sensors.begin(); sensor != sensors.end(); ++sensor) {
        // each sensor takes a single sample (as specified in the .world)
        const double srange = (*sensor).ranges[0];
        const double angle = (*sensor).pose.a;

        dx += srange * std::cos(angle);
        dy += srange * std::sin(angle);
      }

      if (dx == 0)
        continue;

      if (dy == 0)
        continue;

      // calculate the angle towards the farthest obstacle
      const double resultant_angle = std::atan2(dy, dx);

      // check whether the front is clear
      const unsigned int forward_idx = sensors.size() / 2u - 1u;

      const double forwardm_range = sensors[forward_idx - 1].ranges[0];
      const double forward_range = sensors[forward_idx + 0].ranges[0];
      const double forwardp_range = sensors[forward_idx + 1].ranges[0];

      bool front_clear =
          ((forwardm_range > safe_dist / 5.0) && (forward_range > safe_dist)
           && (forwardp_range > safe_dist / 5.0) && (std::abs(resultant_angle) < safe_angle));

      // turn the sensor input into movement commands

      // move forwards if the front is clear
      const double forward_speed = front_clear ? vspeed : 0.1;
      // do not strafe
      const double side_speed = 0.0;

      // turn towards the farthest obstacle
      const double turn_speed = wgain * resultant_angle;

      // finally, relay the commands to the robot
      robots[idx].position->SetSpeed(brainos_forward_speed, side_speed, brainos_angular_speed);
    }
  }

protected:
  unsigned int population_size;
  Robot *robots;
};

struct StageSimulator
{
    StageSimulator(std::string world_file): world_file(world_file) {
        // initialize libstage
        int argc = 3;
        char* argv[3];
        argv[0] = "stage";
        argv[1] = (char *) world_file.c_str();
        argv[2] = "1";
        int popsize = 1;
        char ** a = argv;
        pthread_t cThread;

        Stg::Init(&argc, &a);

        // create the world
        world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");

        // normal posix pthread C function pointer
        typedef void *(*func_ptr)(void *);
        pthread_create(&cThread, NULL,  (func_ptr)StageSimulator::run, world);
        //sleep(1);

    } // added constructor
    //static void run(void * world_param){
        //Stg::WorldGui * world = reinterpret_cast<Stg::WorldGui *>(world_param);
    static void run(Stg::WorldGui * world){

        // create the world
        //world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");
        world->Load("/home/jb/projects/stage4/Stage/worlds/benchmark/hospital_2.world");
        // create the logic and connect it to the world
        Logic logic(1);
        logic.connect(world);
        world->Run();
    }

    uint64_t get_info(){
        world->UnpauseForNumSteps(20);
        return world->GetUpdateCount();
    }

    std::string world_file;
    Stg::WorldGui * world;
};


BOOST_PYTHON_MODULE(stagesim)
{
    class_<StageSimulator>("StageSimulator", init<std::string>())
            .def("run", &StageSimulator::run)
            .def("get_info", &StageSimulator::get_info)
            ;
}

int main(int argc, char *argv[])
{
  // check and handle the argumets
  if (argc < 3) {
    puts("Usage: stest <worldfile> <number of robots>");
    exit(0);
  }

  const int popsize = atoi(argv[2]);

  // initialize libstage
  Stg::Init(&argc, &argv);

  // create the world
  // Stg::World world;
  Stg::WorldGui world(800, 700, "Stage Benchmark Program");
  world.Load(argv[1]);

  // create the logic and connect it to the world
  Logic logic(popsize);
  logic.connect(&world);

  // and then run the simulation
  world.Run();

  return 0;
}
