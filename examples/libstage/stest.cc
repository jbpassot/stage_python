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
#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"
#include <boost/python.hpp>
#include <numpy/ndarrayobject.h>
//#include <numpy/arrayobject.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
using namespace boost::python;

typedef std::vector<double> DoubleVectorType;
typedef std::vector<int> IntVectorType;


class DifferentialDriveState
{
public:
    Stg::usec_t timestamp_us;
    double traction_left_distance_mm;
    double traction_right_distance_mm;
    double heading_angle_rad;
    double angular_velocity_rad_per_sec;

    DifferentialDriveState() : timestamp_us(0), traction_left_distance_mm(0.), traction_right_distance_mm(0.), heading_angle_rad(0.), angular_velocity_rad_per_sec(0.)
    {}
};

boost::python::object stdVecToNumpyArray( std::vector<double> const& vec )
{
   npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    double * data = size ? const_cast<double *>(&vec[0])
                         : static_cast<double *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_DOUBLE, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}

boost::python::object stdFloatVecToNumpyArray( std::vector<float> const& vec )
{
    npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    float * data = size ? const_cast<float *>(&vec[0])
                         : static_cast<float *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_FLOAT, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}



class Robot {
public:
  Stg::ModelPosition *position;
  Stg::ModelRanger *ranger;
  Stg::ModelCamera *camera;
};

class Logic {
  Logic(): scan(){}
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

  Stg::Velocity get_odometry_data(){
    return velocity;
  }

  std::vector<Stg::meters_t> get_lidar_data(){
    return scan;
  }

   std::vector<float> get_camera_data(){
    return depth_data;
  }


    int get_camera_width(){
      return camera_width;
    }

    int get_camera_height(){
      return camera_height;
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

        // get the robot's camera model and subscribe to it
        Stg::ModelCamera *cammod =
                reinterpret_cast<Stg::ModelCamera *>(robots[idx].position->GetChild("camera:0"));
        robots[idx].camera = cammod;


        if (cammod)
            camera_width = robots[idx].camera->getWidth();
            camera_height = robots[idx].camera->getHeight();
            robots[idx].camera->Subscribe();

                // get the robot's ranger model and subscribe to it
      Stg::ModelRanger *rngmod =
          reinterpret_cast<Stg::ModelRanger *>(robots[idx].position->GetChild("ranger:1"));
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

      //Step 1. Read status of the robot
      velocity = robots[0].position-> GetVelocity();
      scan = robots[0].ranger->GetSensors()[0].ranges;
      const double interval((double)world->sim_interval / 1e6);

      robot_state.timestamp_us = world->SimTimeNow();
      robot_state.traction_left_distance_mm += wheel_distance* (2*velocity.x + velocity.a) / 2.;
      robot_state.traction_right_distance_mm += wheel_distance* (2*velocity.x - velocity.a) / 2.;
      robot_state.angular_velocity_rad_per_sec = velocity.a;
      robot_state.heading_angle_rad += velocity.a * interval;


      if (robots[0].camera != NULL) {
          float *depth_data_camera = (float *) robots[0].camera->FrameDepth();
          float *rgb_data_camera = (float *) robots[0].camera->FrameColor();
          int size = robots[0].camera->getWidth() * robots[0].camera->getHeight();
          if (size && depth_data_camera) {
              //depth_data_camera = std::array<float,4> b = a;
              depth_data.resize(size);
              std::copy(depth_data_camera, depth_data_camera + size, depth_data.begin());
          }
      }

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

      robots[0].position->SetSpeed(brainos_forward_speed, 0., brainos_angular_speed);

      /*
      if (brainos_forward_speed != 0 || brainos_angular_speed != 0) {
          printf("Run for %d steps\n", 50);
          world_gui->UnpauseForNumSteps(50);
      }*/

        /*
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
    */
  }

protected:
  unsigned int population_size;
  Robot *robots;

    Stg::Velocity velocity;
    Stg::ModelRanger *rgr;
    std::vector<double> scan;
    std::vector<float> depth_data;
    int camera_width = 0;
    int camera_height = 0;
    double wheel_distance = 0.5;

public:
    DifferentialDriveState robot_state;

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

        logic = new Logic(1);

        std::pair<Stg::WorldGui *, Logic * > * params = new std::pair<Stg::WorldGui *, Logic * >(world, logic);
        pthread_create(&cThread, NULL,  (func_ptr)StageSimulator::run, params);
        //sleep(1);

    } // added constructor
    //static void run(void * world_param){
        //Stg::WorldGui * world = reinterpret_cast<Stg::WorldGui *>(world_param);
       //static void run(Stg::WorldGui * world){
       static void run(std::pair<Stg::WorldGui *, Logic * > * params){

        Stg::WorldGui * world = params->first;
        Logic * logic = params->second;

        // create the world
        //world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");
        world->Load("/home/jb/projects/stage4/Stage/worlds/benchmark/hospital_2.world");

        // create the logic and connect it to the world
        //logic = new Logic(1);
        logic->connect(world);
        world->Run();
    }


    DoubleVectorType get_odom(){
        Stg::Velocity velocity = logic->get_odometry_data();
        DoubleVectorType odom;
        odom.push_back(velocity.x);
        odom.push_back(velocity.y);
        odom.push_back(velocity.a);
        return odom;
    }

    boost::python::object get_scan(){
        std::vector<double> lidar =  logic->get_lidar_data();
        if (lidar.size() == 0) lidar.push_back(0.);
        return (stdVecToNumpyArray(lidar));
    }


    boost::python::object get_depth(){
        std::vector<float> depth =  logic->get_camera_data();
        if (depth.size() == 0) depth.push_back(0.);
        return (stdFloatVecToNumpyArray(depth));
    }

    boost::python::dict get_robot_state( )
    {
        boost::python::dict robot_state_dict;
        robot_state_dict["timestamp_us"] = logic->robot_state.timestamp_us;
        robot_state_dict["traction_left_distance_mm"] = logic->robot_state.traction_left_distance_mm;
        robot_state_dict["traction_right_distance_mm"] = logic->robot_state.traction_right_distance_mm;
        robot_state_dict["heading_angle_rad"] = logic->robot_state.heading_angle_rad;
        robot_state_dict["angular_velocity_rad_per_sec"] =  logic->robot_state.angular_velocity_rad_per_sec;
        return robot_state_dict;
    }

    int get_camera_width(){
        return logic->get_camera_width();
    }

    int get_camera_height(){
        return logic->get_camera_height();
    }

    DoubleVectorType get_left_and_right_wheels_distances(){
        DoubleVectorType wheel_velocities;
        wheel_velocities.push_back(logic->robot_state.traction_left_distance_mm);
        wheel_velocities.push_back(logic->robot_state.traction_right_distance_mm);
        return wheel_velocities;
    }


    std::string world_file;
    Stg::WorldGui * world;
    Logic * logic;
};




BOOST_PYTHON_MODULE(stagesim)
{

    Py_Initialize();
    import_array()
    numeric::array::set_module_and_type("numpy", "ndarray");

    class_<DoubleVectorType>("DoubleVectorType")
            .def(vector_indexing_suite<DoubleVectorType>() );


    class_<IntVectorType>("IntVectorType")
            .def(vector_indexing_suite<IntVectorType>() );

    class_<StageSimulator>("StageSimulator", init<std::string>())
            .def("run", &StageSimulator::run)
            .def("get_odom", &StageSimulator::get_odom)
            .def("get_scan", &StageSimulator::get_scan)
            .def("get_depth", &StageSimulator::get_depth)
            .def("get_camera_width", &StageSimulator::get_camera_width)
            .def("get_camera_height", &StageSimulator::get_camera_height)
            .def("get_left_and_right_wheels_distances", &StageSimulator::get_left_and_right_wheels_distances)
            .def("get_robot_state", &StageSimulator::get_robot_state)
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
