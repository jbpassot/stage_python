#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"
#include "python_utils.hh"
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

using namespace boost::python;

typedef std::vector<double> DoubleVectorType;
typedef std::vector<int> IntVectorType;


class DifferentialDriveCommand{
public:
    Stg::usec_t timestamp_us;
    double traction_left_wheel_speed;
    double traction_right_wheel_speed;

    DifferentialDriveCommand(): timestamp_us(0), traction_left_wheel_speed(0.), traction_right_wheel_speed(0.)
    {}
};

class DifferentialDriveState
{
public:
    Stg::usec_t timestamp_us;
    double traction_left_distance_mm;
    double traction_right_distance_mm;
    double heading_angle_rad;
    double angular_velocity_rad_per_sec;
    double wheel_distance;

    DifferentialDriveState() : wheel_distance(0.413), timestamp_us(0), traction_left_distance_mm(0.),
        traction_right_distance_mm(0.), heading_angle_rad(0.), angular_velocity_rad_per_sec(0.)
    {}
};

class LidarState{
public:
    Stg::usec_t timestamp_us;
    //Stg::Pose pose;
    //Stg::Bounds range;
    double fov;
    //double angle_noise; //< variance for ranger angle
    //double range_noise; //< variance for range readings
    //double range_noise_const; //< variance for constant noise (not depending on range)
    unsigned int sample_count;

    std::vector<double> ranges;
    std::vector<double> intensities;
    std::vector<double> bearings;

    LidarState():
    //pose(0, 0, 0, 0), range(0.0, 5.0),
    fov(0.1), timestamp_us(0),
    sample_count(1), ranges(), intensities(), bearings()
    {}
};

class CameraState{
public:
    Stg::usec_t timestamp_us;
    std::vector<float> depth_data;
    std::vector<unsigned char> rgb_data;
    int camera_width = 0;
    int camera_height = 0;

    bool enabled;

    CameraState():
        depth_data(),rgb_data(),camera_width(0),camera_height(0), timestamp_us(0), enabled(true)
    {}
};


class FiducialState{
public:
    Stg::usec_t timestamp_us;
    bool detected;
    double bearing;
    double range;
    double heading;


    FiducialState():
            timestamp_us(0), detected(false),bearing(0.), range(0.), heading(0.)
    {}
};



class Robot {
public:
  Stg::ModelPosition *position;
  Stg::ModelRanger *ranger;
  Stg::ModelCamera *camera;
  Stg::ModelFiducial *fiducial;
  Stg::ModelBumper *bumpers;
};

class Logic {
  Logic() {}
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


    explicit Logic(unsigned int popsize, std::string world_file):
    population_size(popsize), robots(new Robot[population_size]), last_timestamp_us_command(0), world_file(world_file)
  {
  }

  Stg::Velocity get_odometry_data(){
    return velocity;
  }

   Stg::ModelCamera *  get_camera(){
      return robots[0].camera;
  }

  void connect(Stg::World *world){
        int idx = 0;
        // the robots' models are named r0 .. r1999
        std::stringstream name;
        name << "r" << idx;

        // get the robot's model and subscribe to it
        Stg::ModelPosition *posmod =
          reinterpret_cast<Stg::ModelPosition *>(world->GetModel(name.str()));
        assert(posmod != 0);

        robot_state.wheel_distance = posmod->wheeldistance;
        robots[idx].position = posmod;
        robots[idx].position->Subscribe();

        robots[idx].fiducial = (Stg::ModelFiducial *)robots[idx].position->GetUnusedModelOfType("fiducial");
        robots[idx].fiducial ->AddCallback(Stg::Model::CB_UPDATE, (Stg::model_callback_t)FiducialUpdate, &fiducial_state);
        robots[idx].fiducial ->Subscribe();

        robots[idx].bumpers = reinterpret_cast<Stg::ModelBumper *>(robots[idx].position->GetChild("bumper:0"));
          if (robots[idx].bumpers){
              robots[idx].bumpers->Subscribe();
          }

        // get the robot's camera model and subscribe to it
        Stg::ModelCamera *cammod =
                reinterpret_cast<Stg::ModelCamera *>(robots[idx].position->GetChild("camera:0"));
        robots[idx].camera = cammod;
        if (cammod){
            camera_state.camera_width = robots[idx].camera->getWidth();
            camera_state.camera_height = robots[idx].camera->getHeight();
            robots[idx].camera->Subscribe();
            //robots[idx].camera->enable_camera(false);
        }
        // get the robot's ranger model and subscribe to it
        Stg::ModelRanger *rngmod =
          reinterpret_cast<Stg::ModelRanger *>(robots[idx].position->GetChild("ranger:0"));
        assert(rngmod != 0);
        robots[idx].ranger = rngmod;
        robots[idx].ranger->Subscribe();

        Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);

        // register with the world
        world->AddUpdateCallback(Logic::Callback, reinterpret_cast<void *>(this));
  }

    static int FiducialUpdate(Stg::ModelFiducial *mod, FiducialState *fiducial)
    {
        std::vector<Stg::ModelFiducial::Fiducial> &fids = mod->GetFiducials();

        fiducial->detected = false;
        for (unsigned int i = 0; i < fids.size(); i++) {
            // printf( "fiducial %d is %d at %.2f m %.2f radians\n",
            //	  i, f->id, f->range, f->bearing );

            if (fids[i].id == 9) // I see a charging station
            {
                fiducial->detected = true;
                fiducial->range = fids[i].range;
                fiducial->heading = fids[i].geom.a;
                fiducial->bearing = fids[i].bearing;
                break;
            }
        }

        return 0; // run again
    }

  ~Logic() { delete[] robots; }
  void Tick(Stg::World * world) {
      bool debug = false;
      Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);
      double brainos_forward_speed;
      double brainos_angular_speed;

      //Read status of the robot
      velocity = robots[0].position->GetVelocity();
      odom_pose = robots[0].position->GetPose();
      global_pose = robots[0].position->GetGlobalPose();

      if (debug){
          PRINT_ERR1("%s", world->ClockString().c_str());
          PRINT_ERR2("Robot Speed (%f, %f)", velocity.x, velocity.a);
      }

      /*
      Stg::ModelBumper * bump = robots[0].bumpers;
      if ((bump->samples && bump->bumpers && bump->bumper_count)) {

          if (bump->samples[0].hit){
              PRINT_ERR("HITING BUMPER 0");
          }
          if (bump->samples[1].hit)
          {
              PRINT_ERR("HITING BUMPER 1");
          }
      }*/

      // Filling scan state
      lidar_state.timestamp_us = robots[0].ranger->last_update;
      lidar_state.ranges = robots[0].ranger->GetSensors()[0].ranges;
      lidar_state.intensities = robots[0].ranger->GetSensors()[0].intensities;
      lidar_state.bearings = robots[0].ranger->GetSensors()[0].bearings;
      lidar_state.fov = robots[0].ranger->GetSensors()[0].fov;
      lidar_state.sample_count = robots[0].ranger->GetSensors()[0].sample_count;

      // Filling robot state
      const double interval((double)world->sim_interval / 1e6);
      robot_state.timestamp_us = world->SimTimeNow();

      double left_velocity = velocity.x - 0.5*(velocity.a*robot_state.wheel_distance);
      double right_velocity = velocity.x + 0.5*(velocity.a*robot_state.wheel_distance);
      robot_state.traction_left_distance_mm += (left_velocity*interval) * 1000.;
      robot_state.traction_right_distance_mm += (right_velocity*interval) * 1000.;

      robot_state.angular_velocity_rad_per_sec = velocity.a;
      robot_state.heading_angle_rad += velocity.a * interval;

      // Filling Camera state
      if (camera_state.enabled && robots[0].camera != NULL && robots[0].camera->last_update > camera_state.timestamp_us) {

          float *depth_data_camera = (float *) robots[0].camera->FrameDepth();
          int size = robots[0].camera->getWidth() * robots[0].camera->getHeight();
          if (size && depth_data_camera) {
              camera_state.depth_data.resize(size);
              std::copy(depth_data_camera, depth_data_camera + size, camera_state.depth_data.begin());
          }
          camera_state.timestamp_us = robots[0].camera->last_update;
          unsigned char *rgb_data_camera = (unsigned char *) robots[0].camera->FrameColor();
          if (size && rgb_data_camera) {
              camera_state.rgb_data.resize(size * 4);
              std::copy(rgb_data_camera, rgb_data_camera + size * 4, camera_state.rgb_data.begin());
          }
          camera_state.timestamp_us = robots[0].camera->last_update;
      }

      // Sending motor command
      brainos_forward_speed = world_gui->linear * 1.5;
      brainos_angular_speed = -world_gui->angular * 0.5;

      if (drive_command.timestamp_us > last_timestamp_us_command){
          last_timestamp_us_command = drive_command.timestamp_us;
          brainos_forward_speed = (drive_command.traction_left_wheel_speed + drive_command.traction_right_wheel_speed) / 2.;
          brainos_angular_speed = (drive_command.traction_right_wheel_speed - drive_command.traction_left_wheel_speed) / robot_state.wheel_distance;
      }
      robots[0].position->SetSpeed(brainos_forward_speed, 0., brainos_angular_speed);
  }

protected:
  unsigned int population_size;
  Robot *robots;

  Stg::ModelRanger *rgr;
  Stg::usec_t last_timestamp_us_command;


public:
    DifferentialDriveState robot_state;
    DifferentialDriveCommand drive_command;
    LidarState lidar_state;
    CameraState camera_state;
    FiducialState fiducial_state;
    std::string world_file;

    Stg::Pose odom_pose;
    Stg::Pose global_pose;
    Stg::Velocity velocity;
};



struct StageSimulator
{
    StageSimulator(std::string world_file): world_file(world_file) {

        // initialize libstage
        int number_of_arguments = 3;
        char* parameters[3];
        parameters[0] = "stage";
        parameters[1] = (char *) world_file.c_str();
        parameters[2] = "1";
        char ** cast_parameters = parameters;
        int popsize = 1;
        pthread_t cThread;
        Stg::Init(&number_of_arguments, (char***)&cast_parameters);

        // create the world
        world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");
        timestamp_us = world->SimTimeNow();

        // normal posix pthread C function pointer
        typedef void *(*func_ptr)(void *);

        logic = new Logic(1, world_file);

        std::pair<Stg::WorldGui *, Logic * > * params = new std::pair<Stg::WorldGui *, Logic * >(world, logic);
        pthread_create(&cThread, NULL,  (func_ptr)StageSimulator::run, params);
        //sleep(1);

    } // added constructor

    static void run(std::pair<Stg::WorldGui *, Logic * > * params){

        Stg::WorldGui * world = params->first;
        Logic * logic = params->second;

        // create the world
        world->Load(logic->world_file.c_str());

        // create the logic and connect it to the world
        logic->connect(world);
        world->Run();
    }


    boost::python::object get_odom(){
        boost::python::dict odom;
        Stg::Velocity velocity = logic->velocity;
        Stg::Pose odom_pose = logic->odom_pose;
        Stg::Pose global_pose = logic->global_pose;

        DoubleVectorType odom_velocity_vect, odom_pose_vect, global_pose_vect;
        odom_velocity_vect.push_back(velocity.x);
        odom_velocity_vect.push_back(velocity.y);
        odom_velocity_vect.push_back(velocity.a);

        odom_pose_vect.push_back(odom_pose.x);
        odom_pose_vect.push_back(odom_pose.y);
        odom_pose_vect.push_back(odom_pose.a);

        global_pose_vect.push_back(global_pose.x);
        global_pose_vect.push_back(global_pose.y);
        global_pose_vect.push_back(global_pose.z);

        odom["velocity"]=odom_velocity_vect;
        odom["pose"]=odom_pose_vect;
        odom["global_pose"]=global_pose_vect;

        return odom;
    }

    boost::python::object get_scan_data() {
        boost::python::dict scan_data_dict;
        std::vector<double> ranges = logic->lidar_state.ranges;
        std::vector<double> intensities = logic->lidar_state.intensities;
        std::vector<double> bearings = logic->lidar_state.bearings;
        scan_data_dict["ranges"] = NULL;
        scan_data_dict["intensities"] = NULL;
        scan_data_dict["bearings"] = NULL;
        if (ranges.size() > 0) scan_data_dict["ranges"] = stdDoubleVecToNumpyArray(ranges);
        if (intensities.size() > 0) scan_data_dict["intensities"] = stdDoubleVecToNumpyArray(intensities);
        if (bearings.size() > 0) scan_data_dict["bearings"] = stdDoubleVecToNumpyArray(bearings);
        scan_data_dict["fov"] = logic->lidar_state.fov;
        scan_data_dict["timestamp_us"] = logic->lidar_state.timestamp_us;
        return scan_data_dict;
    }


    boost::python::object get_depth_data(){
        boost::python::dict depth_data_dict;
        std::vector<float> depth =  logic->camera_state.depth_data;
        depth_data_dict["timestamp_us"] = logic->camera_state.timestamp_us;
        depth_data_dict["width"] = logic->camera_state.camera_width;
        depth_data_dict["height"] = logic->camera_state.camera_height;
        depth_data_dict["data"] = NULL;
        if (depth.size() > 0) depth_data_dict["data"] = stdFloatVecToNumpyArray(depth);

        return depth_data_dict;
    }

    boost::python::object get_rgb_data(){
        boost::python::dict rgb_data_dict;
        std::vector<unsigned char> rgb_data =  logic->camera_state.rgb_data;
        rgb_data_dict["timestamp_us"] = logic->camera_state.timestamp_us;
        rgb_data_dict["width"] = logic->camera_state.camera_width;
        rgb_data_dict["height"] = logic->camera_state.camera_height;
        rgb_data_dict["data"] = NULL;
        if (rgb_data.size() > 0) {
            rgb_data_dict["data"] = stdUnsignedCharVecToNumpyArray(rgb_data);
        }
        return rgb_data_dict;
    }

    boost::python::dict get_robot_state( )
    {
        boost::python::dict robot_state_dict;

        robot_state_dict["timestamp_us"] = logic->robot_state.timestamp_us;
        robot_state_dict["traction_left_distance_mm"] = logic->robot_state.traction_left_distance_mm;
        robot_state_dict["traction_right_distance_mm"] = logic->robot_state.traction_right_distance_mm;
        robot_state_dict["heading_angle_rad"] = logic->robot_state.heading_angle_rad;
        robot_state_dict["angular_velocity_rad_per_sec"] =  logic->robot_state.angular_velocity_rad_per_sec;
        robot_state_dict["wheel_distance"] =  logic->robot_state.wheel_distance;
        return robot_state_dict;
    }

    boost::python::dict get_home_marker( )
    {
        boost::python::dict home_marker_dict;

        home_marker_dict["timestamp_us"] = logic->fiducial_state.timestamp_us;
        home_marker_dict["detected"] = logic->fiducial_state.detected;
        home_marker_dict["heading"] = logic->fiducial_state.heading;
        home_marker_dict["range"] = logic->fiducial_state.range;
        home_marker_dict["bearing"] = logic->fiducial_state.bearing;
        return home_marker_dict;
    }

    Stg::usec_t get_robot_state_timestamp_us(){return logic->robot_state.timestamp_us;}
    Stg::usec_t get_depth_data_timestamp_us(){return logic->camera_state.timestamp_us;}
    Stg::usec_t get_rgb_data_timestamp_us(){return logic->camera_state.timestamp_us;}
    Stg::usec_t get_scan_data_timestamp_us(){return logic->lidar_state.timestamp_us;}

    int get_camera_width(){
        return logic->camera_state.camera_width;
    }

    int get_camera_height(){
        return logic->camera_state.camera_height;
    }

    void send_command(double traction_left_wheel_speed, double traction_right_wheel_speed) {
        logic->drive_command.timestamp_us = world->SimTimeNow();
        logic->drive_command.traction_left_wheel_speed = traction_left_wheel_speed;
        logic->drive_command.traction_right_wheel_speed = traction_right_wheel_speed;
    }

    bool step_simulation_async(int number_of_ms) {

        if (world->has_more_step_to_run()) return false;

        int sim_interval_ms = world->sim_interval / 1000;
        int num_steps = number_of_ms /  sim_interval_ms;
        if (number_of_ms % sim_interval_ms != 0){
            num_steps ++;
            PRINT_ERR2("%d ms is not a fraction of sim interval %d", number_of_ms, sim_interval_ms);
            PRINT_ERR2("Step simulation of %d steps (%d ms)", num_steps, num_steps*sim_interval_ms);
        }
        world->UnpauseForNumSteps(num_steps);
        return true;
    }

    bool step_simulation_sync(int number_of_ms, bool wait_after_trigger) {

        while (world->has_more_step_to_run()){usleep(1000);} // Make sure the previous step is done

        int sim_interval_ms = world->sim_interval / 1000;
        int num_steps = number_of_ms /  sim_interval_ms;
        if (number_of_ms % sim_interval_ms != 0){
            num_steps ++;
            PRINT_ERR2("%d ms is not a fraction of sim interval %d", number_of_ms, sim_interval_ms);
            PRINT_ERR2("Step simulation of %d steps (%d ms)", num_steps, num_steps*sim_interval_ms);
        }
        world->UnpauseForNumSteps(num_steps);
        if (wait_after_trigger) {
            while (world->has_more_step_to_run()) { usleep(1000); } // Make sure this step is done
        }
        return true;
    }

    bool has_simulation_stepped() {
        return not world->has_more_step_to_run();
    }

    void release_simulation(){
        world->Unlock();
    }

    void lock_simulation(){
        world->Lock();
    }

    void start_simulation(){
        world->Start();
    }

    void stop_simulation(){
        world->Stop();
    }

    Stg::usec_t get_timestamp_us(){
        timestamp_us = world->SimTimeNow();
        return timestamp_us;
    }

    void enable_camera(){
        logic->camera_state.enabled = true;
    }

    void disable_camera(){
        logic->camera_state.enabled = false;
    }


    std::string world_file;
    Stg::WorldGui * world;
    Logic * logic;
    Stg::usec_t timestamp_us;
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
            .def("get_scan_data", &StageSimulator::get_scan_data)
            .def("get_scan_data_timestamp_us", &StageSimulator::get_scan_data_timestamp_us)

            .def("get_camera_width", &StageSimulator::get_camera_width)
            .def("get_camera_height", &StageSimulator::get_camera_height)

            .def("enable_camera", &StageSimulator::enable_camera)
            .def("disable_camera", &StageSimulator::disable_camera)
            .def("get_depth_data", &StageSimulator::get_depth_data)
            .def("get_depth_data_timestamp_us", &StageSimulator::get_depth_data_timestamp_us)

            .def("get_rgb_data", &StageSimulator::get_rgb_data)
            .def("get_rgb_data_timestamp_us", &StageSimulator::get_rgb_data_timestamp_us)

            .def("get_robot_state", &StageSimulator::get_robot_state)
            .def("get_robot_state_timestamp_us", &StageSimulator::get_robot_state_timestamp_us)

            .def("send_command", &StageSimulator::send_command)
            .def("step_simulation_async", &StageSimulator::step_simulation_async)
            .def("step_simulation_sync", &StageSimulator::step_simulation_sync)
            .def("has_simulation_stepped", &StageSimulator::has_simulation_stepped)
            .def("release_simulation", &StageSimulator::release_simulation)
            .def("lock_simulation", &StageSimulator::lock_simulation)
            .def("start_simulation", &StageSimulator::start_simulation)
            .def("stop_simulation", &StageSimulator::stop_simulation)
            .def("get_timestamp_us", &StageSimulator::get_timestamp_us)
            .def("get_home_marker", &StageSimulator::get_home_marker)
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
  Stg::WorldGui world(800, 700, "Stage Benchmark Program");
  std::string world_file(argv[1]);
  world.Load(world_file.c_str());

  // create the logic and connect it to the world
  Logic logic(popsize, world_file);
  logic.connect(&world);

  // and then run the simulation
  world.Run();

  return 0;
}
