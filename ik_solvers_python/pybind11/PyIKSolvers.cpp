/**
   @author YoheiKakiuchi
*/

#define USE_PRIORITIZED

#include <cnoid/PyUtil>
//
#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
//
#ifdef USE_PRIORITIZED
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>
#endif
//
#include <ik_constraint/AngularMomentumConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/COMVelocityConstraint.h>
#include <ik_constraint/ClientCollisionConstraint.h>
//#include <ik_constraint/CollisionConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/JointLimitConstraint.h>
#include <ik_constraint/JointVelocityConstraint.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/IKConstraint.h>

#include <memory>
#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

//IKConstraint
//AngularMomentum
//COM
//COMVelocity
//ClientCollision
//JointAngle
//JointLimit
//JointVelocity
//Position

typedef std::shared_ptr<IK::IKConstraint >              IKConstraintPtr;
typedef std::shared_ptr<IK::AngularMomentumConstraint > AngularMomentumConstraintPtr;
typedef std::shared_ptr<IK::COMConstraint >             COMConstraintPtr;
typedef std::shared_ptr<IK::COMVelocityConstraint >     COMVelocityConstraintPtr;
typedef std::shared_ptr<IK::ClientCollisionConstraint > ClientCollisionConstraintPtr;
typedef std::shared_ptr<IK::JointAngleConstraint >      JointAngleConstraintPtr;
typedef std::shared_ptr<IK::JointLimitConstraint >      JointLimitConstraintPtr;
typedef std::shared_ptr<IK::JointVelocityConstraint >   JointVelocityConstraintPtr;
typedef std::shared_ptr<IK::PositionConstraint >        PositionConstraintPtr;

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

class Constraints
{
public:
  std::vector< IKConstraintPtr > ikc_list;

public:
  Constraints() { };

public:
  void push_back(IKConstraintPtr &ptr) {
    ikc_list.push_back(ptr);
  }
  size_t size() { return ikc_list.size(); }
  IKConstraintPtr &at(int i) { return ikc_list.at(i); }
};

typedef std::shared_ptr < Constraints > ConstraintsPtr;
typedef std::vector < std::shared_ptr < Constraints > > ConstraintsPtrList;
#ifdef USE_PRIORITIZED
typedef std::shared_ptr< prioritized_qp_base::Task > TaskPtr;
class Tasks
{
public:
  std::vector< TaskPtr > tasks;

public:
  Tasks() {};

public:
  void push_back(TaskPtr &ptr) {
    tasks.push_back(ptr);
  }
  size_t size() { return tasks.size(); }
  TaskPtr &at(int i) { return tasks.at(i); }
};
typedef std::shared_ptr < Tasks > TasksPtr;
#endif

int solveFullbodyIKLoopFast (const cnoid::BodyPtr& robot,
                             //const std::vector<std::shared_ptr<IK::IKConstraint> >& ikc_list,
                             Constraints &const_,
                             cnoid::VectorX& jlim_avoid_weight_old,
                             const cnoid::VectorX& dq_weight_all,
                             const size_t max_iteration,
                             double wn,
                             int debugLevel)
                             //const size_t max_iteration = 1,
                             //double wn = 1e-6,
                             //int debugLevel = 0)
{
  return fik::solveFullbodyIKLoopFast(robot,
                                      const_.ikc_list,
                                      jlim_avoid_weight_old,
                                      dq_weight_all,
                                      max_iteration,
                                      wn,
                                      debugLevel);
}

#ifdef USE_PRIORITIZED
//std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)>
void taskGeneratorFunc (std::shared_ptr<prioritized_qp_base::Task>& task, int debugLevel)
{
  std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
  if(!taskOSQP){
    task = std::make_shared<prioritized_qp_osqp::Task>();
    taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
  }
  taskOSQP->settings().verbose = debugLevel;
  taskOSQP->settings().max_iter = 4000;
  taskOSQP->settings().eps_abs = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
  taskOSQP->settings().eps_rel = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
  taskOSQP->settings().scaled_termination = true;// avoid too severe termination check
}

int prioritized_solveIKLoop(const std::vector<cnoid::LinkPtr>& variables,
         const ConstraintsPtrList &lst,
         TasksPtr& prevTasks,
         size_t max_iteration, double wn, int debugLevel, double dt)
         //size_t max_iteration = 1,
         //double wn = 1e-6,
         //int debugLevel = 0,
         //double dt = 0.1
{
  std::vector<std::vector < IKConstraintPtr> > ikc_list_;
  for(int i = 0; i < lst.size(); i++) {
    std::vector < IKConstraintPtr> &vec = lst[i]->ikc_list;
    std::vector < IKConstraintPtr> copied_vec;
    for(int j = 0; j < vec.size(); j++) {
      copied_vec.push_back(vec[j]);
    }
    ikc_list_.push_back(copied_vec);
  }
  std::vector< TaskPtr > prevTasks_;
  prioritized_inverse_kinematics_solver::IKParam param;
  param.maxIteration = max_iteration;
  param.wn = wn;
  //param.we = we;
  param.debugLevel = debugLevel;
  param.dt = dt;
  int ret = prioritized_inverse_kinematics_solver::solveIKLoop
    (variables, ikc_list_, prevTasks_, param,
     static_cast < std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> > ( &taskGeneratorFunc)
    );

  prevTasks->tasks.clear();
  for(int i = 0; i < prevTasks_.size(); i++) {
    prevTasks->tasks.push_back(prevTasks_[i]);
  }

  return ret;
}
#endif

class pyIKConstraint : public IK::IKConstraint
{
public:
  using IK::IKConstraint::IKConstraint;

  // trampoline (one for each virtual function)
  virtual bool checkConvergence () override {
    PYBIND11_OVERLOAD_PURE(
      bool, /* Return type */
      IK::IKConstraint,      /* Parent class */
      checkConvergence        /* Name of function in C++ (must match Python name) */
    );
  }
};


PYBIND11_MODULE(IKSolvers, m)
{
    m.doc() = "fullbody inverse kinematics module";

    py::module::import("cnoid.Util");

    py::class_< IK::IKConstraint, IKConstraintPtr, pyIKConstraint > (m, "IKConstraint")
      .def(py::init<>())
      .def_property("debuglevel",
                    (int & (IK::IKConstraint::*)())&IK::IKConstraint::debuglevel,
                    [] (IK::IKConstraint &self, int &in) { self.debuglevel() = in; })
      .def("checkConvergence", &IK::IKConstraint::checkConvergence)
      ;

    py::class_< Constraints, ConstraintsPtr > (m, "Constraints")
      .def(py::init<>())
      .def("size", &Constraints::size)
      .def("at", &Constraints::at)
      .def("__iter__", [](const Constraints &s) { return py::make_iterator(s.ikc_list.begin(), s.ikc_list.end()); },
           py::keep_alive<0, 1>())
      .def("push_back", (void (Constraints::*)(IKConstraintPtr &)) &Constraints::push_back)
      ;

    //py::class_< prioritized_qp_base::Task, TaskPtr > (m, "Task")
    //  .def(py::init<>())
    //  ;
#ifdef USE_PRIORITIZED
    py::class_< Tasks, TasksPtr > (m, "Tasks")
      .def(py::init<>())
      //.def("size", &Tasks::size)
      //.def("at", &Tasks::at)
      //.def("__iter__", [](const Tasks &s) { return py::make_iterator(s.tasks.begin(), s.tasks.end()); },
      //     py::keep_alive<0, 1>())
      //.def("push_back", (void (Tasks::*)(TaskPtr &)) &Tasks::push_back)
      ;
#endif
    py::class_<IK::AngularMomentumConstraint, AngularMomentumConstraintPtr, IK::IKConstraint > (m, "AngularMomentumConstraint")
      .def(py::init<>());

    py::class_<IK::COMConstraint, COMConstraintPtr, IK::IKConstraint > (m, "COMConstraint")
      .def(py::init<>())
      .def_property("A_robot",
                    (cnoid::BodyPtr & (IK::COMConstraint::*)()) &IK::COMConstraint::A_robot,
                    //&IK::COMConstraint::set_A_robot)
                    [](IK::COMConstraint &self, cnoid::BodyPtr &in) { self.A_robot() = in; })
      .def_property("B_robot",
                    (cnoid::BodyPtr & (IK::COMConstraint::*)()) &IK::COMConstraint::B_robot,
                    //&IK::COMConstraint::set_B_robot)
                    [](IK::COMConstraint &self, cnoid::BodyPtr &in) { self.B_robot() = in; })
      .def_property("A_localp",
                    (cnoid::Vector3 & (IK::COMConstraint::*)()) &IK::COMConstraint::A_localp,
                    //&IK::COMConstraint::set_A_localp)
                    [](IK::COMConstraint &self, cnoid::Vector3 &in) { self.A_localp() = in; })
      .def_property("B_localp",
                    (cnoid::Vector3 & (IK::COMConstraint::*)()) &IK::COMConstraint::B_localp,
                    //&IK::COMConstraint::set_B_localp)
                    [](IK::COMConstraint &self, cnoid::Vector3 &in) { self.B_localp() = in; })
      .def_property("eval_R",
                    (cnoid::Matrix3d & (IK::COMConstraint::*)()) &IK::COMConstraint::eval_R,
                    //&IK::COMConstraint::set_eval_R)
                    [](IK::COMConstraint &self, cnoid::Matrix3d &in) { self.eval_R() = in; })
      .def_property("maxError",
                    (cnoid::Vector3 & (IK::COMConstraint::*)()) &IK::COMConstraint::maxError,
                    //&IK::COMConstraint::set_maxError)
                    [](IK::COMConstraint &self, cnoid::Vector3 &in) { self.maxError() = in; })
      .def_property("precision",
                    (cnoid::Vector3 & (IK::COMConstraint::*)()) &IK::COMConstraint::precision,
                    //&IK::COMConstraint::set_precision)
                    [](IK::COMConstraint &self, cnoid::Vector3 &in) { self.precision() = in; })
      .def_property("weight",
                    (cnoid::Vector3 & (IK::COMConstraint::*)()) &IK::COMConstraint::weight,
                    //&IK::COMConstraint::set_weight)
                    [](IK::COMConstraint &self, cnoid::Vector3 &in) { self.weight() = in; })
      ;

    py::class_<IK::COMVelocityConstraint, COMVelocityConstraintPtr, IK::IKConstraint > (m, "COMVelocityConstraint")
      .def(py::init<>());

    py::class_<IK::ClientCollisionConstraint, ClientCollisionConstraintPtr, IK::IKConstraint > (m, "ClientCollisionConstraint")
      .def(py::init<>());

    //py::class_<IK::CollisionConstraint, CollisionConstraintPtr > (m, "CollisionConstraint")
    //  .def(py::init<>());
    py::class_<IK::JointAngleConstraint, JointAngleConstraintPtr, IK::IKConstraint > (m, "JointAngleConstraint")
      .def(py::init<>())
      .def_property("joint",
                    (cnoid::LinkPtr & (IK::JointAngleConstraint::*)()) &IK::JointAngleConstraint::joint,
                    //&IK::JointAngleConstraint::set_joint)
                    [](IK::JointAngleConstraint &self, cnoid::LinkPtr &in) { self.joint() = in; })
      .def_property("targetq",
                    (double & (IK::JointAngleConstraint::*)()) &IK::JointAngleConstraint::targetq,
                    //&IK::JointAngleConstraint::set_targetq)
                    [](IK::JointAngleConstraint &self, double &in) { self.targetq() = in; })
      .def_property("maxError",
                    (double & (IK::JointAngleConstraint::*)()) &IK::JointAngleConstraint::maxError,
                    //&IK::JointAngleConstraint::set_maxError)
                    [](IK::JointAngleConstraint &self, double &in) { self.maxError() = in; })
      .def_property("precision",
                    (double & (IK::JointAngleConstraint::*)()) &IK::JointAngleConstraint::precision,
                    //&IK::JointAngleConstraint::set_precision)
                    [](IK::JointAngleConstraint &self, double &in) { self.precision() = in; })
      .def_property("weight",
                    (double & (IK::JointAngleConstraint::*)()) &IK::JointAngleConstraint::weight,
                    //&IK::JointAngleConstraint::set_weight)
                    [](IK::JointAngleConstraint &self, double &in) { self.weight() = in; })
      ;

    py::class_<IK::JointLimitConstraint, JointLimitConstraintPtr, IK::IKConstraint > (m, "JointLimitConstraint")
      .def(py::init<>());

    py::class_<IK::JointVelocityConstraint, JointVelocityConstraintPtr, IK::IKConstraint > (m, "JointVelocityConstraint")
      .def(py::init<>());

    py::class_<IK::PositionConstraint, PositionConstraintPtr, IK::IKConstraint > (m, "PositionConstraint")
      .def(py::init<>())
      .def_property("A_link",
                    (cnoid::LinkPtr & (IK::PositionConstraint::*)()) &IK::PositionConstraint::A_link,
                    //&IK::PositionConstraint::set_A_link)
                    [](IK::PositionConstraint &self, cnoid::LinkPtr &in) { self.A_link() = in; })
      .def_property("B_link",
                    (cnoid::LinkPtr & (IK::PositionConstraint::*)()) &IK::PositionConstraint::B_link,
                    //&IK::PositionConstraint::set_B_link)
                    [](IK::PositionConstraint &self, cnoid::LinkPtr &in) { self.B_link() = in; })
      .def_property("eval_link",
                    (cnoid::LinkPtr & (IK::PositionConstraint::*)()) &IK::PositionConstraint::eval_link,
                    //&IK::PositionConstraint::set_eval_link)
                    [](IK::PositionConstraint &self, cnoid::LinkPtr &in) { self.eval_link() = in; })
      .def_property("A_localpos",
                    //(cnoid::Position & (IK::PositionConstraint::*)()) &IK::PositionConstraint::A_localpos,
                    //&IK::PositionConstraint::set_A_localpos)
                    [](IK::PositionConstraint& self) -> Isometry3::MatrixType& { return self.A_localpos().matrix(); },
                    [](IK::PositionConstraint& self, Eigen::Ref<const Matrix4RM> in_pos) {
                      Position p(in_pos); self.A_localpos() = p; })
      .def_property("B_localpos",
                    //(cnoid::Position & (IK::PositionConstraint::*)()) &IK::PositionConstraint::B_localpos,
                    //&IK::PositionConstraint::set_B_localpos)
                    [](IK::PositionConstraint& self) -> Isometry3::MatrixType& { return self.B_localpos().matrix(); },
                    [](IK::PositionConstraint& self, Eigen::Ref<const Matrix4RM> in_pos) {
                      Position p(in_pos); self.B_localpos() = p; })
      .def_property("maxError",
                    (cnoid::Vector6 & (IK::PositionConstraint::*)()) &IK::PositionConstraint::maxError,
                    //&IK::PositionConstraint::set_maxError)
                    [](IK::PositionConstraint &self, cnoid::Vector6 &in) { self.maxError() = in; })
      .def_property("precision",
                    (cnoid::Vector6 & (IK::PositionConstraint::*)()) &IK::PositionConstraint::precision,
                    //&IK::PositionConstraint::set_precision)
                    [](IK::PositionConstraint &self, cnoid::Vector6 &in) { self.precision() = in; })
      .def_property("weight",
                    (cnoid::Vector6 & (IK::PositionConstraint::*)()) &IK::PositionConstraint::weight,
                    //&IK::PositionConstraint::set_weight)
                    [](IK::PositionConstraint &self, cnoid::Vector6 &in) { self.weight() = in; })
      .def_property("eval_localR",
                    (cnoid::Matrix3d & (IK::PositionConstraint::*)()) &IK::PositionConstraint::eval_localR,
                    //&IK::PositionConstraint::set_eval_localR)
                    [](IK::PositionConstraint &self, cnoid::Matrix3d &in) { self.eval_localR() = in; })
      ;

    m.def("solveFullbodyIKLoopFast", &solveFullbodyIKLoopFast,
          py::arg("robot"),
          py::arg("constraints"),
          py::arg("jlim_avoid_weight_old"),
          py::arg("dq_weight_all"),
          py::arg("max_iteration") = 1,
          py::arg("wn") = 1e-6,
          py::arg("debug_level") = 0);
#ifdef USE_PRIORITIZED
    m.def("prioritized_solveIKLoop", &prioritized_solveIKLoop,
          py::arg("variables"),
          py::arg("constraints_list"),
          py::arg("prev_tasks"),
          py::arg("max_iteration") = 1,
          py::arg("wn") = 1e-6,
          py::arg("debug_level") = 0,
          py::arg("dt") = 0.1);
#endif
}
