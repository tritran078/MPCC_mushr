#include <pybind11/pybind11.h>
#include <pybind11/stl.h>          // converts std::vector <-> Python list
#include "mpcc_controller.h"

namespace py = pybind11;

// Python:
//   import mpcc_py
//   cost = mpcc_py.CostParam()          # starts with the defaults from MPCC.h
//   cost.gamma = 10.0
//   bounds = mpcc_py.NlpBounds()        # starts with the defaults from nlp_solve.h
//   bounds.vx_max = 3.0
//   ctrl = mpcc_py.MpccController(wp_x, wp_y, 0.02, 40, cost, bounds, 5)
//   steer, speed = ctrl.step([px, py, yaw, vx, vy, yaw_rate])
PYBIND11_MODULE(mpcc_py, m) {

    py::class_<CostParam>(m, "CostParam")
        .def(py::init<>())
        .def_readwrite("q_c",      &CostParam::q_c)        // contour error weight
        .def_readwrite("q_l",      &CostParam::q_l)        // lag error weight
        .def_readwrite("gamma",    &CostParam::gamma)      // progress reward
        .def_readwrite("R_steer",  &CostParam::R_steer)    // input size penalties
        .def_readwrite("R_PWM",    &CostParam::R_PWM)
        .def_readwrite("R_delta",  &CostParam::R_delta)
        .def_readwrite("Rt_steer", &CostParam::Rt_steer)   // input change penalties
        .def_readwrite("Rt_PWM",   &CostParam::Rt_PWM)
        .def_readwrite("Rt_delta", &CostParam::Rt_delta);

    py::class_<NlpBounds>(m, "NlpBounds")
        .def(py::init<>())
        .def_readwrite("pwm_min",          &NlpBounds::pwm_min)
        .def_readwrite("pwm_max",          &NlpBounds::pwm_max)
        .def_readwrite("steer_max",        &NlpBounds::steer_max)
        .def_readwrite("dtheta_min",       &NlpBounds::dtheta_min)
        .def_readwrite("dtheta_max",       &NlpBounds::dtheta_max)
        .def_readwrite("vx_min",           &NlpBounds::vx_min)
        .def_readwrite("vx_max",           &NlpBounds::vx_max)
        .def_readwrite("track_half_width", &NlpBounds::track_half_width)
        .def_readwrite("car_margin",       &NlpBounds::car_margin);
        // theta_max is not exposed: the controller always sets it to the track length

    py::class_<MpccController>(m, "MpccController")
        .def(py::init<const std::vector<double>&, const std::vector<double>&, double, int,
                      const CostParam&, const NlpBounds&, int>(),
             py::arg("waypoints_x"), py::arg("waypoints_y"), py::arg("dt"), py::arg("horizon"),
             py::arg("cost_param") = CostParam(),
             py::arg("bounds") = NlpBounds(),
             py::arg("speed_lookahead") = 5)
        .def("step", &MpccController::step, py::arg("car_state"))
        .def("get_theta", &MpccController::get_theta)
        .def("get_track_length", &MpccController::get_track_length);
}
