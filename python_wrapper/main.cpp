#include <pybind11/pybind11.h>
#include <iostream>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include "../vins_estimator/src/vslam_interface.h"
#include "../vins_estimator/src/estimator/VOStateSubscriber.h"
#include "../loop_fusion/src/LPStateSubscriber.h"
#include <pybind11/numpy.h>
#include <memory>
#include "opencv_bind11.h"
namespace py = pybind11;

extern LPStateSubscribers g_lp_state_subscriber;



int add(int i, int j) {
	std::cout<<"add func is called !!"<<std::endl;
    return i + j;
}
cv::Mat getMat(py::array_t<uint8_t> &  img){
	auto rows = img.shape(0);
	auto cols = img.shape(1);
	auto type = CV_8UC1;

	if(cols > 5){
		cv::Mat img2(rows, cols, type, (unsigned char*)img.data());
		return img2;
	}
	return cv::Mat();
}
void inputImage_nparr(double t, py::array_t<uint8_t> &  img1, py::array_t<uint8_t> &  img2){
	cv::Mat m1 = (getMat(img1)).clone();
	cv::Mat m2 = (getMat(img2)).clone();
	inputImage(t, m1, m2);
}

void reloc_image_nparr(double t, py::array_t<uint8_t> &  img1){
	cv::Mat m1 = (getMat(img1)).clone();
	reloc_image(t, m1);
}

void do_it2(){
	g_lp_state_subscriber.lp_info_.P_ = {1, 2, 33};
	g_lp_state_subscriber.lp_info_.ric_ << 10, 20, 30,40,50,60,70,80,90;
	g_lp_state_subscriber.lp_info_.lp_detection_img_ = cv::imread("/home/levin/raw_data/loop_image/0-262-3pnp_match.jpg");

	g_lp_state_subscriber.do_callback();
}

PYBIND11_MODULE(vslam, m) {
    m.doc() = R"pbdoc(
        Pybind11 vslam plugin
        -----------------------

        .. currentmodule:: vslam

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
//    py::class_<Estimator>(m, "Estimator")
//                     .def(py::init<>())
//    				 .def("setParameter", &Estimator::setParameter)
//                     .def("inputIMU", &Estimator::inputIMU);
//    py::class_<PyEstimator, Estimator>(m, "PyEstimator")
//                 .def(py::init<>())
//				 .def("PyinputImage", &PyEstimator::PyinputImage);
//
//    m.def("readParameters", &readParameters);

	py::class_<OdomExtrinsicInfo>(m, "OdomExtrinsicInfo")
					 .def(py::init<>())
					 .def_readwrite("P_", &OdomExtrinsicInfo::P_)
					 .def_readwrite("V_", &OdomExtrinsicInfo::V_)
					 .def_readwrite("R_", &OdomExtrinsicInfo::R_)
					 .def_readwrite("ric_", &OdomExtrinsicInfo::ric_)
					 .def_readwrite("tic_", &OdomExtrinsicInfo::tic_);

	py::class_<LPInfo>(m, "LPInfo")
						 .def(py::init<>())
						 .def_readwrite("P_", &LPInfo::P_)
						 .def_readwrite("lp_detection_img_", &LPInfo::lp_detection_img_)
						 .def_readwrite("lp_matching_img_", &LPInfo::lp_matching_img_)
						 .def_readwrite("R_", &LPInfo::R_)
						 .def_readwrite("find_conn_1_", &LPInfo::find_conn_1_)
						 .def_readwrite("find_conn_2_", &LPInfo::find_conn_2_)
						 .def_readwrite("find_conn_3_", &LPInfo::find_conn_3_)
						 .def_readwrite("bloop_detected_", &LPInfo::bloop_detected_)
						 .def_readwrite("bconn_founded_", &LPInfo::bconn_founded_)
						 .def_readwrite("ric_", &LPInfo::ric_)
						 .def_readwrite("tic_", &LPInfo::tic_);

	m.def("do_it2", &do_it2);

    m.def("init_estimator", &init_estimator);
    m.def("inputIMU", &inputIMU);
    m.def("inputImage_nparr", &inputImage_nparr);
    m.def("init_loop_fusion", &init_loop_fusion);
    m.def("create_map", &create_map);
    m.def("register_vo_callbacks", &register_vo_callbacks);
    m.def("init_reloc", &init_reloc);
    m.def("reloc_image_nparr", &reloc_image_nparr);
    m.def("reloc_callback", &reloc_callback);

}
