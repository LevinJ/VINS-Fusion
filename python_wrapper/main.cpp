#include <pybind11/pybind11.h>
#include <iostream>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include "../vins_estimator/src/vslam_interface.h"
#include "../vins_estimator/src/estimator/VOStateSubscriber.h"
//#include "../loop_fusion/src/LoopFusion.h"
#include <pybind11/numpy.h>
#include <memory>
namespace py = pybind11;

//class PyEstimator: public Estimator{
//public:
//	PyEstimator():Estimator(){
//
//	}
//
//	void PyinputImage(double t, py::array_t<uint8_t> &  img1){
//		cv::Mat m1 = getMat(img1);
//		cv::Mat m2;
//		inputImage(t, m1, m2);
//	}
//	virtual ~PyEstimator(){
//
//	}
//private:
//	cv::Mat getMat(py::array_t<uint8_t> &  img){
//		auto rows = img.shape(0);
//		auto cols = img.shape(1);
//		auto type = CV_8UC1;
//
//		cv::Mat img2(rows, cols, type, (unsigned char*)img.data());
//		return img2;
//	}
//
//};

//std::shared_ptr<PyEstimator> g_est_ptr;
//PyEstimator g_est;
//void init_estimator(std::string config_file){
//	readParameters(config_file);
//	g_est.setParameter();
//}
//LoopFusion g_loop_fusion;
//std::shared_ptr<LoopFusion> g_lf_ptr;
//void init_loop_fusion(std::string loop_fution_path){
//	g_loop_fusion.init(loop_fution_path, g_est);
////	g_lf_ptr = std::make_shared<LoopFusion>(loop_fution_path, g_est);
//}
//
//void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity){
//	g_est.inputIMU(t, linearAcceleration, angularVelocity);
//}
//
//void inputImage(double t, py::array_t<uint8_t> &  img1){
//	g_est.PyinputImage(t, img1);
//}





int add(int i, int j) {
	std::cout<<"add func is called !!"<<std::endl;
    return i + j;
}
cv::Mat getMat(py::array_t<uint8_t> &  img){
	auto rows = img.shape(0);
	auto cols = img.shape(1);
	auto type = CV_8UC1;

	cv::Mat img2(rows, cols, type, (unsigned char*)img.data());
	return img2;
}
void inputImage_nparr(double t, py::array_t<uint8_t> &  img1){
	cv::Mat m1 = (getMat(img1)).clone();
	inputImage(t, m1);
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
    m.def("init_estimator", &init_estimator);
    m.def("inputIMU", &inputIMU);
    m.def("inputImage_nparr", &inputImage_nparr);
    m.def("init_loop_fusion", &init_loop_fusion);
    m.def("create_map", &create_map);
    m.def("register_vo_callbacks", &register_vo_callbacks);

}
