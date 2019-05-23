#include <utility>

//
// Created by libaoyu on 19-5-22.
//

#include <OptimizeStructure.h>
#include <include/OptimizeStructure.h>


#if OPT_LIB == G2O
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <eigen3/Eigen/Geometry>


//typedef  g2o::SE3Quat optimize::SE3Type_Impl;
//class optimize::SE3_Parameter_Impl: public g2o::VertexSE3Expmap{
//};
//
//class optimize::SE3ProjectError_Impl:public g2o::EdgeSE3ProjectXYZ{
//
//};
//class optimize::StereoSE3ProjectError_Impl: public g2o::EdgeStereoSE3ProjectXYZ{
//
//};
//class optimize::SE3ProjectOnlyPoseError_Impl: public g2o::EdgeSE3ProjectXYZOnlyPose{
//
//};
//class optimize::StereoSE3ProjectOnlyPoseError_Impl: public g2o::EdgeStereoSE3ProjectXYZOnlyPose{
//
//};
//class optimize::HuberKernel_Impl: public g2o::RobustKernelHuber{
//
//};
//class optimize::Point3_Parameter_Impl: public g2o::VertexSBAPointXYZ{
//
//};
class optimize::Optimizer_Impl:public g2o::SparseOptimizer{
    g2o::BlockSolver_6_3::LinearSolverType * solver_type = nullptr;
    g2o::BlockSolver_6_3 * blockSolver = nullptr;
    g2o::OptimizationAlgorithmLevenberg *lmSolver = nullptr;
public:
    Optimizer_Impl(g2o::BlockSolver_6_3::LinearSolverType *_solver_type, g2o::BlockSolver_6_3 *_bsolver, g2o::OptimizationAlgorithmLevenberg* _solver){
        setAlgorithm(_solver);
        lmSolver = _solver;
        solver_type = _solver_type;
        blockSolver = _bsolver;
    }
    ~Optimizer_Impl() override {
        //these struct leave to g2o to release
//        delete lmSolver;
//        delete blockSolver;
//        delete solver_type;
    }
};
//------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
optimize::SE3_Parameter::SE3_Parameter() {
//    impl = std::make_shared<SE3_Parameter_Impl>();
//    impl = std::allocate_shared<SE3_Parameter_Impl>(Eigen::aligned_allocator<SE3_Parameter_Impl>());
//    impl =Eigen::aligned_allocator<SE3_Parameter_Impl>().allocate(2);
    impl = new SE3_Parameter_Impl();
    impl->setEstimate(EstimateType());
//    estimate = std::make_shared<SE3Type>();
}
void optimize::SE3_Parameter::setEstimate(const EstimateType &se3Type){
//    *estimate = se3Type;
//    SE3Type_Impl *inner = (SE3Type_Impl *)se3Type.get_impl();
//    std::cout<<inner->to_homogeneous_matrix()<<std::endl;
//    g2o::SE3Quat se3Quat(inner->rotation(),inner->translation());
    impl->setEstimate(se3Type);
}

void optimize::SE3_Parameter::setId(int id){
    _id = id;
    impl->setId(id);
}
void optimize::SE3_Parameter::setFixed(bool fixed){
    impl->setFixed(fixed);
}

optimize::Optimizer::Optimizer() {
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    impl = new Optimizer_Impl(linearSolver,solver_ptr,solver);
}

const optimize::SE3_Parameter::EstimateType& optimize::SE3_Parameter::getEstimate(){
    return impl->estimate();
}
//---------------------------------------------------------------
optimize::Optimizer::~Optimizer() {
    for(auto data: idParameter){
        delete(data.second);
    }
    for(auto data: allCost){
        delete(data);
    }
    delete impl;
}

void optimize::Optimizer::setForceStopFlag(bool *flag) {
    assert(impl);
    impl->setForceStopFlag(flag);
}

optimize::Parameter * optimize::Optimizer::getParameter(size_t id) {
//    assert(impl->vertex(id) == parameters[id]->get_impl());
//    impl->vertex()
    auto iter = idParameter.find(id);
    assert(iter != idParameter.end());
    return iter->second;
}
bool optimize::Optimizer::addParameter(optimize::Parameter *parameter) {
    idParameter.insert(std::make_pair(parameter->getId(),parameter));
    return impl->addVertex((g2o::OptimizableGraph::Vertex*)parameter->get_impl());
}

bool optimize::Optimizer::addCost(optimize::Cost *cost) {
    allCost.push_back(cost);
    return impl->addEdge((g2o::OptimizableGraph::Edge *)cost->get_impl());
}
bool optimize::Optimizer::removeParameter(optimize::Parameter *parameter) {
    return impl->removeVertex((g2o::OptimizableGraph::Vertex *)parameter->get_impl());
}
bool optimize::Optimizer::initializeOptimization() {
    return impl->initializeOptimization();
}
bool optimize::Optimizer::initializeOptimization(int level) {
    return impl->initializeOptimization(level);
}
int optimize::Optimizer::optimize(int iterations) {
    return impl->optimize(iterations);
}

int optimize::Optimizer::numCost(){
    return impl->edges().size();
}
int optimize::Optimizer::numParameter(){
    return impl->vertices().size();
}
//---------------------------------------------------------------
void optimize::SE3ProjectError::setParameter(size_t i, Parameter *parameter) {
    impl->setVertex(i, static_cast<g2o::OptimizableGraph::Vertex*>(parameter->get_impl()));
}

optimize::SE3ProjectError::SE3ProjectError(double fx, double fy, double cx, double cy) {

//    impl = std::allocate_shared<SE3ProjectError_Impl>(Eigen::aligned_allocator<SE3ProjectError_Impl>());
//    impl =Eigen::aligned_allocator<SE3ProjectError_Impl>().allocate(1);
    impl = new SE3ProjectError_Impl();
    impl->fx = fx;
    impl->fy = fy;
    impl->cx = cx;
    impl->cy = cy;
}
void optimize::SE3ProjectError::setInformation(const optimize::SE3ProjectError::InformationType &information) {
    impl->setInformation(information);
}
void optimize::SE3ProjectError::setMeasurement(const optimize::SE3ProjectError::Measurement &measurement) {
    impl->setMeasurement(measurement);
}

void optimize::SE3ProjectError::setKernel(optimize::Kernel *kernel) {
    kernel = kernel;
    if(kernel)
    impl->setRobustKernel((g2o::RobustKernel*)kernel->get_impl());
    else
        impl->setRobustKernel(0);
}

double optimize::SE3ProjectError::chi2()const{
    return impl->chi2();
}
bool optimize::SE3ProjectError::isDepthPositive(){
    return impl->isDepthPositive();
}
void optimize::SE3ProjectError::computeError(){
    impl->computeError();
}

void optimize::SE3ProjectError::setLevel(int level) {
    impl->setLevel(level);
}

//---------------------------------------------------------------
optimize::StereoSE3ProjectError::StereoSE3ProjectError(double fx, double fy, double cx, double cy, double bf) {
//    impl = std::allocate_shared<StereoSE3ProjectError_Impl>(Eigen::aligned_allocator<StereoSE3ProjectError_Impl>());
//    impl =Eigen::aligned_allocator<StereoSE3ProjectError_Impl>().allocate(1);
    impl = new StereoSE3ProjectError_Impl();
    impl->fx = fx;
    impl->fy = fy;
    impl->cx = cx;
    impl->cy = cy;
    impl->bf = bf;
}

void optimize::StereoSE3ProjectError::setParameter(size_t i, Parameter *parameter) {
    impl->setVertex(i, static_cast<g2o::OptimizableGraph::Vertex*>(parameter->get_impl()));
}

void
optimize::StereoSE3ProjectError::setInformation(const optimize::StereoSE3ProjectError::InformationType &information) {

    impl->setInformation(information);
}
void optimize::StereoSE3ProjectError::setMeasurement(const optimize::StereoSE3ProjectError::Measurement &measurement) {
    impl->setMeasurement(measurement);
}
void optimize::StereoSE3ProjectError::setKernel(optimize::Kernel *kernel) {
    if(kernel)
    impl->setRobustKernel((g2o::RobustKernel*)kernel->get_impl());
    else
        impl->setRobustKernel(0);
}

double optimize::StereoSE3ProjectError::chi2()const{
    return impl->chi2();
}
bool optimize::StereoSE3ProjectError::isDepthPositive(){
    return impl->isDepthPositive();
}
void optimize::StereoSE3ProjectError::computeError(){
    impl->computeError();
}

void optimize::StereoSE3ProjectError::setLevel(int level) {
    impl->setLevel(level);
}
//------------------------------------------------------------------------------

optimize::SE3ProjectOnlyPoseError::SE3ProjectOnlyPoseError(double fx, double fy, double cx, double cy) {
//    impl = std::allocate_shared<SE3ProjectOnlyPoseError_Impl>(Eigen::aligned_allocator<SE3ProjectOnlyPoseError_Impl>());
//    impl =Eigen::aligned_allocator<SE3ProjectOnlyPoseError_Impl>().allocate(1);
    impl = new SE3ProjectOnlyPoseError_Impl();
    impl->fx = fx;
    impl->fy = fy;
    impl->cx = cx;
    impl->cy = cy;
}

void optimize::SE3ProjectOnlyPoseError::setParameter(size_t i, Parameter *parameter) {
    impl->setVertex(i, static_cast<g2o::OptimizableGraph::Vertex*>(parameter->get_impl()));
}
void optimize::SE3ProjectOnlyPoseError::setInformation(const optimize::SE3ProjectOnlyPoseError::InformationType &information) {
    impl->setInformation(information);
}
void optimize::SE3ProjectOnlyPoseError::setMeasurement(const optimize::SE3ProjectOnlyPoseError::Measurement &measurement) {
    impl->setMeasurement(measurement);
}

void optimize::SE3ProjectOnlyPoseError::setKernel(optimize::Kernel *kernel) {
    if(kernel)
        impl->setRobustKernel((g2o::RobustKernel*)kernel->get_impl());
    else
        impl->setRobustKernel(0);
}

double &optimize::SE3ProjectOnlyPoseError::Xw(int i){
    return impl->Xw[i];
}
double optimize::SE3ProjectOnlyPoseError::chi2()const{
    return impl->chi2();
}
void optimize::SE3ProjectOnlyPoseError::computeError(){
    impl->computeError();
}

void optimize::SE3ProjectOnlyPoseError::setLevel(int level) {
    impl->setLevel(level);
}
//------------------------------------------------------------------------------

optimize::StereoSE3ProjectOnlyPoseError::StereoSE3ProjectOnlyPoseError(double fx, double fy, double cx, double cy,double bf){
//    impl = std::allocate_shared<StereoSE3ProjectOnlyPoseError_Impl>(Eigen::aligned_allocator<StereoSE3ProjectOnlyPoseError_Impl>());
//    impl =Eigen::aligned_allocator<StereoSE3ProjectOnlyPoseError_Impl>().allocate(1);
    impl = new StereoSE3ProjectOnlyPoseError_Impl();
    impl->fx = fx;
    impl->fy = fy;
    impl->cx = cx;
    impl->cy = cy;
    impl->bf = bf;
}

void optimize::StereoSE3ProjectOnlyPoseError::setParameter(size_t i, Parameter *parameter) {
    impl->setVertex(i, static_cast<g2o::OptimizableGraph::Vertex*>(parameter->get_impl()));
}
void optimize::StereoSE3ProjectOnlyPoseError::setInformation(const optimize::StereoSE3ProjectOnlyPoseError::InformationType &information) {
    impl->setInformation(information);
}
void optimize::StereoSE3ProjectOnlyPoseError::setMeasurement(const optimize::StereoSE3ProjectOnlyPoseError::Measurement &measurement) {
    impl->setMeasurement(measurement);
}

void optimize::StereoSE3ProjectOnlyPoseError::setKernel(optimize::Kernel *kernel) {
    if(kernel)
    impl->setRobustKernel((g2o::RobustKernel*)kernel->get_impl());
    else
        impl->setRobustKernel(0);
}
double &optimize::StereoSE3ProjectOnlyPoseError::Xw(int i){
    return impl->Xw[i];
}
double optimize::StereoSE3ProjectOnlyPoseError::chi2()const{
    return impl->chi2();
}
void optimize::StereoSE3ProjectOnlyPoseError::computeError(){
    impl->computeError();
}

void optimize::StereoSE3ProjectOnlyPoseError::setLevel(int level) {
    impl->setLevel(level);
}

//------------------------------------------------------------------------------
void optimize::HuberKernel::setDelta(double delta) {
    impl->setDelta(delta);
}

//-----------------------------------------------------------------------
optimize::Point3_Parameter::Point3_Parameter(){
//    impl = std::allocate_shared<Point3_Parameter_Impl>(Eigen::aligned_allocator<Point3_Parameter_Impl>());
//    impl =Eigen::aligned_allocator<Point3_Parameter_Impl>().allocate(1);
    impl = new Point3_Parameter_Impl();
}
const optimize::Point3_Parameter::EstimateType& optimize::Point3_Parameter::getEstimate(){
    return impl->estimate();
}
void optimize::Point3_Parameter::setEstimate(const EstimateType &se3Type){
    impl->setEstimate(se3Type);
}
void optimize::Point3_Parameter::setId(int id) {
    _id = id;
    impl->setId(id);
}
void optimize::Point3_Parameter::setMarginalized(bool ifmarg) {
    impl->setMarginalized(ifmarg);
}

//----------------------------------------------------------------------
optimize::HuberKernel::HuberKernel(){
//    impl = std::allocate_shared<HuberKernel_Impl>(Eigen::aligned_allocator<HuberKernel_Impl>());
//    impl =Eigen::aligned_allocator<HuberKernel_Impl>().allocate(1);
    impl = new HuberKernel_Impl();
}
#elif OPT_LIB == CERES
#endif