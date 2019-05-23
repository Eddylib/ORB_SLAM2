//
// Created by libaoyu on 19-5-22.
//

#ifndef ORB_SLAM2_OPTIMIZESTRUCTURE_H
#define ORB_SLAM2_OPTIMIZESTRUCTURE_H
#define G2O_OPT



#include <vector>
#include <memory>
#include <map>
#include <eigen3/Eigen/Dense>
#define OPT_LIB G2O
#if OPT_LIB == G2O
namespace g2o {
    class SE3Quat;
    class VertexSE3Expmap;
    class EdgeSE3ProjectXYZ;
    class EdgeStereoSE3ProjectXYZ;
    class EdgeSE3ProjectXYZOnlyPose;
    class EdgeStereoSE3ProjectXYZOnlyPose;
    class RobustKernelHuber;
    class VertexSBAPointXYZ;
}
#endif

namespace optimize{
        //-----------------------------------types-----------------------------------
#if OPT_LIB == G2O
        using SE3Type = g2o::SE3Quat;
        using  SE3_Parameter_Impl = g2o::VertexSE3Expmap ;
        using Point3_Parameter_Impl = g2o::VertexSBAPointXYZ;
        using HuberKernel_Impl = g2o::RobustKernelHuber;
        using SE3ProjectError_Impl = g2o::EdgeSE3ProjectXYZ;
        using StereoSE3ProjectError_Impl = g2o::EdgeStereoSE3ProjectXYZ;
        using SE3ProjectOnlyPoseError_Impl = g2o::EdgeSE3ProjectXYZOnlyPose;
        using StereoSE3ProjectOnlyPoseError_Impl = g2o::EdgeStereoSE3ProjectXYZOnlyPose;
#endif

        // -----------------------------------parameters-----------------------------------
        class Parameter{
        protected:
            int _id = -1;
        public:
            virtual void *get_impl() = 0;
            virtual int getId(){return _id;};
            virtual ~Parameter(){}
        };

        class SE3_Parameter:public Parameter{
        public:
            using EstimateType = SE3Type;
        private:
//        std::shared_ptr<SE3_Parameter_Impl> impl;
            SE3_Parameter_Impl* impl;
//        std::shared_ptr<SE3Type> estimate;
        public:
            SE3_Parameter();
            void setEstimate(const EstimateType &estimate);
            const EstimateType& getEstimate();
            void setId(int id);
            void setFixed(bool fixed);
            void *get_impl() override {return impl;}


        };
        class SIM3_Parameter:public Parameter{

        };
        class Point3_Parameter:public Parameter{
//        std::shared_ptr<Point3_Parameter_Impl>impl;
            Point3_Parameter_Impl *impl;
        public:
            using EstimateType = Eigen::Matrix<double,3,1>;
            Point3_Parameter();
            void *get_impl(){return impl;}
            void setEstimate(const EstimateType &estimate);
            const EstimateType &getEstimate();
            void setId(int id);
            void setMarginalized(bool ifmarg);
        };

        //-----------------------------------kernel-----------------------------------
        class Kernel{
        public:
            virtual void *get_impl() = 0;
            virtual ~Kernel()= default;
        };

        class HuberKernel:public Kernel{
//        std::shared_ptr<HuberKernel_Impl> impl;
            HuberKernel_Impl *impl;
        public:
            HuberKernel();
            void *get_impl() override {return impl;}
            void setDelta(double delta);
        };

        //----------------------------------- costs-----------------------------------
        class Cost{
        public:
            virtual void *get_impl() = 0;
            virtual void setKernel(Kernel *) = 0;
            Kernel *kernel = nullptr;
            virtual  ~Cost(){delete(kernel);}
        };
        class SE3ProjectError: public Cost{
//        std::shared_ptr<SE3ProjectError_Impl> impl;
            SE3ProjectError_Impl *impl;
        public:
            using InformationType = Eigen::Matrix2d;
            using Measurement = Eigen::Matrix<double,2,1>;
            SE3ProjectError(double fx, double fy, double cx, double cy);

            void setParameter(size_t i, Parameter *parameter);
            void setInformation(const InformationType &information);
            void setMeasurement(const Measurement &measurement);
            void setKernel(Kernel *kernel);
            void *get_impl(){return impl;}
            void computeError();
            void setLevel(int level);
            double chi2() const;
            bool isDepthPositive();
        };

        class StereoSE3ProjectError:public Cost{
//        std::shared_ptr<StereoSE3ProjectError_Impl> impl;
            StereoSE3ProjectError_Impl *impl;
        public:
            using InformationType = Eigen::Matrix3d;
            using Measurement = Eigen::Matrix<double,3,1>;
            StereoSE3ProjectError(double fx, double fy, double cx, double cy, double bf);
            void setParameter(size_t i, Parameter *parameter);
            void setInformation(const InformationType &information);
            void setMeasurement(const Measurement &measurement);
            void *get_impl(){return impl;}
            void setKernel(Kernel *kernel);
            void computeError();
            void setLevel(int level);
            double chi2() const;
            bool isDepthPositive();


        };
        class SE3ProjectOnlyPoseError: public Cost{
//        std::shared_ptr<SE3ProjectOnlyPoseError_Impl> impl;
            SE3ProjectOnlyPoseError_Impl *impl;
        public:
            using InformationType = Eigen::Matrix2d;
            using Measurement = Eigen::Matrix<double,2,1>;
            SE3ProjectOnlyPoseError(double fx, double fy, double cx, double cy);

            void setParameter(size_t i, Parameter *parameter);
            void setInformation(const InformationType &information);
            void setMeasurement(const Measurement &measurement);
            void setKernel(Kernel *kernel) override;
            void *get_impl() override {return impl;}
            double &Xw(int i);
            void computeError();
            void setLevel(int level);
            double chi2() const;
        };
        class StereoSE3ProjectOnlyPoseError:public Cost{
//        std::shared_ptr<StereoSE3ProjectOnlyPoseError_Impl> impl;
            StereoSE3ProjectOnlyPoseError_Impl *impl;
        public:
            using InformationType = Eigen::Matrix3d;
            using Measurement = Eigen::Matrix<double,3,1>;
            StereoSE3ProjectOnlyPoseError(double fx, double fy, double cx, double cy, double bf);
            void setParameter(size_t i, Parameter *parameter);
            void setInformation(const InformationType &information);
            void setMeasurement(const Measurement &measurement);
            void *get_impl(){return impl;}
            void setKernel(Kernel *kernel);
            double &Xw(int i);
            void computeError();
            void setLevel(int level);
            double chi2() const;

        };
        //----------------------------------- optimizer-----------------------------------
        class Optimizer_Impl;
        class Optimizer{
            Optimizer_Impl *impl;
            std::map<int,Parameter *> idParameter;
            std::vector<Cost *> allCost;
        public:
            void setForceStopFlag(bool* flag);
            Optimizer();
            ~Optimizer();

            void *get_impl() {return impl;}
            bool addParameter(Parameter *parameter);
            bool removeParameter(Parameter *parameter);
            bool addCost(Cost *cost);
            Parameter * getParameter(size_t id);
            bool initializeOptimization();
            bool initializeOptimization(int level);
            int optimize(int iterations);
            int numCost();
            int numParameter();
        };
}
#endif //ORB_SLAM2_OPTIMIZESTRUCTURE_H
