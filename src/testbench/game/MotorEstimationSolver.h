// Created by lars on 17.11.15.
//

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <iostream>

#include "testbench/game/adept_autodiff_cost_function.h"
#include "testbench/game/adept_autodiff_local_parameterization.h"
#include "testbench/game/motor_parameterization.h"
#include "testbench/game/vsr/vsr.h"

using vsr::nga::Flat;
using vsr::cga::Scalar;
using vsr::cga::Vector;
using vsr::cga::Bivector;
using vsr::cga::Point;
using vsr::cga::DualLine;
using vsr::cga::DualPlane;
using vsr::cga::Motor;
using vsr::cga::Rotor;
using vsr::cga::Translator;
using vsr::cga::Origin;
using vsr::cga::Infinity;
using vsr::cga::TangentVector;
using vsr::cga::DirectionVector;
using vsr::cga::DirectionTrivector;
using vsr::cga::Circle;
using vsr::cga::Cir;
using vsr::cga::Pnt;
using vsr::cga::Drv;
using vsr::cga::Dll;
using vsr::cga::Dlp;
using vsr::cga::Mot;
using vsr::cga::Tnv;
using vsr::nga::Op;


namespace game {

    class MotorEstimationSolver {
    public:
        MotorEstimationSolver() {}
        MotorEstimationSolver(const MotorEstimationSolver &motor_estimation_solver) {}
        MotorEstimationSolver(const Mot &motor) : motor_(motor) {}

        struct TangentVectorPointAngleErrorCostFunctor {
            TangentVectorPointAngleErrorCostFunctor(const Tnv &a, const Tnv &b)
                    : a_(a), b_(b) {}

            template <typename T>
            bool operator()(const T *const motor, T *residual) const {
                Motor<T> M(motor);
                TangentVector<T> a(a_);
                TangentVector<T> b(b_);
                TangentVector<T> c = a.spin(M);

                Point<T> pb = b / (-Infinity<T>(T(1.0)) <= b);
                Point<T> pc = c / (-Infinity<T>(T(1.0)) <= c);

                for (int i = 0; i < 3; ++i) {
                    residual[i] = pc[i] - pb[i];
                }

                DirectionVector<T> db = -(Infinity<T>(T(1.0)) <= b) ^ Infinity<T>(T(1.0));
                DirectionVector<T> dc = -(Infinity<T>(T(1.0)) <= c) ^ Infinity<T>(T(1.0));

                // residual[3] = (Vector<T>{db[0], db[1], db[2]} <= Vector<T>{dc[0],
                // dc[1], dc[2]})[0];

                // for (int i = 3; i < 6; ++i) {
                //   residual[i] = dc[i] - db[i];
                // }

                return true;
            }

        private:
            const Tnv a_;
            const Tnv b_;
        };

        template <typename T> static DualLine<T> log(const Motor<T> &m) {
            DualLine<T> q(m);
            Scalar<T> ac{acos(m[0])};
            Scalar<T> den{sin(ac[0]) / ac[0]};
            Scalar<T> den2{ac * ac * den};

            if (den2[0] > T(0.0)) {
                DualLine<T> b = Bivector<T>(m) / den;
                DualLine<T> c_perp = -b * DirectionTrivector<T>(m) / den2;
                DualLine<T> c_para = -b * DualLine<T>(b * q) / den2;
                return b + c_perp + c_para;
            } else {
                return q;
            }
        }

        template <typename T> static Motor<T> exp(const DualLine<T> &l) {
            const T m0_arr[8] = {T(1.0), T(0.0), T(0.0), T(0.0),
                                 T(0.0), T(0.0), T(0.0), T(0.0)};
            Motor<T> m;
            game::MotorFromBivectorGenerator()(&m0_arr[0], l.begin(), m.data());
            return m;
        }

        struct DualPlaneAngleErrorCostFunctor {
            DualPlaneAngleErrorCostFunctor(const Dlp &a, const Dlp &b) : a_(a), b_(b) {}

            template <typename T>
            bool operator()(const T *const motor, T *residual) const {
                Motor<T> M(motor);
                DualPlane<T> a(a_);
                DualPlane<T> b(b_);
                DualPlane<T> c = a.spin(M);

                Origin<T> no{T(1.0)};
                Infinity<T> ni{T(1.0)};

                Motor<T> X = exp(Scalar<T>{1.0} * log(Motor<T>(c / b)));
                Rotor<T> R{X[0], X[1], X[2], X[3]};
                Vector<T> t = Scalar<T>{-2.0} * (no <= X) / R;

                T scale{T(1.0)};
                if (abs(T(1.0) - X[0]) > T(0.0)) {
                    Bivector<T> B{R[1], R[2], R[3]};
                    B = B.unit();
                    Vector<T> v = Op::project(t, B);
                    residual[0] = v[0];
                    residual[1] = v[1];
                    residual[2] = v[2];
                } else {
                    residual[0] = t[0];
                    residual[1] = t[1];
                    residual[2] = t[2];
                }

                // residual[0] = t[0];
                // residual[1] = t[1];
                // residual[2] = t[2];
                residual[3] = T(1.0) - X[0]; // 1 - cos(theta)

                // residual[0] = X[0];
                // T distance = t.norm();
                // residual[1] = distance;

                return true;
            }

        private:
            const Dlp a_;
            const Dlp b_;
        };

        struct CircleCommutatorCostFunctor {
            CircleCommutatorCostFunctor(const Cir &a, const Cir &b) : a_(a), b_(b) {}

            template <typename T>
            bool operator()(const T *const motor, T *residual) const {
                Motor<T> M(motor);
                Circle<T> a(a_);
                Circle<T> b(b_);
                Circle<T> c = a.spin(M);
                Circle<T> d =
                        ((c.dual() * b.dual() - b.dual() * c.dual()) * Scalar<T>(0.5))
                                .undual();

                for (int i = 0; i < 10; ++i) {
                    residual[i] = d[i];
                }

                return true;
            }

        private:
            const Cir a_;
            const Cir b_;
        };

        struct CircleDifferenceCostFunctor {
            CircleDifferenceCostFunctor(const Cir &a, const Cir &b) : a_(a), b_(b) {}

            template <typename T>
            bool operator()(const T *const motor, T *residual) const {
                Motor<T> M(motor);
                Circle<T> a(a_);
                Circle<T> b(b_);
                Circle<T> c = a.spin(M);

                for (int i = 0; i < 10; ++i) {
                    residual[i] = c[i] - b[i];
                }

                return true;
            }

        private:
            const Cir a_;
            const Cir b_;
        };

        struct DirectionVectorInnerProductCostFunctor {
            DirectionVectorInnerProductCostFunctor(const Drv &a, const Drv &b)
                    : a_(a), b_(b) {}

            template <typename T>
            bool operator()(const T *const motor, T *residual) const {
                Motor<T> M(motor);
                DirectionVector<T> a(a_);
                DirectionVector<T> b(b_);
                DirectionVector<T> c = a.spin(M);

                residual[0] =
                        T(1.0) - ((Origin<T>{T(1.0)} <= b) * ~(Origin<T>{T(1.0)} <= c))[0];

                return true;
            }

        private:
            const Drv a_;
            const Drv b_;
        };

        struct DualPlaneDifferenceFunctor {
            DualPlaneDifferenceFunctor(const Dlp &a, const Dlp &b) : a_(a), b_(b) {}

            template <typename T>
            bool operator()(const T *const motor, T *residual) const {
                Motor<T> M(motor);
                DualPlane<T> a(a_);
                DualPlane<T> b(b_);
                DualPlane<T> c = a.spin(M);

                for (int i = 0; i < 4; ++i) {
                    residual[i] = c[i] - b[i];
                }

                return true;
            }

        private:
            const Dlp a_;
            const Dlp b_;
        };

        struct DualPlaneInnerProductCostFunctor {
            DualPlaneInnerProductCostFunctor(const Dlp &a, const Dlp &b)
                    : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualPlane<T> a(a_);
                DualPlane<T> b(b_);
                DualPlane<T> c = a.spin(M);
                residual[0] = T(1.0) - (b * ~c)[0];
                return true;
            }

        private:
            const Dlp a_;
            const Dlp b_;
        };

        struct FlatPointCommutatorCostFunctor {
            FlatPointCommutatorCostFunctor(const Pnt &a, const Pnt &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                DualLine<T> d = (c * b - b * c) * Scalar<T>{0.5};
                for (int i = 0; i < 6; ++i) {
                    residual[i] = d[i];
                }
                return true;
            }

        private:
            const Dlp a_;
            const Dlp b_;
        };

        struct DualPlaneCommutatorCostFunctor {
            DualPlaneCommutatorCostFunctor(const Dlp &a, const Dlp &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualPlane<T> a(a_);
                DualPlane<T> b(b_);
                DualPlane<T> c = a.spin(M);
                DualLine<T> d = c * ~b;

                residual[0] = d[0];
                residual[1] = d[1];
                residual[2] = d[2];

                // Bivector<T> A{d[0], d[1], d[2]};
                // if( A.norm() > T(0.00000001) ) {
                //   Vector<T> f{d[3], d[4], d[5]};
                //   Vector<T> b_d = Op::reject(f, A.unit());
                //   residual[3] = b_d[0];
                //   residual[4] = b_d[1];
                //   residual[5] = b_d[2];
                // }
                // else {
                residual[3] = d[3];
                residual[4] = d[4];
                residual[5] = d[5];
                // }

                // for (int i = 0; i < 6; ++i) {
                //   residual[i] = d[i];
                // }

                return true;
            }

        private:
            const Dlp a_;
            const Dlp b_;
        };

        struct LineInnerProductCostFunctor {
            LineInnerProductCostFunctor(const Dll &a, const Dll &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                residual[0] = T(1.0) - (b * ~c)[0];
                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineDualAngleCostFunctor {
            LineDualAngleCostFunctor(const Dll &a, const Dll &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = ~a.spin(M);
                Motor<T> d = b * c;

                residual[0] = d[0];
                residual[1] = d[7];
                // residual[2] = T(1.0);

                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineAntiCommutatorCostFunctor {
            LineAntiCommutatorCostFunctor(const Dll &a, const Dll &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                DualLine<T> d = c * ~b;

                // residual[0] = T(1.0) - d[0];
                residual[0] = d[7];

                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineProjectedCommutatorCostFunctor {
            LineProjectedCommutatorCostFunctor(const Dll &a, const Dll &b)
                    : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                DualLine<T> d = c * ~b;

                // Point<T> p =
                //   Flat::location(d, Vector<T>{T(0.0),T(0.0), T(0.0)}.null(), true) *
                //   Scalar<T>{T(0.5)};
                // p = p.unit();
                // Motor<T> Tr{T(1.0), T(0.0), T(0.0), T(0.0), -p[0], -p[1], -p[2],
                // T(0.0)};
                // DualLine<T> dt = d.spin(~Tr);
                Origin<T> no{T(1.0)};
                Infinity<T> ni{T(1.0)};
                Motor<T> X = exp(Scalar<T>{0.5} * log(c / b));
                Rotor<T> R{X[0], X[1], X[2], X[3]};
                Vector<T> t = Scalar<T>{-2.0} * (no <= X) / R;

                residual[0] = d[0];
                residual[1] = d[1];
                residual[2] = d[2];

                Bivector<T> A{d[0], d[1], d[2]};
                if (A.norm() > T(0.00000001)) {
                    Vector<T> f{d[3], d[4], d[5]};
                    Vector<T> b_d = Op::reject(f, A.unit());
                    residual[3] = b_d[0];
                    residual[4] = b_d[1];
                    residual[5] = b_d[2];
                } else {
                    residual[3] = d[3];
                    residual[4] = d[4];
                    residual[5] = d[5];
                }


                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineCommutatorCostFunctor {
            LineCommutatorCostFunctor(const Dll &a, const Dll &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                // DualLine<T> d = (b * c - c * b) * Scalar<T>{0.5};

                DualLine<T> d = c * ~b;

                for (int i = 0; i < 6; ++i) {
                    residual[i] = d[i];
                }

                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineAngleDistanceNormCostFunctor {
            LineAngleDistanceNormCostFunctor(const Dll &a, const Dll &b)
                    : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);

                Origin<T> no{T(1.0)};
                Infinity<T> ni{T(1.0)};

                Motor<T> X = exp(Scalar<T>{0.5} * log(c / b));
                Rotor<T> R{X[0], X[1], X[2], X[3]};
                Vector<T> t = Scalar<T>{-2.0} * (no <= X) / R;

                T distance;
                if (abs(T(1.0) - X[0]) > T(0.0)) {
                    Bivector<T> B{R[1], R[2], R[3]};
                    B = B.unit();
                    Vector<T> w = Op::reject(t, B);
                    distance = w.norm();
                    residual[0] = distance;
                } else {
                    distance = t.norm();
                    residual[0] = distance;
                }

                residual[1] = T(1.0) - X[0];
                // Bivector<T> biv{R[1], R[2], R[3]};
                // T theta = atan2(R[0], biv.norm());
                // residual[1] = theta;

                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineAngleDistanceCostFunctor {
            LineAngleDistanceCostFunctor(const Dll &a, const Dll &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                DualLine<T> d = c - b;

                residual[0] = d[0];
                residual[1] = d[1];
                residual[2] = d[2];

                // Origin<T> no{T(1.0)};
                // Infinity<T> ni{T(1.0)};
                // Motor<T> X = (c * ~b);
                // Bivector<T> A{X[1], X[2], X[3]};
                // Rotor<T> R{X[0], X[1], X[2], X[3]};
                // Vector<T> t = Scalar<T>{1.0} * (no <= X) / R;

                // if (A.norm() > T(1e-6)) {
                // Vector<T> w = Op::reject(t, A.unit());
                // residual[3] = w[0];
                // residual[4] = w[1];
                // residual[5] = w[2];
                // } else {
                //   residual[3] = t[0];
                //   residual[4] = t[1];
                //   residual[5] = t[2];
                // }

                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineCorrespondencesCostFunctor {
            LineCorrespondencesCostFunctor(const Dll &a, const Dll &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                DualLine<T> d = c - b;
                residual[0] = d[0];
                residual[1] = d[1];
                residual[2] = d[2];
                residual[3] = d[3];
                residual[4] = d[4];
                residual[5] = d[5];
                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct LineProjectedCorrespondencesCostFunctor {
            LineProjectedCorrespondencesCostFunctor(const Dll &a, const Dll &b)
                    : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                DualLine<T> a(a_);
                DualLine<T> b(b_);
                DualLine<T> c = a.spin(M);
                DualLine<T> d = c - b;

                residual[3] = d[0];
                residual[4] = d[1];
                residual[5] = d[2];

                Bivector<T> A{d[0], d[1], d[2]};
                if (A.norm() > T(0.0)) {
                    Vector<T> f{d[3], d[4], d[5]};
                    Vector<T> b_d = Op::reject(f, A.unit());
                    residual[0] = b_d[0];
                    residual[1] = b_d[1];
                    residual[2] = b_d[2];
                } else {
                    residual[0] = d[3];
                    residual[1] = d[4];
                    residual[2] = d[5];
                }

                // Point<T> p =
                //   Flat::location(d, Vector<T>{T(0.0),T(0.0), T(0.0)}.null(), true) *
                //   Scalar<T>{T(0.5)};
                // Motor<T> Tr{T(1.0), T(0.0), T(0.0), T(0.0), -p[0], -p[1], -p[2],
                // T(0.0)};

                // DualLine<T> dt = d.spin(~Tr);

                // for (int i = 0; i < b.Num; ++i) {
                //   // residual[i] = c[i] - b[i];
                //   // residual[i] = dt[i];
                //   residual[i] = d[i];
                // }

                return true;
            }

        private:
            const Dll a_;
            const Dll b_;
        };

        struct PointDistanceCostFunctor {
            PointDistanceCostFunctor(const Pnt &a, const Pnt &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                Point<T> a(a_);
                Point<T> b(b_);
                Point<T> c = a.spin(M);

                // T cos_distance = cos(sqrt(T(-2.0) * minus_half_distance_squared));
                // residual[0] = cos_distance;

                T minus_half_distance_squared = (c <= b)[0];
                residual[0] = minus_half_distance_squared;

                return true;
            }

        private:
            const Pnt a_;
            const Pnt b_;
        };

        struct GaalopPointCorrespondencesCostFunctor {
            GaalopPointCorrespondencesCostFunctor(const Pnt &a, const Pnt &b)
                    : a_(a), b_(b) {}

            template <typename T>
            void Calculate(const T &a1, const T &a2, const T &a3, const T &a4,
                           const T &a5, const T &b1, const T &b2, const T &b3,
                           const T &b4, const T &b5, const T &m1, const T &m2,
                           const T &m3, const T &m4, const T &m5, const T &m6,
                           const T &m7, const T &m8, T *residual) const {
                residual[0] = ((((-(2.0 * a4 * m4 * m8)) - 2.0 * a4 * m3 * m7 -
                                 2.0 * a4 * m2 * m6 - 2.0 * a4 * m1 * m5 + a1 * m4 * m4 +
                                 (2.0 * a3 * m2 - 2.0 * a2 * m3) * m4) -
                                a1 * m3 * m3 + 2.0 * a3 * m1 * m3) -
                               a1 * m2 * m2 + 2.0 * a2 * m1 * m2 + a1 * m1 * m1) -
                              b1; // e1
                residual[1] = (((2.0 * a4 * m3 * m8 - 2.0 * a4 * m4 * m7 -
                                 2.0 * a4 * m1 * m6 + 2.0 * a4 * m2 * m5) -
                                a2 * m4 * m4 + (2.0 * a3 * m1 - 2.0 * a1 * m3) * m4 +
                                a2 * m3 * m3) -
                               2.0 * a3 * m2 * m3 - a2 * m2 * m2 - 2.0 * a1 * m1 * m2 +
                               a2 * m1 * m1) -
                              b2; // e2
                residual[2] = ((((-(2.0 * a4 * m2 * m8)) - 2.0 * a4 * m1 * m7 +
                                 2.0 * a4 * m4 * m6 + 2.0 * a4 * m3 * m5) -
                                a3 * m4 * m4 + (2.0 * a1 * m2 - 2.0 * a2 * m1) * m4) -
                               a3 * m3 * m3 + ((-(2.0 * a1 * m1)) - 2.0 * a2 * m2) * m3 +
                               a3 * m2 * m2 + a3 * m1 * m1) -
                              b3; // e3
            }

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Calculate(T(a_[0]), T(a_[1]), T(a_[2]), T(a_[3]), T(a_[4]), T(b_[0]),
                          T(b_[1]), T(b_[2]), T(b_[3]), T(b_[4]), T(motor[0]),
                          T(motor[1]), T(motor[2]), T(motor[3]), T(motor[4]), T(motor[5]),
                          T(motor[6]), T(motor[7]), residual);

                return true;
            }

        private:
            const Pnt a_;
            const Pnt b_;
        };

        struct PointDifferenceCostFunctor {
            PointDifferenceCostFunctor(const Pnt &a, const Pnt &b) : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                Point<T> a(a_);
                Point<T> b(b_);
                Point<T> c = a.spin(M);

                for (int i = 0; i < 5; ++i) {
                    residual[i] = c[i] - b[i];
                }

                return true;
            }

        private:
            const Pnt a_;
            const Pnt b_;
        };

        struct PointCorrespondencesCostFunctor {
            PointCorrespondencesCostFunctor(const Pnt &a, const Pnt &b)
                    : a_(a), b_(b) {}

            template <typename T>
            auto operator()(const T *const motor, T *residual) const -> bool {
                Motor<T> M(motor);
                Point<T> a(a_);
                Point<T> b(b_);
                Point<T> c = a.spin(M);

                for (int i = 0; i < 3; ++i) {
                    residual[i] = c[i] - b[i];
                }

                return true;
            }

        private:
            const Pnt a_;
            const Pnt b_;
        };

        bool AddTangentVectorPointAngleErrorResidualBlock(const Tnv &a,
                                                          const Tnv &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TangentVectorPointAngleErrorCostFunctor,
                            3, 8>(
                            new TangentVectorPointAngleErrorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        bool AddDualPlaneAngleErrorResidualBlock(const Dlp &a, const Dlp &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<DualPlaneAngleErrorCostFunctor, 4, 8>(
                            new DualPlaneAngleErrorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        bool AddDualPlaneCommutatorResidualBlock(const Dlp &a, const Dlp &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<DualPlaneCommutatorCostFunctor, 6, 8>(
                            new DualPlaneCommutatorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        bool AddCircleCommutatorResidualBlock(const Cir &a, const Cir &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<CircleCommutatorCostFunctor, 10, 8>(
                            new CircleCommutatorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }
        bool AddCircleDifferenceResidualBlock(const Cir &a, const Cir &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<CircleDifferenceCostFunctor, 10, 8>(
                            new CircleDifferenceCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        bool AddDirectionVectorInnerProductResidualBlock(const Drv &a, const Drv &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<DirectionVectorInnerProductCostFunctor,
                            1, 8>(
                            new DirectionVectorInnerProductCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        bool AddDualPlaneDifferenceResidualBlock(const Dlp &a, const Dlp &b) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<DualPlaneDifferenceFunctor, 4, 8>(
                            new DualPlaneDifferenceFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddDualPlaneInnerProductResidualBlock(const Dlp &a, const Dlp &b)
        -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<DualPlaneInnerProductCostFunctor, 1, 8>(
                            new DualPlaneInnerProductCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }
        auto AddLineInnerProductResidualBlock(const Dll &a, const Dll &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineInnerProductCostFunctor, 1, 8>(
                            new LineInnerProductCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }
        auto AddLineDualAngleResidualBlock(const Dll &a, const Dll &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineDualAngleCostFunctor, 2, 8>(
                            new LineDualAngleCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }
        auto AddLineAntiCommutatorResidualBlock(const Dll &a, const Dll &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineAntiCommutatorCostFunctor, 1, 8>(
                            new LineAntiCommutatorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddLineProjectedCommutatorResidualBlock(const Dll &a, const Dll &b)
        -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineProjectedCommutatorCostFunctor, 6,
                            8>(
                            new LineProjectedCommutatorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddLineCommutatorResidualBlock(const Dll &a, const Dll &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineCommutatorCostFunctor, 6, 8>(
                            new LineCommutatorCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddLineProjectedCorrespondencesResidualBlock(const Dll &a, const Dll &b)
        -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineProjectedCorrespondencesCostFunctor,
                            6, 8>(
                            new LineProjectedCorrespondencesCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }
        auto AddLineCorrespondencesResidualBlock(const Dll &a, const Dll &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineCorrespondencesCostFunctor, 6, 8>(
                            new LineCorrespondencesCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddLineAngleDistanceResidualBlock(const Dll &a, const Dll &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineAngleDistanceCostFunctor, 3, 8>(
                            new LineAngleDistanceCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddLineAngleDistanceNormResidualBlock(const Dll &a, const Dll &b)
        -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<LineAngleDistanceNormCostFunctor, 2, 8>(
                            new LineAngleDistanceNormCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddPointCorrespondencesResidualBlock(const Pnt &a, const Pnt &b)
        -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<PointCorrespondencesCostFunctor, 3, 8>(
                            new PointCorrespondencesCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddGaalopPointCorrespondencesResidualBlock(const Pnt &a, const Pnt &b)
        -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<GaalopPointCorrespondencesCostFunctor,
                            3, 8>(
                            new GaalopPointCorrespondencesCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddPointDifferenceResidualBlock(const Pnt &a, const Pnt &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<PointDifferenceCostFunctor, 5, 8>(
                            new PointDifferenceCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto AddPointDistanceResidualBlock(const Pnt &a, const Pnt &b) -> bool {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<PointDistanceCostFunctor, 1, 8>(
                            new PointDistanceCostFunctor(a, b));
            problem_.AddResidualBlock(cost_function, NULL, &motor_[0]);
            return true;
        }

        auto SetMotorParameterizationTypeFromString(const std::string &type) -> void {
            if (type == "NORMALIZE") {
                problem_.SetParameterization(
                        &motor_[0],
                        new ceres::AutoDiffLocalParameterization<MotorNormalizeRotor, 8, 8>);
            } else if (type == "POLAR_DECOMPOSITION") {
                problem_.SetParameterization(
                        &motor_[0],
                        new ceres::AutoDiffLocalParameterization<MotorPolarDecomposition, 8,
                                8>);
                // } else if (type == "RETRACT_FIRST_ORDER") {
                //   problem_.SetParameterization(
                //       &motor_[0],
                //       new ceres::AutoDiffLocalParameterization<MotorRetractFirstOrder,
                //       8,
                //                                                6>);
            } else if (type == "POLAR_DECOMPOSITION_TANGENT") {
                problem_.SetParameterization(
                        &motor_[0], new ceres::AutoDiffLocalParameterization<
                                MotorTangentSpacePolarDecomposition, 8, 6>);
            } else if (type == "CAYLEY") {
                problem_.SetParameterization(
                        &motor_[0], new ceres::AutoDiffLocalParameterization<Cayley, 8, 6>);
            } else if (type == "BIVECTOR_GENERATOR") {
                problem_.SetParameterization(
                        &motor_[0],
                        new ceres::AutoDiffLocalParameterization<MotorFromBivectorGenerator,
                                8, 6>);
            } else if (type == "OUTER_EXPONENTIAL") {
                problem_.SetParameterization(
                        &motor_[0],
                        new ceres::AutoDiffLocalParameterization<OuterExponential, 8, 6>);
            } else if (type == "BIVECTOR_GENERATOR_ADEPT") {
                std::cout << "game:: ADEPT Using bivector generator (Versor)."
                          << std::endl;
                std::cout << "ERROR: NOT SUPPORTED!";
                return;
            } else {
                std::cout << "Unknown motor parameterization type" << std::endl;
            }
        }

        class UpdateMotorEachIterationCallback : public ceres::IterationCallback {
        public:
            UpdateMotorEachIterationCallback(const Mot *motor, std::vector<Mot> *list)
                    : motor_(motor), list_(list), iteration_k(1) {}
            virtual ceres::CallbackReturnType
            operator()(const ceres::IterationSummary &summary) {
                // std::cout << "Iteration " << iteration_k++ << std::endl;
                list_->push_back(Mot(*motor_));
                return ceres::SOLVER_CONTINUE;
            }

        private:
            std::vector<Mot> *list_;
            const Mot *motor_;
            int iteration_k;
        };

        auto Solve() -> Mot {
            UpdateMotorEachIterationCallback callback(&motor_, &list_);
            options_.callbacks.push_back(&callback);
            options_.update_state_every_iteration = true;
            ceres::Solve(options_, &problem_, &summary_);
            summary_.BriefReport();
            return motor_;
        }


        // private:
        Mot motor_;
        std::vector<Mot> list_;

        ceres::Problem problem_;
        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
    };

} // namespace game

