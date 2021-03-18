# Optimization on manifold

[toc]

In this document we demonstrate internal implementation in Ceres, then implement SE3 by Ceres.

## Normal Non-linear Least Square Problem

Let's take a brief look at how some of the core optimization algorithms in Ceres work, including **CostFunction** and **LocalParameterization**.

Ceres solves unconstrained non-linear least squares problems of the form:
$$
\frac12 \mathop{min}\limits_x \sum||r(x)||^2
$$
In Ceres parlance, the expression $||r(x)||^2$ is known as a **residual block**, where $r(x)$ is a **CostFunction** that depends on the **parameter blocks** $x$.

### Solve the problem in Ceres

After some derivations, we get the Gauss-Newton equation as following (where $\Delta x$ denotes step increasement and $J$  is the jacobian of $r(x)$ wrt. $x$ ):
$$
J^TJ\Delta x=-J^Tr
$$
So all we need to do is providing $r$ and $J$ to Ceres. Take a look at [ceres-autodiff-example](http://www.ceres-solver.org/nnls_tutorial.html#hello-world), where AutoDiffCostFunction computes Jacobian automaticly.

### Local parameterization

In many optimization problems, especially sensor fusion problems, one has to model quantities that live in spaces known as Manifolds , for example the rotation/orientation of a sensor that is represented by a Quaternion.

Manifolds are spaces, which locally look like Euclidean spaces. More precisely, at each point on the manifold there is a linear space that is tangent to the manifold, called tangent manifold. It has dimension equal to the intrinsic dimension of the manifold itself, which is less than or equal to the ambient space in which the manifold is embedded.

Ceres provides the LocalParameterization interface which allow the user to define and associate with parameter blocks the manifold that they belong to. Methods Plus(x,delta,x_plus_delta) and ComputeJacobian(x,jacobian) have to be defined within LocalParameterization, see [LocalParameterization](http://www.ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres21LocalParameterizationE) for reference.

Let's take an ICP (Iterative Closest Point) example:

```cpp
class CostFunctor {
public:
  CostFunctor(Vec x, Vec y) : x_(x), y_(y) {}

  template <typename T> bool operator()(const T *const q_raw, T *r_raw) const {
    Eigen::Map<const Eigen::Quaternion<T>> q(q_raw);
    Eigen::Map<Eigen::Matrix<T, 3, 1>>{r_raw} =
        (q * x_.cast<T>() - y_.cast<T>());
    return true;
  }

  static ceres::CostFunction *Create(const Vec &x, const Vec &y) {
    return new ceres::AutoDiffCostFunction<CostFunctor, 3, 4>(
        new CostFunctor(x, y));
  }

private:
  Vec x_;
  Vec y_;
};
// Use ceres::EigenQuaternionParameterization to optimize quaternion on manifolds
```

#### Internal mechanism*

The cost function jacobian $J_g$ denotes jacobian of residuals wrt. ambient space (see [above](#solve-the-problem-in-ceres)). The local parameterization jacobian $J_l$ is jacobian of ambient space wrt. tangent space. Then Ceres multiply $J_g$ and $J_l$ to get jacobian of redisuals wrt. tanget space, denoted as $J$ , use formula in [section above](#solve-the-problem-in-ceres) to get increasement in tanget space. Finally use `Plus` to apply tanget increasement to original paramter $x$.

## SE(3) perturbation model

See example below:

```cpp
class CostFunctorSO3 : ceres::SizedCostFunction<3, 4> {
public:
  CostFunctorSO3(Vec x, Vec y) : x_(x), y_(y) {}

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> q(parameters[0]);
    Eigen::Map<Eigen::Vector3d>{residuals} = q * x_ - y_;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J(
            jacobians[0]);
        // keypoint1:
        // jacobian of q * SO3::exp(x)  wrt. x at x=0
        // that is the jacobian of redisual wrt tanget space
        J.leftCols(3) = -(q.toRotationMatrix() * Skew(x_));
        J.rightCols(1).setZero();
      }
    }
    return true;
  }

  static ceres::CostFunction *Create(const Vec &x, const Vec &y) {
    return new CostFunctorSO3(x, y);
  }

private:
  Vec x_;
  Vec y_;
};

class ParameterizationSO3 : public ceres::LocalParameterization {
public:
  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override {
    auto dq = Sophus::SO3d::exp(Vec(delta));
    Eigen::Map<Eigen::Quaterniond>{x_plus_delta} =
        Eigen::Map<const Eigen::Quaterniond>{x} * dq.unit_quaternion();
    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> J(jacobian);
    // keypoint2:
    // we've already got jacobian of rediduals to tanget space,
    // so local parameterization jacobian should be set to identity
    J.setIdentity();
    return true;
  }

  int GlobalSize() const override { return 4; }

  int LocalSize() const override { return 3; }
};
```

