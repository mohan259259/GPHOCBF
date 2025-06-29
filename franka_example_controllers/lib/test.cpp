#include <casadi/casadi.hpp>
using namespace casadi;

int main() {
  casadi::DM H, f, A, b;

  // {
  //   std::lock_guard<std::mutex> lock(opt_mutex_);
  //   H = casadi::MX::evalf(cbf_params.H);
  //   f = casadi::MX::evalf(cbf_params.f);
  //   A = casadi::MX::evalf(cbf_params.A);
  //   b = casadi::MX::evalf(cbf_params.b);
  // }
  H = 2 * casadi::DM::eye(7);
  f = casadi::DM::ones(7, 1);
  A = casadi::DM::ones(1, 7);
  b = casadi::DM::ones(1, 1);

  casadi::Function cbf_solver = casadi::external("cbf_solver", "cbf_solver.so");

  std::vector<casadi::DM> arg = {H, f, A, b};
  auto res = cbf_solver(arg);


  std::cout << "H:" << H << std::endl;
  std::cout << "f:" << f << std::endl;
  std::cout << "A:" << A << std::endl;
  std::cout << "b:" << b << std::endl;
  std::cout << "res:" << res << std::endl;
  std::cout << "res.at(0):" << res.at(0) << std::endl;

  return 0;
}