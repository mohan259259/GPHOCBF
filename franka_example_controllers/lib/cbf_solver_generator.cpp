#include <dlfcn.h>
#include <stdio.h>
#include <casadi/casadi.hpp>

int main() {
  casadi::MX ub = 0.8 * casadi::MX::vertcat({
                            87.0, 87.0, 87.0, 87.0,  // 前4个关节扭矩上限
                            12.0, 12.0, 12.0         // 后3个关节扭矩上限
                        });
  casadi::MX lb = (-1) * ub;

  casadi::Dict casadiOptions;
  casadi::Dict solverOptions;

  //   casadiOptions["expand"] = true;
  //   casadiOptions["error_on_fail"] = true;
  casadiOptions["print_time"] = true;

  solverOptions["print_level"] = 0;
  solverOptions["max_iter"] = 3000;
  solverOptions["tol"] = 1e-8;
  solverOptions["linear_solver"] = "ma57";

  auto cbf_opti = casadi::Opti();
  casadi::MX tau = cbf_opti.variable(7, 1);

  casadi::MX H = cbf_opti.parameter(7, 7);  // 参数占位符
  casadi::MX f = cbf_opti.parameter(7, 1);
  casadi::MX A = cbf_opti.parameter(1, 7);  // 参数占位符
  casadi::MX b = cbf_opti.parameter(1, 1);

  //设置初始值
  // cbf_opti.set_value(H, 2 * casadi::DM::eye(7));
  // cbf_opti.set_value(f, casadi::DM::ones(7, 1));
  // cbf_opti.set_value(A, casadi::DM::ones(1, 7));
  // cbf_opti.set_value(b, casadi::DM::ones(1, 1));

  //   cbf_opti.set_value(H, NAN);
  //   cbf_opti.set_value(f, NAN);
  //   cbf_opti.set_value(A, NAN);
  //   cbf_opti.set_value(b, NAN);
  casadi::MX cost = 0.5 * casadi::MX::mtimes(casadi::MX::mtimes(tau.T(), H), tau) +
                    casadi::MX::mtimes(f.T(), tau);

  cbf_opti.minimize(cost);  // 添加目标函数

  // 约束1：线性约束
  cbf_opti.subject_to(casadi::MX::mtimes(A, tau) <= b);

  // 约束2：上下界
  cbf_opti.subject_to(lb <= tau <= ub);

  cbf_opti.solver("ipopt", casadiOptions, solverOptions);

  //   auto solution = cbf_opti.solve();
  //   casadi::DM output = solution.value(tau);

  casadi::Function cbf_solver = cbf_opti.to_function("cbf_solver", {H, f, A, b},  // 输入参数
                                                     {tau}                        // 输出变量

  );

  //   std::cout << "output:" << output << std::endl;
  // 调用casadi求解器生成代码
  cbf_solver.generate("cbf_solver");

  std::string compile_command = "gcc -fPIC -shared -O3 cbf_solver.c -o cbf_solver.so";
  int flag = system(compile_command.c_str());
  casadi_assert(flag == 0, "Compilation failed");

  return 0;
}