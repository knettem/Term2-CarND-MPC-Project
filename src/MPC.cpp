#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
//Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
double ref_v = 75;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        fg[0] = 0;
        // The part of the cost based on the reference state.
        for (int i = 0; i < N; i++) {
            fg[0] += 2000 * CppAD::pow(vars[cte_start + i], 2);
            fg[0] += 2000 * CppAD::pow(vars[epsi_start + i], 2);
            fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
        }

        // Minimize the use of actuators..
        for (int i = 0; i < N - 1; i++) {
            fg[0] += 5 * CppAD::pow(vars[delta_start + i], 2);
            fg[0] += 5 * CppAD::pow(vars[a_start + i], 2);
            fg[0] += 500*CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
        }

        // Minimizing the value gap between sequential actuations.
        for (int i = 0; i < N - 2; i++) {
            fg[0] += 200 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
        }
        //Initialization & constraints

        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        for (int j = 1; j < N; j++) {

            AD<double> x0 = vars[x_start + j - 1];
            AD<double> y0 = vars[y_start + j - 1];
            AD<double> psi0 = vars[psi_start + j - 1];
            AD<double> v0 = vars[v_start + j - 1];
            AD<double> cte0 = vars[cte_start + j - 1];
            AD<double> epsi0 = vars[epsi_start + j - 1];

            AD<double> x1 = vars[x_start + j];
            AD<double> y1 = vars[y_start + j];
            AD<double> psi1 = vars[psi_start + j];
            AD<double> cte1 = vars[cte_start + j];
            AD<double> v1 = vars[v_start + j];
            AD<double> epsi1 = vars[epsi_start + j];
            AD<double> a = vars[a_start + j - 1];
            AD<double> delta = vars[delta_start + j - 1];

            if (j > 1) {
                a = vars[a_start + j - 2];
                delta = vars[delta_start + j - 2];
            }
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            //Setup the rest of the model constraints
            fg[1 + x_start + j] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + j] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + j] = psi1 - (psi0 - v0/Lf * delta * dt);
            fg[1 + v_start + j] = v1 - (v0 + a * dt);
            fg[1 + cte_start + j] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + j] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);
        }

    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9

    size_t n_vars = N * 6 + (N - 1) * 2;
    size_t n_constraints = N * 6;
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    //Set lower and upper limits for variables.

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.

    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    for (int i = delta_start; i < a_start; i++) {

        vars_lowerbound[i] = -0.436332*Lf;
        vars_upperbound[i] = 0.436332*Lf;
    }

    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    vector<double> result;

    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for (int i = 0; i < N-1; i++) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    return result;
}