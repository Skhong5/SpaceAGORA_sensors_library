# The project's working example, run by the contributor workflow after the tests.
using ContributorProject

solution = ContributorProject.run_case()
println("run_case: t = ", last(solution.t), " s, final state finite: ", all(isfinite, solution.u[end]))
