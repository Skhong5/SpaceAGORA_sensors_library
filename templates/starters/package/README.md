# StarterName

An RPO reference planner for [SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl).
Replace this paragraph with what the planner does and when to use it.

## The planner

`StraightLinePlanner` is an example to replace with the package's own method. It
subtypes `AbstractRPOPlanner`, so it belongs to SpaceAGORA's RPO reference family:
the family builds each request, validates every proposed reference with trusted
code and installs only a validated copy. The planner declares what it supports
(`planner_capabilities`), whether it draws from its random stream (`consumes_rng`)
and proposes references (`plan_rpo!`). A package adds no family of its own; a new
family is a SpaceAGORA pull request.

## Set up and test

The package is developed against the SpaceAGORA commit in `SpaceAGORASource.toml`,
in a separate environment that develops that commit by path. Once:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git ../SpaceAGORA.jl
git -C ../SpaceAGORA.jl checkout <the rev in SpaceAGORASource.toml>
julia --startup-file=no ../SpaceAGORA.jl/scripts/contributor_environment.jl package . ../StarterName-env --manifest core
```

`GIT_LFS_SKIP_SMUDGE=1` skips SpaceAGORA's large data files (Git LFS), which a
contributor repository does not use.

Then `julia --project=../StarterName-env -e 'using Pkg; Pkg.test("StarterName")'`.
`test/runtests.jl` runs SpaceAGORA's conformance checks for planners on the family's
representative requests and a short simulation; `test/method_tests.jl` holds the
package's own tests of its method. `examples/run_in_simulation.jl` runs the planner
in the RPO pilot; CI runs it after the tests.

Users of the package install SpaceAGORA first: the pin file is for this package's
own development and CI, and it does not reach them. `[compat]` admits one
SpaceAGORA release series.

## License

`LICENSE` is the MIT license, as for SpaceAGORA, with the year and the copyright
holder the generator was given (`--holder`). Replace `Your name or organization`
if it is still there, or replace the file to choose another license.

## Claims

A conformance pass certifies that the planner meets the family's interface; the
method's correctness and every claim made from it are the author's work.
