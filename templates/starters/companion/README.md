# StarterName

A companion package for [SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl)
that serves its external-propagation boundary: spacecraft that an outside integrator
owns for a whole run. Replace this paragraph with the engine it brings.

## The boundary

The companion reaches SpaceAGORA only through the versioned facade
`SpaceAGORA.ExternalPropagationServices`. It requires the facade's exact contract
version (`CORE_CONTRACT_VERSION`) before it defines any method, and again at load,
so an incompatible core is refused before any of its code runs.
`CircularOrbitPropagator` is an example owner to replace with the companion's own
engine; it implements the facade's `IMPLEMENTED_FUNCTIONS`.

A companion for a new boundary needs that boundary in SpaceAGORA first: its facade,
reviewed there, then this starter pointed at it.

## Set up and test

Once:

```bash
git clone -c lfs.fetchexclude='*' https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git ../SpaceAGORA.jl
git -C ../SpaceAGORA.jl checkout <the rev in SpaceAGORASource.toml>
julia --startup-file=no ../SpaceAGORA.jl/scripts/contributor_environment.jl companion . ../StarterName-env --manifest core
```

`-c lfs.fetchexclude='*'` keeps this clone, its checkouts and later pulls from
downloading SpaceAGORA's large data files (Git LFS), which a contributor
repository does not use. Without Git LFS installed the setting does nothing,
and nothing is downloaded either.

Then `julia --project=../StarterName-env -e 'using Pkg; Pkg.test("StarterName")'`.
The tests check the facade's shape at the companion's contract version, a short run
with the example owner, and that a SpaceAGORA-only session loads none of the
companion. `test/incompatible_core.jl` (an incompatible contract is refused) and
`test/load_order.jl core-first` / `companion-first` run in fresh processes; CI runs
all of them.

## License

`LICENSE` is the MIT license, as for SpaceAGORA, with the year and the copyright
holder the generator was given (`--holder`). Replace `Your name or organization`
if it is still there, or replace the file to choose another license.

## Claims

A conformance pass certifies the boundary; the engine's correctness and every
claim made from it are the author's work.
