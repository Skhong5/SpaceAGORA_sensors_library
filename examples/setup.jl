using Pkg, TOML

# Prepare the examples environment (examples/Project.toml): this checkout's
# SpaceAGORA, the packages the examples, plotting scripts and reporting studies
# use (Plots, Roots and the others), and every dependency of SpaceAGORA, which
# the examples import directly. Its manifest is generated, not committed, and is
# seeded with the root manifest on every run, so the examples load the package
# versions the root project pins and reuse their package images. Resolving keeps
# every seeded version and adds only the example tools.
examples = @__DIR__
root = dirname(examples)
undeclared = setdiff(keys(TOML.parsefile(joinpath(root, "Project.toml"))["deps"]),
                     keys(TOML.parsefile(joinpath(examples, "Project.toml"))["deps"]))
isempty(undeclared) ||
    error("examples/Project.toml must also declare these dependencies of the root project: $(join(sort(collect(undeclared)), ", ")).")
cp(joinpath(root, "Manifest.toml"), joinpath(examples, "Manifest.toml"); force=true)
Pkg.activate(examples)
Pkg.resolve()
Pkg.instantiate()
