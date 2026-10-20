using Pkg, TOML

# Prepare the test environment (test/Project.toml): this checkout's SpaceAGORA,
# the test tools (Aqua, the reference and plotting packages) and every package
# the harness imports, which includes all of SpaceAGORA's own dependencies,
# because suites 01 and 03 load the package source into sandbox modules. Its
# manifest is generated, not committed, and is seeded with the root manifest on
# every run, so the tests load the package versions the root project pins and
# reuse their package images. Resolving keeps every seeded version and adds
# only the test tools.
test = @__DIR__
root = dirname(test)
undeclared = setdiff(keys(TOML.parsefile(joinpath(root, "Project.toml"))["deps"]),
                     keys(TOML.parsefile(joinpath(test, "Project.toml"))["deps"]))
isempty(undeclared) ||
    error("test/Project.toml must also declare these dependencies of the root project: $(join(sort(collect(undeclared)), ", ")).")
cp(joinpath(root, "Manifest.toml"), joinpath(test, "Manifest.toml"); force=true)
Pkg.activate(test)
Pkg.resolve()
Pkg.instantiate()
