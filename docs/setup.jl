using Pkg

# Prepare the documentation environment: Documenter and this checkout's
# SpaceAGORA (docs/Project.toml). Its manifest is generated, not committed, and
# is seeded with the root manifest on every run, so the docs build loads the
# package versions the tests load and reuses their package images. Resolving
# keeps every seeded version and adds only the documentation tools.
docs = @__DIR__
cp(joinpath(dirname(docs), "Manifest.toml"), joinpath(docs, "Manifest.toml"); force=true)
Pkg.activate(docs)
Pkg.resolve()
Pkg.instantiate()
