const REPO_ROOT = normpath(joinpath(@__DIR__, "..", ".."))

# Deprecated extension hooks keep working for user models until their removal,
# but the package, its templates and its examples no longer implement them. This
# gate parses every Julia file under src/, templates/ and examples/ and fails on
# any method definition of a deprecated hook, qualified or not, except the
# sanctioned untyped forwarder for direct calls.
const DEPRECATED_HOOKS = (:calcForceTorque,)
const SCANNED_DIRS = ("src", "templates", "examples")
# (file, hook): the one method each deprecated hook may keep in src/.
const SANCTIONED = Set([
    (joinpath("src", "orchestration", "simulation", "engine", "effector_sampling.jl"), :calcForceTorque),
])

_last_name(name::Symbol) = name
_last_name(ex::Expr) = ex.head === :. ? _last_name(ex.args[end]) : nothing
_last_name(q::QuoteNode) = q.value isa Symbol ? q.value : nothing
_last_name(::Any) = nothing

# The call expression of a method signature, through `where` and return types.
function _signature_call(sig)
    while sig isa Expr && sig.head in (:where, :(::))
        sig = sig.args[1]
    end
    return sig isa Expr && sig.head === :call ? sig : nothing
end

_untyped(arg) = arg isa Symbol

function _method_definitions!(found, ex, rel::String)
    ex isa Expr || return found
    if ex.head === :function || (ex.head === :(=) && _signature_call(ex.args[1]) !== nothing)
        call = _signature_call(ex.args[1])
        if call !== nothing
            name = _last_name(call.args[1])
            if name in DEPRECATED_HOOKS
                push!(found, (rel, name, all(_untyped, call.args[2:end])))
            end
        end
    end
    for arg in ex.args
        _method_definitions!(found, arg, rel)
    end
    return found
end

violations = String[]
sanctioned_seen = Set{Tuple{String, Symbol}}()
for dir in SCANNED_DIRS
    base = joinpath(REPO_ROOT, dir)
    isdir(base) || continue
    for (root, _, files) in walkdir(base), file in sort(files)
        endswith(file, ".jl") || continue
        path = joinpath(root, file)
        rel = relpath(path, REPO_ROOT)
        parsed = try
            Meta.parseall(read(path, String); filename=rel)
        catch err
            push!(violations, "$(rel): does not parse ($(sprint(showerror, err)))")
            continue
        end
        for (file_rel, name, untyped) in _method_definitions!(Tuple{String, Symbol, Bool}[], parsed, rel)
            if (file_rel, name) in SANCTIONED && untyped && !((file_rel, name) in sanctioned_seen)
                push!(sanctioned_seen, (file_rel, name))
            else
                push!(violations, "$(file_rel): defines a method of the deprecated hook $(name); implement its replacement instead (see the hook's docstring).")
            end
        end
    end
end
for entry in SANCTIONED
    entry in sanctioned_seen || push!(violations,
        "$(entry[1]): the untyped $(entry[2]) forwarder for direct calls is missing.")
end

if !isempty(violations)
    println("deprecated_hook_methods_violations:")
    foreach(v -> println("  - ", v), violations)
    error("Deprecated hook methods found in src/, templates/ or examples/.")
end
println("deprecated_hook_methods_gate_ok")
