const REPO_ROOT = normpath(joinpath(@__DIR__, "..", ".."))

# Deprecated extension hooks keep working for user models until their removal,
# but the package, its templates and its examples no longer implement them. This
# gate parses every Julia file under src/, templates/ and examples/ and fails on
# any method definition of a deprecated hook, qualified or not and including the
# callable form `(f::typeof(hook))(...)`, except each hook's sanctioned
# forwarder for direct calls: one method with exactly the hook's number of
# untyped positional arguments.
const DEPRECATED_HOOKS = (:calcForceTorque, :calcControlForceTorque, :calcControlMassFlowRate, :calcReactionWheelTorque,
    :calcControlEffect!, :calcGuidanceEffect!, :calcNavigationEffect!)
const SCANNED_DIRS = ("src", "templates", "examples")
const _BRIDGES = joinpath("src", "orchestration", "simulation", "engine", "control_hook_bridges.jl")
const _TICK_BRIDGES = joinpath("src", "orchestration", "simulation", "callbacks", "tick_hook_bridges.jl")
# (file, hook) => positional arguments: the one method each deprecated hook may keep in src/.
const SANCTIONED = Dict(
    (joinpath("src", "orchestration", "simulation", "engine", "effector_sampling.jl"), :calcForceTorque) => 4,
    (_BRIDGES, :calcControlForceTorque) => 5,
    (_BRIDGES, :calcControlMassFlowRate) => 5,
    (_BRIDGES, :calcReactionWheelTorque) => 5,
    (_TICK_BRIDGES, :calcControlEffect!) => 5,
    (_TICK_BRIDGES, :calcGuidanceEffect!) => 5,
    (_TICK_BRIDGES, :calcNavigationEffect!) => 5,
)
const _COUNT_WORDS = Dict(4 => "four", 5 => "five")

_last_name(name::Symbol) = name
function _last_name(ex::Expr)
    ex.head === :. && return _last_name(ex.args[end])
    # `(f::typeof(hook))(...)` and `(::typeof(hook))(...)`
    if ex.head === :(::)
        T = ex.args[end]
        T isa Expr && T.head === :call && T.args[1] === :typeof && length(T.args) == 2 && return _last_name(T.args[2])
    end
    return nothing
end
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
                arguments = call.args[2:end]
                push!(found, (rel, name, all(_untyped, arguments) ? length(arguments) : -1))
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
        for (file_rel, name, untyped_count) in _method_definitions!(Tuple{String, Symbol, Int}[], parsed, rel)
            if get(SANCTIONED, (file_rel, name), nothing) == untyped_count && !((file_rel, name) in sanctioned_seen)
                push!(sanctioned_seen, (file_rel, name))
            else
                push!(violations, "$(file_rel): defines a method of the deprecated hook $(name); implement its replacement instead (see the hook's docstring).")
            end
        end
    end
end
for (entry, count) in sort!(collect(SANCTIONED); by=string)
    entry in sanctioned_seen || push!(violations,
        "$(entry[1]): the $(entry[2]) forwarder for direct calls ($(_COUNT_WORDS[count]) untyped positional arguments) is missing.")
end

if !isempty(violations)
    println("deprecated_hook_methods_violations:")
    foreach(v -> println("  - ", v), violations)
    error("Deprecated hook methods found in src/, templates/ or examples/.")
end
println("deprecated_hook_methods_gate_ok")
