# Walkers: the concrete types a package defines, and the state reachable from an
# object.

"""
    package_types(T::Type, root::Module=SpaceAGORA) -> Vector{Any}

Every concrete type that subtypes `T` and is defined in `root` or one of its
submodules (types are found by name, so each appears once). Core's conformance
tests use it to run a check on every built-in model; a package can use it the same
way on its own module.
"""
function package_types(T::Type, root::Module=parentmodule(@__MODULE__))
    found = Any[]
    seen = Set{Module}()
    function walk(m::Module)
        m in seen && return
        push!(seen, m)
        for n in names(m; all=true)
            isdefined(m, n) || continue
            v = getfield(m, n)
            if v isa Module
                parentmodule(v) === m && v !== m && walk(v)
            elseif v isa Union{DataType, UnionAll} && parentmodule(v) === m
                body = Base.unwrap_unionall(v)
                body isa DataType && !isabstracttype(body) && body <: T && push!(found, v)
            end
        end
    end
    walk(root)
    return unique(found)
end

# Whether `a` and `b` hold the same state: equal bits for bits values, equal
# elements for arrays and dictionaries, and the same field values for every other
# object, followed recursively. A mutable object reached twice in `a` must
# correspond to one object in `b` (aliasing is part of the state). Tasks, locks and
# IO streams are not model state and are not compared.
_same_state(a, b) = _same_state(a, b, IdDict{Any, Any}())

function _same_state(a, b, seen::IdDict{Any, Any})
    typeof(a) === typeof(b) || return false
    T = typeof(a)
    isbitstype(T) && return a === b
    (a isa Module || a isa Function && !_has_captures(a) || a isa Type || a isa Symbol) && return a === b
    a isa AbstractString && return a == b
    (a isa Task || a isa Base.AbstractLock || a isa IO || a isa Threads.Condition) && return true
    if ismutable(a)
        haskey(seen, a) && return seen[a] === b
        seen[a] = b
    end
    if a isa AbstractDict
        length(a) == length(b) || return false
        for (k, v) in a
            haskey(b, k) || return false
            _same_state(v, b[k], seen) || return false
        end
        return true
    end
    if a isa Array
        size(a) == size(b) || return false
        for i in eachindex(a)
            isassigned(a, i) == isassigned(b, i) || return false
            isassigned(a, i) && !_same_state(a[i], b[i], seen) && return false
        end
        return true
    end
    for i in 1:fieldcount(T)
        isdefined(a, i) == isdefined(b, i) || return false
        isdefined(a, i) && !_same_state(getfield(a, i), getfield(b, i), seen) && return false
    end
    return true
end

_has_captures(f::Function) = fieldcount(typeof(f)) > 0
