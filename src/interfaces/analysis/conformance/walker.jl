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
# object, followed recursively. Mutable objects are matched one to one in both
# directions, so aliasing is part of the state: two objects of `a` cannot both
# correspond to one object of `b`, nor the reverse. Tasks, locks and IO streams are
# not model state and are not compared.
_same_state(a, b) = _same_state(a, b, IdDict{Any, Any}(), IdDict{Any, Any}())

function _same_state(a, b, seen::IdDict{Any, Any}, back::IdDict{Any, Any})
    typeof(a) === typeof(b) || return false
    T = typeof(a)
    isbitstype(T) && return a === b
    (a isa Module || a isa Function && !_has_captures(a) || a isa Type || a isa Symbol) && return a === b
    a isa AbstractString && return a == b
    (a isa Task || a isa Base.AbstractLock || a isa IO || a isa Threads.Condition) && return true
    if ismutable(a)
        haskey(seen, a) && return seen[a] === b
        haskey(back, b) && return false
        seen[a] = b
        back[b] = a
    end
    if a isa AbstractDict
        length(a) == length(b) || return false
        for (k, v) in a
            haskey(b, k) || return false
            _same_state(v, b[k], seen, back) || return false
        end
        return true
    end
    if a isa Array
        size(a) == size(b) || return false
        for i in eachindex(a)
            isassigned(a, i) == isassigned(b, i) || return false
            isassigned(a, i) && !_same_state(a[i], b[i], seen, back) && return false
        end
        return true
    end
    for i in 1:fieldcount(T)
        isdefined(a, i) == isdefined(b, i) || return false
        isdefined(a, i) && !_same_state(getfield(a, i), getfield(b, i), seen, back) && return false
    end
    return true
end

_has_captures(f::Function) = fieldcount(typeof(f)) > 0

# Every mutable object reachable from `x` (arrays, dictionaries, mutable structs),
# by identity, skipping what `_same_state` does not compare.
function _reachable_mutables(x, found::IdDict{Any, Nothing}=IdDict{Any, Nothing}())
    T = typeof(x)
    isbitstype(T) && return found
    (x isa Module || x isa Type || x isa Symbol || x isa AbstractString || x isa Task ||
        x isa Base.AbstractLock || x isa IO || x isa Threads.Condition) && return found
    if ismutable(x)
        haskey(found, x) && return found
        found[x] = nothing
    end
    if x isa AbstractDict
        for (k, v) in x
            _reachable_mutables(k, found)
            _reachable_mutables(v, found)
        end
    elseif x isa Array
        for i in eachindex(x)
            isassigned(x, i) && _reachable_mutables(x[i], found)
        end
    else
        for i in 1:fieldcount(T)
            isdefined(x, i) && _reachable_mutables(getfield(x, i), found)
        end
    end
    return found
end
