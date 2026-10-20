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

# Whether `a` and `b` hold the same state: equal bits for bits values, equal values
# for big numbers (a `BigInt`'s limbs live behind a pointer that differs between
# copies, as in a `MersenneTwister`), equal elements for arrays, the same entries for
# dictionaries (keys paired one to one and compared like values, `_same_entries`), and
# the same field values for every other object, followed recursively. Mutable
# objects are matched one to one in both directions, so aliasing is part of the
# state: two objects of `a` cannot both correspond to one object of `b`, nor the
# reverse. Tasks, locks and IO streams are not model state and are not compared.
_same_state(a, b) = _same_state(a, b, IdDict{Any, Any}(), IdDict{Any, Any}())

function _same_state(a, b, seen::IdDict{Any, Any}, back::IdDict{Any, Any})
    typeof(a) === typeof(b) || return false
    T = typeof(a)
    isbitstype(T) && return a === b
    (a isa Module || a isa Function && !_has_captures(a) || a isa Type || a isa Symbol) && return a === b
    a isa AbstractString && return a == b
    a isa Union{BigInt, BigFloat} && return isequal(a, b)
    (a isa Task || a isa Base.AbstractLock || a isa IO || a isa Threads.Condition) && return true
    if ismutable(a)
        haskey(seen, a) && return seen[a] === b
        haskey(back, b) && return false
        seen[a] = b
        back[b] = a
    end
    a isa AbstractDict && return _same_entries(a, b, seen, back)
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

# Keys whose identity carries no state (bits values, symbols, strings): such a key is
# found by lookup and then compared with the key it finds.
_plain_key(k) = isbitstype(typeof(k)) || k isa Symbol || k isa String

# The key of `b` that `k` looks up to, or a `_NoKey()` when there is none. A `Dict`
# finds keys by `isequal`, so it can be another value (`1.0` for `1`); other
# dictionaries (an `IdDict` finds keys by identity) give `k` itself.
struct _NoKey end
_looked_up_key(b::Dict, k) = getkey(b, k, _NoKey())
_looked_up_key(b::AbstractDict, k) = haskey(b, k) ? k : _NoKey()

# Whether two dictionaries hold the same entries. Keys are paired one to one and
# compared like any other state, through the same alias maps as the values, so a key
# that is shared with another part of the state must stay shared, and a copied
# identity dictionary (an `IdDict`, whose keys are found by identity) still matches.
# Plain keys are paired by lookup. Every other key of `a` is paired with the key of `b`
# the alias maps already give it, or else with the first unpaired key of `b` (the one
# it looks up to, first) whose key and value both compare equal; each trial runs on
# copies of the alias maps, which only a match commits.
function _same_entries(a::AbstractDict, b::AbstractDict, seen::IdDict{Any, Any}, back::IdDict{Any, Any})
    length(a) == length(b) || return false
    others = Pair[]
    for (k, v) in a
        if _plain_key(k)
            _same_state(k, _looked_up_key(b, k), seen, back) || return false
            _same_state(v, b[k], seen, back) || return false
        else
            push!(others, k => v)
        end
    end
    unpaired = Pair[k => v for (k, v) in b if !_plain_key(k)]
    length(unpaired) == length(others) || return false
    for (k, v) in others
        if ismutable(k) && haskey(seen, k)
            # The key is already matched elsewhere in the state: only its partner will do.
            j = findfirst(entry -> first(entry) === seen[k], unpaired)
            j === nothing && return false
            _same_state(v, last(unpaired[j]), seen, back) || return false
        else
            j = _paired_entry(k, v, b, unpaired, seen, back)
            j === nothing && return false
        end
        deleteat!(unpaired, j)
    end
    return true
end

function _paired_entry(k, v, b::AbstractDict, unpaired::Vector{Pair}, seen::IdDict{Any, Any}, back::IdDict{Any, Any})
    order = collect(eachindex(unpaired))
    looked_up = _looked_up_key(b, k)
    i = findfirst(entry -> first(entry) === looked_up, unpaired)
    i === nothing || (deleteat!(order, i); pushfirst!(order, i))
    for j in order
        trial_seen, trial_back = copy(seen), copy(back)
        _same_state(k, first(unpaired[j]), trial_seen, trial_back) &&
            _same_state(v, last(unpaired[j]), trial_seen, trial_back) || continue
        merge!(empty!(seen), trial_seen)
        merge!(empty!(back), trial_back)
        return j
    end
    return nothing
end

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

# Mutable objects reachable from both `a` and `b`.
function _shared_mutables(a, b)
    from_a = _reachable_mutables(a)
    return [object for object in keys(_reachable_mutables(b)) if haskey(from_a, object)]
end
