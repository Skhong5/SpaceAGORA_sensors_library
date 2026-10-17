"""Shared pre-run warnings, including renamed or retired extension hooks."""
module HookDeprecation

"""
    _hook_deprecation(key::Tuple, msg::String)

Warn once per process about a deprecated extension hook, with `key` naming the
model type and the hook, for example `(MyEffector, :calcForceTorque)`. Under
`julia --depwarn=error` it throws an `ErrorException` with `msg` instead, so a
test run can refuse deprecated hooks.

The run calls this before integration, for each configured model type that
implements a deprecated hook. `Base.depwarn` is not used: its once-only filter
is keyed on the call site, so one check over several model types would warn
about the first type only, and it is silent unless Julia runs with
`--depwarn=yes`.
"""
function _hook_deprecation(key::Tuple, msg::String)
    return _preflight_warning(key, msg; group=:depwarn)
end

"""
    _preflight_warning(key::Tuple, msg::String; group::Symbol=:preflight)

Emit a pre-run warning once per key through the active logger. Refuse the
configuration under `--depwarn=error`, even if the logger has already seen the
key. Non-deprecation configuration checks use the default `:preflight` group.
"""
function _preflight_warning(key::Tuple, msg::String; group::Symbol=:preflight)
    Base.JLOptions().depwarn == 2 && throw(ErrorException(msg))
    @warn msg _group=group _id=key maxlog=1
    return nothing
end

# Renamed index fields (spacecraft_index, chaser_index, target_index): a model
# type's old keyword and property both warn through `_hook_deprecation`, keyed by
# `(type, old name)`, so the warning shows once per type and name.
@noinline function _renamed_field_warning(::Type{T}, old::Symbol, new::Symbol) where {T}
    _hook_deprecation((T, old),
        "$(nameof(T)).$(old) is deprecated and will be removed in SpaceAGORA 0.5.0; use $(new) instead: " *
        "only the name changes.")
    return new
end

# The default of a renamed keyword and of its old name: "not passed". It is not
# `nothing`, so an explicit `nothing` reaches the field and is refused there.
struct _NotPassed end
const _NOT_PASSED = _NotPassed()

"""
    _renamed_keyword(T, old, old_value, new, new_value, default)

The value of a constructor keyword that was renamed from `old` to `new`, where
`_NOT_PASSED` means "not passed" (any other value, `nothing` included, counts as
passed). Passing `old` warns once per type and name (see
`_renamed_field_warning`); passing both names throws an `ArgumentError`; passing
neither gives `default`.
"""
function _renamed_keyword(::Type{T}, old::Symbol, old_value, new::Symbol, new_value, default) where {T}
    old_value === _NOT_PASSED && return new_value === _NOT_PASSED ? default : new_value
    new_value === _NOT_PASSED || throw(ArgumentError(
        "$(nameof(T)): pass $(new) only; $(old) is its deprecated name."))
    _renamed_field_warning(T, old, new)
    return old_value
end

end # module HookDeprecation
