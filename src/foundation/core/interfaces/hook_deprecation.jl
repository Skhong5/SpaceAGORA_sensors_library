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

end # module HookDeprecation
