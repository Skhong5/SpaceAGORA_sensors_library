# EDL Aerocapture GP Uncertainty Study

This study evaluates whether a preceding aerocapture-like sensing pass can
improve real-time atmospheric uncertainty modeling for a later Earth EDL pass
relative to using Earth GRAM alone. The setup uses:

- a position-indexed truth field (see [Truth Sources](#truth-sources))
- a seeded `GRAMSuite.jl` nominal and ensemble as the onboard prior
- a Gaussian-process residual model over `lat, lon, alt` with an optional
  parametric mean function
- synthetic aerocapture and EDL corridors in nearby latitude/longitude bands

Historical result tables and investigations follow the
[research-record policy](../../../docs/src/user/studies_benchmarks.md#research-records-and-generated-results).
The definitions and procedures below do not assert a new validation of this study.

## Truth Sources

The estimator assumes a position-indexed residual. Native GRAM's perturbed
sampling carries instance history, so `gram_walk` is retained as a legacy
comparison rather than an independent position-indexed truth. The other
sources build on nominal GRAM or the specified reanalysis construction.

| `--truth-source` | Truth | Use |
|---|---|---|
| `merra2_native` | Day-specific MERRA-2 **model-level** reanalysis, `72` levels to `0.01 hPa` (~`80 km`), `3-hourly` | Day-specific weather and time evolution; requires downloaded granules |
| `merra2` (default) | The MERRA-2 grids vendored with Earth-GRAM, applied as a multiplicative anomaly on GRAM nominal | Climatology, ceiling ~`64 km`. Works with no download |
| `synthetic_field` | GRAM nominal times `exp(bias + amplitude * f)`, `f` a smooth field constructed from random Fourier features | Estimator checks against a known planted signal |
| `gram_epoch_shift` | GRAM nominal at an epoch shifted from the one the prior assumes | Realistic same-model case: genuine day-to-day variability the prior cannot know |
| `gram_walk` | GRAM `perturbedDensity` walked along the trajectory | Legacy only. Not position-indexed; the driver warns and `RESULTS.md` is annotated. Retained to reproduce pre-fix numbers |

### MERRA-2

[`merra2.jl`](merra2.jl)
reads the binary grids vendored with Earth-GRAM directly, following the layout
in `Earth/source/MERRA2Data.cpp`: nineteen 3-D `(pressure, lat, lon)` Float64
blocks with density first and level height second, then twenty surface blocks.
The binary layout provides structural checks on block count, dimensions and
variable interpretation. Run the supplied reader tests and comparisons for the
actual data used; archived profile agreement does not qualify a new dataset.

Truth is applied as a multiplicative anomaly so it stays defined at every
altitude:

```text
rho_truth = rho_gram * exp(taper * (log(rho_m2 / rho_gram) + dispersion * sigma_m2 * g))
```

Two properties of the vendored data drive the design:

- The grids are **monthly climatologies by time-of-day slot**, not specific-day
  reanalysis. `--merra2-dispersion` scales a smooth random field by the
  climatology's relative interannual density standard deviation. This is a
  constructed specific-day anomaly, not an observed day. Set it to `0` for the
  raw climatology field.
- MERRA-2 spans `1000 mb` to `0.1 mb`, about `0.1 km` to **`64 km`**, not
  `80 km`. The implementation evaluates the anomaly no higher than the local
  data ceiling minus `1 km`, then holds that boundary value while tapering it
  to zero over `--merra2-blend-width-km` (default `10 km`). Truth reverts to
  GRAM nominal at the end of that taper; the return altitude depends on the
  local data ceiling and configured blend width.

That ceiling interacts directly with the corridor. See
[Known Remaining Issues](#known-remaining-issues).

## Mean Function

A zero-mean stationary GP tends toward no residual correction away from its
training data. Correlation length, sensing depth and the scored altitude band
therefore need to be considered together.

`--mean-basis` adds a parametric mean fitted by generalized least squares
(universal kriging), which extrapolates where the zero-mean GP term cannot, and
inflates the predictive variance where the basis itself is extrapolated:

- `none` — the original zero-mean residual GP
- `constant` — a bulk log-density offset relative to the prior
- `linear_alt` — offset plus an altitude trend

The fitted coefficients are written to `mean_function_fits.csv` and summarized
in `RESULTS.md`.

Compare mean bases on the same truth field and sensing band. Extrapolating a
fitted altitude trend is an assumption to evaluate, not an automatic improvement.

## Correlation Structure

Earth-GRAM carries an explicit spatial correlation model. `EarthAtmosphere::getCorrelationCoefficients`
forms

```text
rho = exp(-( dx / Lh + dz / Lz + dt / Lt ))
```

a separable exponential on an L1 metric, with `Lh` and `Lz` tabulated against
altitude in `EarthAtmosphereData.cpp::rsData` and transcribed into
[`gram_correlation.jl`](gram_correlation.jl):

| altitude | `Lh` | `Lz` |
|---|---|---|
| `20 km` | `2040 km` | `16.0 km` |
| `35 km` | `2279 km` | `18.3 km` |
| `50 km` | `2300 km` | `19.3 km` |
| `120 km` | `3900 km` | `32.0 km` |

### Checking correlation and extrapolation

[`measure_merra2_correlation.jl`](measure_merra2_correlation.jl) forms anomaly
ensembles from the vendored grids and fits horizontal correlation against
chordal separation. Time-of-day departures from a monthly mean measure the
diurnal component; monthly departures from the annual mean describe a different
ensemble with only twelve members. Keep those interpretations separate.

The CLI defaults are `--lat-scale-deg 20.5 --lon-scale-deg 21.8
--alt-scale-km 18.33`; `gram_exponential` uses the tabulated separable-exponential
form. These are model choices. Assess predictive error and uncertainty on the
chosen truth source rather than treating correlation similarity as validation.

### Time

GRAM's correlation includes a temporal term, `dt / Lt` with
`Lt = max(3 hr, 0.735 day * h_km^0.116)` — about `26.6 hr` at `35 km`, so a
`6 hr` gap retains `rho_t = 0.80`. `--kernel-time-scale-hours` adds the matching
factor to the kernel.

GRAM's `Lt` describes its small-scale perturbation model; it is not a measured
persistence time for every reanalysis anomaly.

The `--merra2-time-decorrelation` AR(1) surrogate exists only for the
climatology truth source, which contains no weather and therefore no real
evolution. `merra2_native` supplies genuine `3-hourly` evolution and does not
use it.

## Uncertainty: `--amplitude-mode`

The original estimator gave the GP one global kernel amplitude,
`var(detrended residuals)`, and then re-introduced the GRAM prior variance
afterwards with

```text
fused_mu  = prior_mu + delta_mu                        # additive
fused_var = 1/(1/prior_var + 1/delta_var)              # inverse-variance
```

Those come from two different models. The additive form pairs with
`var = prior_var + delta_var`; the inverse-variance form pairs with a
precision-weighted mean. Taking one from each means the mean moves by the full
GP correction while the variance is computed as if the GP were a redundant
second measurement of the same quantity. Since `1/prior_var > 0`, the result is
always smaller than `delta_var`, and for a zero-mean GP `delta_var <= amplitude^2`
— so the reported variance could never exceed the kernel amplitude anywhere in
the corridor, however uncertain the prior or however far from data the point.

`--amplitude-mode prior_scaled`, the default, folds the prior variance into the
kernel instead:

```text
k(x, x') = lambda^2 * s(x) * s(x') * rho(x, x')
```

`s(x)` is the GRAM ensemble relative sigma and `rho` the correlation function;
`D K D` with `D = diag(s)` is positive definite whenever `K` is. The GP prior
variance at `x` is then exactly `lambda^2 s(x)^2`, so the posterior returns to
the GRAM prior far from data and tightens only near it, and the predictive
variance *is* the GP posterior variance — there is no fusion step to get wrong.
`lambda` is a single inflation factor for GRAM ensemble mis-dispersion, fitted
by maximizing the restricted log marginal likelihood.

This changes the mean as well. In the noise-free limit,

```text
mu(x*) = s(x*) * interp( y_i / s(x_i) )
```

so residuals transfer in units of the local prior sigma rather than as an
unchanged log-density offset at every altitude.
`--amplitude-mode stationary` reproduces the legacy behaviour.

## Scope

The implemented study makes the favorable assumptions requested for a
preliminary test:

- aerocapture and EDL stay near the same ground track
- the time gap is a few hours
- trajectory knowledge is perfect
- aerocapture yields direct noisy density samples every `2 s`
- measurement noise is `N(0, (0.05 rho_truth)^2)`
- the GP is held fixed after the aerocapture pass
- there is no onboard density filter during EDL; the baseline is GRAM alone
- for the climatology truth, the optional time surrogate uses GRAM's timescale;
  day-specific `merra2_native` instead supplies its own time evolution (see
  [Time](#time))

The estimator compares three state parameterizations:

- `log_density`
- `density_scale_factor`
- `log_density_scale_factor`

and four kernels:

- squared exponential
- Matérn `3/2`
- Matérn `5/2`
- `gram_exponential`, GRAM's own separable-exponential correlation model on
  physical distance with its tabulated altitude-dependent scales

## Layout

- [`main.jl`](main.jl): study driver
- [`corridor.jl`](corridor.jl): synthetic aerocapture and EDL paths
- [`truth_sources.jl`](truth_sources.jl): pluggable truth fields
- [`merra2.jl`](merra2.jl): reader for the vendored MERRA-2 binary climatology grids
- [`merra2_native.jl`](merra2_native.jl): reader for day-specific MERRA-2 model-level netCDF granules
- [`fetch_merra2_native.sh`](fetch_merra2_native.sh) and [`verify_merra2_native.jl`](verify_merra2_native.jl): download and post-download checks
- [`gram_correlation.jl`](gram_correlation.jl): GRAM's tabulated correlation scales and the chordal metric
- [`prior_sources.jl`](prior_sources.jl): swappable onboard prior (GRAM nominal or NRLMSISE-00)
- [`accel_filter.jl`](accel_filter.jl) and [`compare_filter.jl`](compare_filter.jl): in-situ density-ratio filter and its comparison against the GP
- [`measure_merra2_correlation.jl`](measure_merra2_correlation.jl): measures MERRA-2's empirical correlation structure
- [`gram_prior.jl`](gram_prior.jl): seeded GRAM nominal and dispersed prior sampling
- [`gp_models.jl`](gp_models.jl): GP residual models and target transforms
- [`scoring.jl`](scoring.jl): RMSE, NLPD, and coverage metrics
- [`plot_results.jl`](plot_results.jl): summary plotting

## Project Setup

This study uses its own Julia environment for `SpaceAGORA.jl` and
`GRAMSuite.jl`.

Instantiate it with:

```text
julia --project=benchmarks/studies/edl_aerocapture_gp_uncertainty -e "using Pkg; Pkg.instantiate()"
```

The study project declares:

- `SpaceAGORA` from the local repository root
- `GRAMSuite` from `data/GRAMSuite.jl`

You still need:

- a working local GRAM asset install under `data/GRAMSuite.jl/GRAM Suite 2.0`
- a usable native GRAM build
- the MERRA-2 grids under `.../Earth/data/MERRA2data`, or `SPACEAGORA_MERRA2_PATH`
  pointing at them, for the default `merra2` truth source

## Running

Default run:

```text
julia --project=benchmarks/studies/edl_aerocapture_gp_uncertainty benchmarks/studies/edl_aerocapture_gp_uncertainty/main.jl
```

Useful options:

```text
--out-dir output/edl_aerocapture_gp_uncertainty
--seed 42
--truth-seed 10042
--prior-seed 20042
--n-dispersion 24
--case-limit 1
--save-pointwise true
--lat-scale-deg 20.5
--lon-scale-deg 21.8
--alt-scale-km 18.33
--mean-basis none,constant,linear_alt
--kernels squared_exponential,matern32,matern52,gram_exponential
--gram-scale-ref-alt-km 35.0
--kernel-time-scale-hours auto
--amplitude-mode prior_scaled
--merra2-time-decorrelation true
--aerocapture-exit-alt-km 60.0
--truth-source merra2
--merra2-native-dir <path>
--merra2-hour-code 0
--merra2-dispersion 1.0
--merra2-blend-width-km 10.0
--truth-epoch-shift-hours 36.0
--field-bias 0.08
--field-amplitude 0.10
--field-lat-scale-deg 6.0
--field-lon-scale-deg 12.0
--field-alt-scale-km 25.0
```

## Outputs

The driver writes:

- `summary_metrics.csv`
- `kernel_comparison.csv`, averaged by parameterization, kernel, and mean basis
- `mean_function_fits.csv`, the fitted GLS mean coefficients per case
- `cases.csv`, including the truth source and whether it is position-indexed
- `pointwise_predictions.csv` when enabled
- `RESULTS.md`, generated in this study directory; it is a run artifact, not a tracked source file

`plot_results.jl` reads the summary and pointwise CSV files and produces bar
charts for weighted metrics plus a representative density-profile comparison
and relative-error profile. Pointwise output must be enabled for the profile
plots. Generated figures are saved under `plots/` in this study directory by
default.

## Method Notes

The onboard prior uses the nominal GRAM density with `--prior-seed`; its
dispersed ensemble uses GRAM's seeded `perturbedDensity` with consecutive,
different seeds. The defaults derived from `--seed` are distinct
(`seed + 10_000` and `seed + 20_000`) and are recorded in `cases.csv`. The
GRAM-derived truth options are synthetic same-model experiments. The MERRA-2
options instead use the reanalysis constructions described here, including
their GRAM fallback and taper; they are not independent flight validation.

The GP models a residual on top of the GRAM prior mean. The dispersed ensemble
provides the pointwise prior variance for the transformed variables. In the
default `prior_scaled` mode that variance enters the kernel, and prediction uses
the resulting GP posterior variance directly. Only the legacy `stationary` mode
combines an additive mean correction with inverse-variance fusion. Both modes
need error and coverage assessment; this preliminary study does not establish a
flight estimator design.

The EDL weighting profile emphasizes altitudes near a nominal Earth peak
dynamic-pressure region, implemented here as a Gaussian weight centered near
`35 km`.

## `merra2_native`: Day-Specific Model-Level Reanalysis

The seam above is a coverage problem. GRAM ships the MERRA-2 **pressure-level**
grid — Table 4.1 of the MERRA-2 File Specification, `1000 hPa` to `0.1 hPa`,
exactly the `42` levels in `merra2.jl` — averaged into monthly time-of-day
climatologies. The native product supplies a different vertical grid and
time coverage:

| | `merra2` (vendored climatology) | `merra2_native` (M2I3NVASM) |
|---|---|---|
| vertical grid | `42` pressure levels | `72` hybrid sigma-p model levels |
| data ceiling | `0.1 hPa`, about `64 km` | `0.01 hPa` (PTOP), about `80 km` |
| time | monthly climatology by time-of-day slot | `3-hourly` analysis, per day |
| weather | none — averaged out | real |
| level order | bottom-up | **top-down**, level 1 is the model top |

With `--truth-source merra2_native`, day-specific analysis supplies the time
evolution, so this source uses neither the synthetic dispersion term nor the
AR(1) surrogate. Data coverage and the applied anomaly are separate: the code
caps the anomaly at the smaller of the local data ceiling minus `1 km` and
`--merra2-native-anomaly-top-km` (default `52 km`). Above that cap it holds the
boundary anomaly and tapers it to zero over `--merra2-blend-width-km` (default
`10 km`). When the configured cap is limiting, truth returns to GRAM nominal
at `62 km`, despite the native data extending higher. The current cap and taper
are study assumptions, not measurements of the upper atmosphere.

### Getting the data

Granules are not vendored — `~2.1 GB` each, and GES DISC requires a NASA
Earthdata login:

```text
./benchmarks/studies/edl_aerocapture_gp_uncertainty/fetch_merra2_native.sh \
    2024-03-20 2024-03-21 2024-10-05 2024-10-06 2025-03-21 2025-03-22
```

A `6 hr` gap from an `18:00 UTC` anchor crosses midnight, which is why each
anchor needs the following day too. `--dry-run` prints the URLs without
fetching, and the script explains the Earthdata authorization step that causes
most `401`s. GES DISC's spatial subsetter cuts a corridor-sized window to a few
tens of MB per day if bandwidth is tight.

Then, before trusting any result:

```text
julia --project=benchmarks/studies/edl_aerocapture_gp_uncertainty \
      benchmarks/studies/edl_aerocapture_gp_uncertainty/verify_merra2_native.jl
```

### What is verified, and what is not

The reader's logic is pinned by `runtests.jl`, which writes a synthetic granule
carrying the documented schema — `tzyx` variables presented as
`(lon, lat, lev, time)`, `72` top-down levels, `_FillValue = 1e15` — with a
planted atmosphere whose log-density is linear in latitude, longitude, time and
altitude. Every interpolation the reader performs is exact on that field, so
recovery is checked to `1e-6` relative. That pins the level flip, the fill
handling, the bilinear and log-linear interpolations and the time axis.

Two things cannot be settled without real granules, and
`verify_merra2_native.jl` reports on both:

- **Whether `H` is geopotential or geometric height.** The reader assumes
  geopotential and converts, a `~1 km` difference at `80 km` — not negligible
  against an `18 km` vertical correlation scale. Check 3 prints both readings
  against GRAM's geometric-altitude profile; the wrong one shows a systematic
  ramp with altitude.
- **Whether a subsetted file kept its level ordering.** The specification warns
  that some post-processors flip the vertical grid. The reader asserts the level
  count, and check 2 confirms density falls with altitude.

## Comparing a run

The driver writes case definitions, summary metrics, fitted means and optional
pointwise predictions. Use `--report-only true` to regenerate the report from
a preserved run. Compare weighted log RMSE, predictive density and interval
coverage together, retaining truth source, sensing band, time gap and ground
track offset with every row. A lower error alone does not establish calibrated
uncertainty.

[`compare_filter.jl`](compare_filter.jl) compares the GP with a scalar
density-ratio filter in the style of Tracy, Falcone and Manchester, *Robust
Entry Guidance with Atmospheric Adaptation*. The filter here receives drag
acceleration directly; it does not reproduce the paper's navigation input.
The corridor is kinematic and does not respond to density. At-vehicle and
forward-trajectory density errors therefore cannot establish closed-loop
landing or parachute-deployment performance. Keep this distinction when
interpreting either the filter or a GP/filter combination.

## Known Remaining Issues

These limitations bound what a run can establish:

- The aerocapture pass bottoms out at `60 km` while the `q`-weight profile is
  centered at `35 km`, so the sensed and scored bands do not overlap.
  `--aerocapture-exit-alt-km` now exposes this and
  allows a sensing-depth comparison, but does not itself validate the corridor.
- For the vendored-climatology truth, the anomaly is sampled only up to the
  local data ceiling minus `1 km`. The taper above that boundary adds no new
  atmospheric information and eventually returns truth to GRAM nominal.
  Upper-atmosphere inference therefore needs a separately justified truth
  construction rather than treating the taper as measured atmosphere.
- `gram_exponential` freezes GRAM's altitude-dependent scales at a single
  reference altitude, because a kernel with position-dependent scales is not
  positive definite without a Gibbs-style normalization that has no clean form
  for the exponential. Over `30-60 km` GRAM's `Lh` spans `2240-2318 km` and `Lz`
  spans `17.4-19.3 km`, so the approximation is mild, but it is an approximation.
- The AR(1) time decorrelation applies to the *whole* anomaly, including the
  `log(rho_m2 / rho_gram)` model-difference component. A genuine systematic bias
  between GRAM and MERRA-2 would persist rather than decay, so with
  `--merra2-dispersion 0` the construction is conservative: it decorrelates
  something that in reality would partly hold. Splitting persistent model bias
  from decaying weather would need a truth source that separates them.
- `Lt` is GRAM's scale for its *small-scale* perturbation model. A synoptic
  density anomaly plausibly persists longer and a gravity-wave component
  decorrelates in well under an hour, so a single `Lt` is a coarse stand-in for
  a spectrum of timescales.
- Kernel length scales are set from the CLI. `stationary` uses detrended
  sample variance for amplitude; `prior_scaled` fits its inflation factor
  with restricted marginal likelihood. Neither fits the full spatial kernel.
- For `merra2_native`, the default anomaly cap is `52 km`, followed by a
  `10 km` taper. A pass entering at `125 km` therefore includes an upper
  portion where this construction supplies only GRAM nominal, even though
  native MERRA-2 data reaches higher. Changing the cap or adding an
  upper-atmosphere truth model needs separate scientific justification;
  lowering the entry changes the scenario rather than validating that region.
- Nothing here is closed-loop. The comparison against the Tracy/Falcone/
  Manchester filter scores open-loop density prediction; this framework
  cannot measure parachute-deployment miss distance.
- `lambda` is fitted on the sensed band and applied to the scored band. On this
  corridor they need not have the same uncertainty calibration. A single
  scalar cannot express an altitude-dependent change, and no sensing pass
  measures ensemble calibration at an altitude it never reaches.
- `weighted_rmse` is an absolute-density metric, and on the default corridor the
  denser lower atmosphere can dominate it. Inspect `weighted_rmse_log` as
  well. The `q` weights themselves are a Gaussian in altitude with a `0.25`
  floor rather than a dynamic-pressure profile computed as `0.5 rho V^2`.
- `log_density` and `log_density_scale_factor` are the same estimator and score
  the same mathematical quantity; compare their numerical outputs in the run.

## Verification Status

Run the seed-plan and driver-load smoke tests, then a reduced native-GRAM
single-case run with `--n-dispersion 4` and `--save-pointwise false` to confirm:

- the truth, nominal, and ensemble GRAM seeds produce distinct realizations
- GRAM nominal and dispersed point sampling work
- the generated `summary_metrics.csv` looks numerically sane
