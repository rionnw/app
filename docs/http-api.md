# min2phase HTTP API

This document defines the first HTTP interface for running `min2phase` as a
long-lived local service. The server should initialize solver tables once at
startup, then accept JSON requests over HTTP so callers do not pay process
startup and table initialization costs for every solve.

## Design Goals

- Keep the API small and close to the current C++ public API:
  `verify`, `fromScramble`, `solve`, and `solve2L`.
- Use synchronous request/response endpoints for v1. No job queue, persistence,
  authentication, or streaming.
- Make every solver request bounded by existing solver options such as
  `maxDepth`, `probeMax`, and `probeMin`.
- Use stable string status names in JSON while preserving the numeric
  `VerifyError` codes already exposed by the library.

## Server Lifecycle

The server should warm the solver tables before reporting itself ready:

- `min2phase::Search::init()`
- `min2phase::Search2L::init()`

Until initialization completes, solve endpoints return HTTP `503` with
`error.code = "not_ready"`. `GET /v1/health` remains available during startup.

Default bind settings for the future executable should be local-only:

- host: `127.0.0.1`
- port: `8080`

## Conventions

- Base path: `/v1`
- Content type: `application/json; charset=utf-8`
- Request bodies must be JSON objects.
- Field names use `camelCase`.
- A request that accepts cube input must provide exactly one of:
  - `facelets`: 54-character facelet cube string
  - `scramble`: scramble move string, converted with `fromScramble`
- Domain results use `ok` and `status` in the JSON body. HTTP status codes are
  reserved for transport, request-shape, and server-readiness failures.

### HTTP Status Codes

| HTTP status | Meaning |
| --- | --- |
| `200` | Request was accepted and the min2phase operation completed. Check `ok` and `status` for domain success. |
| `400` | Malformed JSON, missing required fields, mutually exclusive fields, or invalid option type/range. |
| `404` | Unknown endpoint. |
| `405` | Unsupported method for a known endpoint. |
| `415` | Request body is not JSON where JSON is required. |
| `503` | Solver tables are not ready yet. |
| `500` | Unexpected server error. |

### Error Envelope

Transport and request-shape errors use this shape:

```json
{
  "error": {
    "code": "bad_request",
    "message": "exactly one of facelets or scramble is required",
    "details": {
      "fields": ["facelets", "scramble"]
    }
  }
}
```

Recommended error codes:

- `bad_request`
- `unsupported_media_type`
- `not_found`
- `method_not_allowed`
- `not_ready`
- `internal_error`

## Status Names

### Verify Status

| Numeric code | JSON name |
| --- | --- |
| `0` | `ok` |
| `1` | `invalid_color_count` |
| `2` | `invalid_edge` |
| `3` | `invalid_flip` |
| `4` | `invalid_corner` |
| `5` | `invalid_twist` |
| `6` | `invalid_parity` |

### Solve Status

| C++ status | JSON name |
| --- | --- |
| `SolveStatus::Ok` | `ok` |
| `SolveStatus::InvalidCube` | `invalid_cube` |
| `SolveStatus::NoSolution` | `no_solution` |
| `SolveStatus::ProbeLimit` | `probe_limit` |
| `SolveStatus::NotImplemented` | `not_implemented` |

## Endpoints

### `GET /v1/health`

Returns process and table readiness. This endpoint does not require the solver
tables to be ready.

Response `200`:

```json
{
  "status": "ready",
  "ready": true,
  "tables": {
    "search": true,
    "search2l": true
  }
}
```

During startup, `status` is `starting`, `ready` is `false`, and one or both
table flags may be `false`.

### `POST /v1/verify`

Checks whether a facelet cube string is valid.

Request:

```json
{
  "facelets": "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"
}
```

Response `200`:

```json
{
  "ok": true,
  "status": "ok",
  "verify": {
    "code": 0,
    "name": "ok"
  }
}
```

Invalid cube response `200`:

```json
{
  "ok": false,
  "status": "invalid_color_count",
  "verify": {
    "code": 1,
    "name": "invalid_color_count"
  }
}
```

### `POST /v1/from-scramble`

Converts a scramble string to a facelet cube string.

Request:

```json
{
  "scramble": "R U R' U'"
}
```

Response `200`:

```json
{
  "facelets": "UULUUFUUFRRUBRRURRFFDFFUFFFDDRDDDDDDBLLLLLLLLBRRBBBBBB"
}
```

### `POST /v1/solve`

Runs the standard two-phase solver. This maps to
`min2phase::solve(facelets, SolveOptions)`.

Request using facelets:

```json
{
  "facelets": "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB",
  "options": {
    "maxDepth": 21,
    "probeMax": 100000000,
    "probeMin": 0,
    "useSeparator": false,
    "inverseSolution": false,
    "appendLength": false,
    "optimal": false
  }
}
```

Request using scramble:

```json
{
  "scramble": "R U R' U'",
  "options": {
    "maxDepth": 21
  }
}
```

All options are optional. Defaults match `min2phase::SolveOptions`.

Response `200`:

```json
{
  "ok": true,
  "status": "ok",
  "solution": "U  R  U' R'",
  "length": 4,
  "probes": 12,
  "estimatedCost": 0,
  "facelets": "UULUUFUUFRRUBRRURRFFDFFUFFFDDRDDDDDDBLLLLLLLLBRRBBBBBB"
}
```

For an already solved cube, `solution` is an empty string and `length` is `0`.

Domain failure response `200`:

```json
{
  "ok": false,
  "status": "probe_limit",
  "message": "probe limit exceeded",
  "solution": null,
  "length": -1,
  "probes": 100000000,
  "estimatedCost": 0,
  "verify": {
    "code": 0,
    "name": "ok"
  }
}
```

If `status` is `invalid_cube`, `verify` contains the concrete verify error.
For `no_solution`, `probe_limit`, and `not_implemented`, `verify` may be
omitted or set to `{ "code": 0, "name": "ok" }` when cube validation passed.

### `POST /v1/solve2l`

Runs the Search2L solver. This maps to
`min2phase::solve2L(facelets, TwoLegSolveOptions)`.

Request:

```json
{
  "scramble": "R U R' U'",
  "options": {
    "maxDepth": 70,
    "probeMax": 10000000,
    "probeMin": 500
  }
}
```

All options are optional. Defaults match `min2phase::TwoLegSolveOptions`.

Response `200`:

```json
{
  "ok": true,
  "status": "ok",
  "solution": "(z1z0) U  (z1s1) y  (s0z1) x",
  "length": 3,
  "probes": 42,
  "estimatedCost": 18,
  "facelets": "UULUUFUUFRRUBRRURRFFDFFUFFFDDRDDDDDDBLLLLLLLLBRRBBBBBB"
}
```

Domain failure shape matches `/v1/solve`.

## Validation Rules

- `facelets` must be a string. Cube validity is reported by `/v1/verify` or by
  solve responses with `status = "invalid_cube"`.
- `scramble` must be a string. Empty scramble is allowed and represents the
  solved cube.
- Exactly one of `facelets` or `scramble` is required for `/v1/solve` and
  `/v1/solve2l`.
- `maxDepth` must be a positive integer.
- `probeMax` and `probeMin` must be non-negative integers.
- Boolean options must be JSON booleans.
- Unknown fields should be rejected with HTTP `400` so client mistakes surface
  early.

## Example Calls

```sh
curl -s http://127.0.0.1:8080/v1/health
```

```sh
curl -s http://127.0.0.1:8080/v1/from-scramble \
  -H 'content-type: application/json' \
  -d '{"scramble":"R U R'\'' U'\''"}'
```

```sh
curl -s http://127.0.0.1:8080/v1/solve \
  -H 'content-type: application/json' \
  -d '{"scramble":"R U R'\'' U'\''","options":{"maxDepth":21}}'
```

## Implementation Notes

- Initialize solver tables once before accepting solve traffic.
- Create a fresh `Search` or `Search2L` instance per request through the public
  API wrappers.
- Keep the HTTP layer separate from solver code:
  - request parsing and JSON validation
  - conversion from HTTP DTOs to `SolveOptions` / `TwoLegSolveOptions`
  - conversion from `SolveResult` / `VerifyError` to response DTOs
- Start with local-only binding. Exposing this service on a network should be a
  separate product decision because solve requests can be CPU intensive.
