# kinematics_properties

The [Hegel](https://hegel.dev) generators that the property tests of
[`chain_kinematics`](../chain_kinematics) and [`srs_model`](../srs_model) share,
and the tolerance their properties compare at. A crate takes it as a
dev-dependency, beside `hegeltest` for the `#[hegel::test]` attribute:

```toml
[dev-dependencies]
hegeltest = { version = "0.50", features = ["static-engine"] }
kinematics_properties = { path = "../kinematics_properties" }
```

| Item | What it is |
|---|---|
| `angles()` | draws an angle in `[-π, π]`, both ends included |
| `unit_vectors()` | draws a unit vector, uniform on the sphere |
| `rotations()` | draws any rotation: any angle about any axis |
| `configs(limits)` | draws a configuration inside `limits`, bounds included |
| `TOLERANCE` | the largest miss a property allows, in metres or radians |

A generator that only one crate draws from, such as `chain_kinematics`' robots of
any serial shape or `srs_model`'s arm sides, lives in that crate's
`tests/common/properties.rs`.

`HEGEL_TEST_CASES` sets the cases each property runs, 100 by default.
